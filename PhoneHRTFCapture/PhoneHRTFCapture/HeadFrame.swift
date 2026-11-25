//
//  HeadFrame.swift
//  PhoneHRTFCapture
//
//  Per-subject calibration of the AprilTag head belt, and speaker direction in
//  head coordinates.
//
//  Belt layout: 7 tags across the front half, every 30 deg. Tag ID k sits at
//  nominal azimuth (k - 3) * 30 deg, counter-clockwise seen from above (SOFA
//  convention, +90 = left):
//    ID 0 = -90 (over the RIGHT ear canal), 1 = -60, 2 = -30, 3 = 0 (front),
//    4 = +30, 5 = +60, 6 = +90 (over the LEFT ear canal).
//  Only IDs 0 and 6 need careful placement (directly above the tragus); every
//  other tag is located by calibration, so its exact position does not matter.
//
//  Calibration (TagMapBuilder): with the head upright and still-ish, move the
//  phone around the head so neighbouring tags are seen in the same frames. Each
//  frame's relative tag poses are independent of head motion, so the map is
//  built by chaining co-visible tags into one rigid "map" frame. Gravity is
//  recorded too.
//
//  Head frame (HeadFrame): origin = midpoint of the ear tags, lowered by the
//  belt height above the ear canals; z up (against gravity during calibration);
//  y towards the left ear tag; x = y cross z (forward). Azimuth is atan2(y, x)
//  (0 front, +90 left), elevation atan2(z, horizontal).
//
//  Measurement (HeadFrame.locate): rigid-fit every visible mapped tag's corners
//  -> camera-to-head transform -> speaker position in the head frame. This uses
//  only the current frame, so the head may move freely between captures.
//

import Foundation
import simd

nonisolated enum Belt {
    /// Tag IDs worn on the belt; any other ID is ignored.
    static let ids = 0...6
    static let rightEarID = 0
    static let frontID = 3
    static let leftEarID = 6
    static func nominalAzimuth(_ id: Int) -> Double { Double((id - frontID) * 30) }
}

// MARK: Rigid transforms

nonisolated struct RigidTransform: Codable, Sendable {
    var rotation: simd_double3x3
    var translation: SIMD3<Double>

    static let identity = RigidTransform(rotation: matrix_identity_double3x3, translation: .zero)

    func apply(_ p: SIMD3<Double>) -> SIMD3<Double> { rotation * p + translation }
    func rotate(_ v: SIMD3<Double>) -> SIMD3<Double> { rotation * v }

    /// Least-squares fit of dst ~ R * src + t. Needs >= 3 non-collinear points.
    static func fit(from src: [SIMD3<Double>], to dst: [SIMD3<Double>]) -> (RigidTransform, rmsError: Double)? {
        guard src.count == dst.count, src.count >= 3 else { return nil }
        let s = src.flatMap { [$0.x, $0.y, $0.z] }
        let d = dst.flatMap { [$0.x, $0.y, $0.z] }
        var r = [Double](repeating: 0, count: 9), t = [Double](repeating: 0, count: 3)
        guard at_rigid_fit(s, d, Int32(src.count), &r, &t) == 1 else { return nil }
        let T = RigidTransform(
            rotation: simd_double3x3(rows: [SIMD3(r[0], r[1], r[2]), SIMD3(r[3], r[4], r[5]), SIMD3(r[6], r[7], r[8])]),
            translation: SIMD3(t[0], t[1], t[2]))
        let sq = zip(src, dst).reduce(0.0) { $0 + simd_length_squared(T.apply($1.0) - $1.1) }
        return (T, (sq / Double(src.count)).squareRoot())
    }

    // Codable as row-major 9 + 3 numbers.
    enum CodingKeys: String, CodingKey { case rotationRowMajor, translation }
    init(rotation: simd_double3x3, translation: SIMD3<Double>) {
        self.rotation = rotation; self.translation = translation
    }
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        let r = try c.decode([Double].self, forKey: .rotationRowMajor)
        rotation = simd_double3x3(rows: [SIMD3(r[0], r[1], r[2]), SIMD3(r[3], r[4], r[5]), SIMD3(r[6], r[7], r[8])])
        translation = try c.decode(SIMD3<Double>.self, forKey: .translation)
    }
    func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        let t = rotation.transpose  // columns of the transpose are the rows
        try c.encode([t.columns.0, t.columns.1, t.columns.2].flatMap { [$0.x, $0.y, $0.z] }, forKey: .rotationRowMajor)
        try c.encode(translation, forKey: .translation)
    }
}

// MARK: Calibration

nonisolated struct TagMapBuilder {
    /// Observations needed before a tag is trusted as a reference / for the head frame.
    static let minObservations = 8
    private static let maxObservations = 300
    /// Frames whose rigid fit to the existing map is worse than this are discarded.
    private static let maxFitRMS = 0.004   // m

    private(set) var anchorID: Int?
    /// Per tag: list of observed corner sets in the map frame.
    private var observations: [Int: [[SIMD3<Double>]]] = [:]
    private var upSum = SIMD3<Double>.zero
    private var upCount = 0

    var counts: [Int: Int] { observations.mapValues(\.count) }

    /// Tags usable as references (the anchor, plus tags with enough observations).
    var mappedIDs: Set<Int> {
        Set(observations.filter { $0.key == anchorID || $0.value.count >= Self.minObservations }.keys)
    }

    /// Robust (per-coordinate median) corners of every mapped tag, in the map frame.
    var corners: [Int: [SIMD3<Double>]] {
        var out: [Int: [SIMD3<Double>]] = [:]
        for id in mappedIDs { out[id] = Self.median(observations[id]!) }
        return out
    }

    mutating func add(_ frame: FrameObservation) {
        let tags = frame.tags
        guard !tags.isEmpty else { return }

        if anchorID == nil {
            // The map frame is the first-seen tag's own frame (prefer the front tag).
            let anchor = tags.min { abs($0.id - Belt.frontID) < abs($1.id - Belt.frontID) }!
            let s = CaptureSettings.tagSizeMeters / 2
            anchorID = anchor.id
            observations[anchor.id] = [[[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]]]
        }

        // Camera -> map from the tags already in the map.
        let reference = corners
        let known = tags.filter { reference[$0.id] != nil }
        guard !known.isEmpty,
              let (cameraToMap, rms) = RigidTransform.fit(from: known.flatMap(\.cameraCorners),
                                                          to: known.flatMap { reference[$0.id]! }),
              rms <= Self.maxFitRMS else { return }

        for tag in tags where tag.id != anchorID {
            var list = observations[tag.id, default: []]
            if list.count < Self.maxObservations {
                list.append(tag.cameraCorners.map(cameraToMap.apply))
                observations[tag.id] = list
            }
        }
        upSum += cameraToMap.rotate(frame.upInCamera)
        upCount += 1
    }

    enum Failure: LocalizedError {
        case missingEarTag(Int), noGravity
        var errorDescription: String? {
            switch self {
            case .missingEarTag(let id): "Tag \(id) (\(id == Belt.leftEarID ? "left" : "right") ear) not mapped yet"
            case .noGravity: "No calibration frames yet"
            }
        }
    }

    func finish(beltHeight: Double) throws -> HeadFrame {
        let mapped = corners
        guard let left = mapped[Belt.leftEarID] else { throw Failure.missingEarTag(Belt.leftEarID) }
        guard let right = mapped[Belt.rightEarID] else { throw Failure.missingEarTag(Belt.rightEarID) }
        guard upCount > 0 else { throw Failure.noGravity }

        let up = simd_normalize(upSum / Double(upCount))
        let l = left.reduce(.zero, +) / 4, r = right.reduce(.zero, +) / 4
        let across = l - r
        let leftAxis = simd_normalize(across - simd_dot(across, up) * up)
        let forward = simd_cross(leftAxis, up)
        let origin = (l + r) / 2 - up * beltHeight

        // map -> head: rows are the head axes expressed in map coordinates.
        let R = simd_double3x3(rows: [forward, leftAxis, up])
        let mapToHead = RigidTransform(rotation: R, translation: -(R * origin))

        var tagCorners: [Int: [SIMD3<Double>]] = [:]
        for (id, c) in mapped { tagCorners[id] = c.map(mapToHead.apply) }

        // Diagnostic: the angle between the belt plane and the gravity-horizontal plane.
        let centers = tagCorners.values.map { $0.reduce(.zero, +) / 4 }
        return HeadFrame(created: Date(),
                         tagCorners: tagCorners,
                         earTagSeparation: simd_length(across - simd_dot(across, up) * up),
                         beltHeight: beltHeight,
                         observationCounts: counts,
                         beltTiltDegrees: Self.planeTilt(centers))
    }

    private static func median(_ obs: [[SIMD3<Double>]]) -> [SIMD3<Double>] {
        (0..<4).map { i in
            func med(_ k: Int) -> Double {
                let v = obs.map { $0[i][k] }.sorted()
                return v.count % 2 == 1 ? v[v.count / 2] : (v[v.count / 2 - 1] + v[v.count / 2]) / 2
            }
            return SIMD3(med(0), med(1), med(2))
        }
    }

    /// Tilt of the best-fit plane through the tag centres relative to horizontal, degrees.
    private static func planeTilt(_ points: [SIMD3<Double>]) -> Double? {
        guard points.count >= 3 else { return nil }
        let c = points.reduce(.zero, +) / Double(points.count)
        // Fit z = a x + b y + c by least squares (head frame: z is up).
        var sxx = 0.0, sxy = 0.0, syy = 0.0, sxz = 0.0, syz = 0.0
        for p in points.map({ $0 - c }) {
            sxx += p.x * p.x; sxy += p.x * p.y; syy += p.y * p.y; sxz += p.x * p.z; syz += p.y * p.z
        }
        let det = sxx * syy - sxy * sxy
        guard abs(det) > 1e-12 else { return nil }
        let a = (sxz * syy - syz * sxy) / det, b = (syz * sxx - sxz * sxy) / det
        return atan(sqrt(a * a + b * b)) * 180 / .pi
    }
}

// MARK: Head frame

nonisolated struct HeadFrame: Codable, Sendable {
    let created: Date
    /// Corners of every mapped tag in head coordinates (x forward, y left, z up), metres.
    let tagCorners: [Int: [SIMD3<Double>]]
    /// Horizontal distance between the ear tags, metres (head width + belt thickness).
    let earTagSeparation: Double
    let beltHeight: Double
    let observationCounts: [Int: Int]
    let beltTiltDegrees: Double?

    /// Measured azimuth of each tag centre (deg) — compare with its nominal k*30.
    var tagAzimuths: [Int: Double] {
        tagCorners.mapValues { c in
            let p = c.reduce(.zero, +) / 4
            return atan2(p.y, p.x) * 180 / .pi
        }
    }

    /// Speaker direction for one frame, or nil if no mapped tag is visible.
    func locate(_ frame: FrameObservation, speakerInCamera: SIMD3<Double>) -> HeadPoseSample? {
        let visible = frame.tags.filter { tagCorners[$0.id] != nil }
        guard !visible.isEmpty,
              let (cameraToHead, rms) = RigidTransform.fit(from: visible.flatMap(\.cameraCorners),
                                                           to: visible.flatMap { tagCorners[$0.id]! })
        else { return nil }

        let speaker = cameraToHead.apply(speakerInCamera)
        let camera = cameraToHead.translation
        return HeadPoseSample(timestamp: frame.timestamp,
                              speaker: Direction(speaker),
                              camera: Direction(camera),
                              tagIDs: visible.map(\.id).sorted(),
                              fitRMSMillimetres: rms * 1000)
    }
}

nonisolated struct Direction: Codable, Sendable {
    let x: Double, y: Double, z: Double
    let azimuth: Double, elevation: Double, distance: Double

    init(_ p: SIMD3<Double>) {
        x = p.x; y = p.y; z = p.z
        distance = simd_length(p)
        azimuth = atan2(p.y, p.x) * 180 / .pi
        elevation = atan2(p.z, (p.x * p.x + p.y * p.y).squareRoot()) * 180 / .pi
    }
}

nonisolated struct HeadPoseSample: Codable, Sendable {
    let timestamp: TimeInterval
    /// Acoustic source (selected speaker) relative to the head centre.
    let speaker: Direction
    /// Camera lens relative to the head centre (for reference).
    let camera: Direction
    let tagIDs: [Int]
    let fitRMSMillimetres: Double
}

/// Speaker direction over a sweep, robust to per-frame tag-pose noise.
///
/// Per-frame pose jitter (0.3–1° at ~1 m with 32 mm tags, more with only 1–2 tags
/// in view) is much larger than real head movement of a seated, still participant,
/// so a max−min range over ~45 frames almost never stays under 1° even when the head
/// is perfectly still. Instead:
///   - the reported pose is the per-axis median,
///   - "moved" is real drift: median of the last third minus median of the first third,
///   - "spread" (interquartile range) measures detection noise and is shown, not used to reject.
nonisolated struct PoseSummary: Codable, Sendable {
    let sampleCount: Int
    let azimuth: Double, elevation: Double, distance: Double        // medians
    let azimuthDrift: Double, elevationDrift: Double, distanceDrift: Double
    let azimuthSpread: Double, elevationSpread: Double, distanceSpread: Double   // IQR
    let moved: Bool
    /// Pose noise is high (usually: too far away, or only 1–2 tags in view).
    let noisy: Bool

    static let noisySpreadDeg = 3.0

    init?(_ samples: [HeadPoseSample], thresholdDeg: Double, thresholdMeters: Double) {
        guard !samples.isEmpty else { return nil }
        let az = samples.map(\.speaker.azimuth), el = samples.map(\.speaker.elevation)
        let d = samples.map(\.speaker.distance)
        // Azimuth as deviations from the circular mean so +179 / -179 don't wrap.
        let s = az.reduce(0) { $0 + sin($1 * .pi / 180) }, c = az.reduce(0) { $0 + cos($1 * .pi / 180) }
        let meanAz = atan2(s, c) * 180 / .pi
        let azDev = az.map { remainder($0 - meanAz, 360) }
        sampleCount = samples.count
        azimuth = remainder(meanAz + Self.median(azDev), 360)
        elevation = Self.median(el)
        distance = Self.median(d)
        azimuthDrift = Self.drift(azDev)
        elevationDrift = Self.drift(el)
        distanceDrift = Self.drift(d)
        azimuthSpread = Self.iqr(azDev)
        elevationSpread = Self.iqr(el)
        distanceSpread = Self.iqr(d)
        moved = azimuthDrift > thresholdDeg || elevationDrift > thresholdDeg || distanceDrift > thresholdMeters
        noisy = azimuthSpread > Self.noisySpreadDeg || elevationSpread > Self.noisySpreadDeg
    }

    private static func median(_ v: [Double]) -> Double { quantile(v, 0.5) }

    private static func quantile(_ v: [Double], _ q: Double) -> Double {
        let s = v.sorted()
        let pos = q * Double(s.count - 1)
        let lo = Int(pos.rounded(.down)), hi = Int(pos.rounded(.up))
        return s[lo] + (s[hi] - s[lo]) * (pos - Double(lo))
    }

    private static func iqr(_ v: [Double]) -> Double { quantile(v, 0.75) - quantile(v, 0.25) }

    /// |median(last third) − median(first third)|; 0 with fewer than 3 samples.
    private static func drift(_ v: [Double]) -> Double {
        let n = v.count / 3
        guard n >= 1 else { return 0 }
        return abs(median(Array(v.suffix(n))) - median(Array(v.prefix(n))))
    }
}
