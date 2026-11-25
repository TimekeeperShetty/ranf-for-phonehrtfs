//
//  AprilTagDetector.swift
//  PhoneHRTFCapture
//
//  AprilTag 36h11 detection + per-tag pose on ARFrame.capturedImage, via the
//  vendored C library (AprilTag/). Not thread-safe: use one instance per queue.
//

import ARKit
import CoreVideo
import simd

/// One tag seen in one frame.
nonisolated struct TagObservation: Sendable, Codable {
    let id: Int
    let hamming: Int
    let decisionMargin: Double
    /// Image pixels (sensor orientation), apriltag corner order.
    let pixelCorners: [SIMD2<Double>]
    /// The same corners in the camera frame (x right, y down, z forward), metres.
    let cameraCorners: [SIMD3<Double>]
    let poseError: Double
    /// Angle between the tag's normal and the line of sight, degrees (0 = square-on).
    let viewingAngle: Double

    var cameraCenter: SIMD3<Double> { cameraCorners.reduce(.zero, +) / 4 }
}

/// Everything the main actor needs from one processed frame.
nonisolated struct FrameObservation: Sendable, Codable {
    let timestamp: TimeInterval
    let tags: [TagObservation]
    /// World "up" (opposite gravity) in the camera frame.
    let upInCamera: SIMD3<Double>
    let intrinsics: [Double]          // fx, fy, cx, cy
    let cameraTransform: [Float]      // ARKit, 4x4 column-major
}

nonisolated final class AprilTagDetector: @unchecked Sendable {
    /// IDs worn on the belt; anything else (incl. sheet tags 7-11) is ignored.
    static let validIDs = Belt.ids
    static let maxViewingAngle = 70.0   // degrees; beyond this, pose is unreliable

    private let handle: OpaquePointer?
    private var buffer = [ATDetection](repeating: ATDetection(), count: 32)

    init(decimate: Float = 2, threads: Int32 = 2) {
        handle = at_detector_create(decimate, threads)
    }

    deinit { at_detector_destroy(handle) }

    func process(_ frame: ARFrame, tagSize: Double) -> FrameObservation {
        let K = frame.camera.intrinsics
        let fx = Double(K[0][0]), fy = Double(K[1][1]), cx = Double(K[2][0]), cy = Double(K[2][1])
        let tags = detect(frame.capturedImage, tagSize: tagSize, fx: fx, fy: fy, cx: cx, cy: cy)

        // ARKit camera space: x right, y up, z toward the viewer. Ours flips y and z.
        let T = frame.camera.transform
        func column(_ c: SIMD4<Float>) -> SIMD3<Double> { SIMD3(Double(c.x), Double(c.y), Double(c.z)) }
        let Rwc = simd_double3x3(column(T.columns.0), column(T.columns.1), column(T.columns.2))
        let upARKit = Rwc.transpose * SIMD3<Double>(0, 1, 0)   // worldAlignment = .gravity
        let up = SIMD3(upARKit.x, -upARKit.y, -upARKit.z)

        return FrameObservation(
            timestamp: frame.timestamp,
            tags: tags,
            upInCamera: simd_normalize(up),
            intrinsics: [fx, fy, cx, cy],
            cameraTransform: [T.columns.0, T.columns.1, T.columns.2, T.columns.3].flatMap { [$0.x, $0.y, $0.z, $0.w] })
    }

    private func detect(_ pixelBuffer: CVPixelBuffer, tagSize: Double,
                        fx: Double, fy: Double, cx: Double, cy: Double) -> [TagObservation] {
        guard let handle, CVPixelBufferGetPlaneCount(pixelBuffer) >= 1 else { return [] }
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        // Plane 0 of ARKit's YCbCr buffer is 8-bit luma: exactly what AprilTag wants.
        guard let base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0) else { return [] }
        let width = Int32(CVPixelBufferGetWidthOfPlane(pixelBuffer, 0))
        let height = Int32(CVPixelBufferGetHeightOfPlane(pixelBuffer, 0))
        let stride = Int32(CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0))

        let count = buffer.withUnsafeMutableBufferPointer { out in
            Int(at_detect(handle, base.assumingMemoryBound(to: UInt8.self), width, height, stride,
                          tagSize, fx, fy, cx, cy, out.baseAddress, Int32(out.count)))
        }

        let s = tagSize / 2
        let object: [SIMD3<Double>] = [[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]]
        return buffer.prefix(count).compactMap { d in
            let id = Int(d.id)
            guard Self.validIDs.contains(id) else { return nil }
            let c = Self.array(d.corners), r = Self.array(d.R), t = Self.array(d.t)
            let R = simd_double3x3(rows: [SIMD3(r[0], r[1], r[2]), SIMD3(r[3], r[4], r[5]), SIMD3(r[6], r[7], r[8])])
            let T = SIMD3(t[0], t[1], t[2])
            guard T.z > 0 else { return nil }

            let normal = R * SIMD3<Double>(0, 0, 1)
            let viewing = acos(min(1, abs(simd_dot(simd_normalize(normal), simd_normalize(T))))) * 180 / .pi
            guard viewing <= Self.maxViewingAngle else { return nil }

            return TagObservation(
                id: id,
                hamming: Int(d.hamming),
                decisionMargin: Double(d.decision_margin),
                pixelCorners: (0..<4).map { SIMD2(c[2 * $0], c[2 * $0 + 1]) },
                cameraCorners: object.map { R * $0 + T },
                poseError: d.pose_error,
                viewingAngle: viewing)
        }
    }

    private static func array<T>(_ tuple: T) -> [Double] {
        withUnsafeBytes(of: tuple) { Array($0.bindMemory(to: Double.self)) }
    }
}
