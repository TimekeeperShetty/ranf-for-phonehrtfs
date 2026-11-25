//
//  ForeheadEstimator.swift
//  PhoneHRTFCapture
//
//  Marker-free distance to the forehead from a single ARFrame:
//    1. Vision face landmarks locate the face (trying each image orientation,
//       so it works however the phone is held).
//    2. The forehead point is placed above the eyebrow midpoint, along the
//       face's own "up" axis (lips -> brows).
//    3. LiDAR scene depth is sampled in a patch around that point; the median
//       of confident samples gives z-depth, which is converted to straight-line
//       distance from the camera's optical centre using the intrinsics.
//

import ARKit
import Vision
import simd

struct ForeheadEstimate {
    /// Straight-line distance, camera optical centre -> forehead, metres.
    let distance: Double
    /// LiDAR depth along the optical axis at the forehead, metres.
    let depth: Double
    /// Interquartile range of the depth samples, metres (lower = cleaner patch).
    let spread: Double
    let sampleCount: Int
    /// Orientation that makes the image upright (face detected in this orientation).
    let orientation: CGImagePropertyOrientation
    /// Upright-image size and positions (pixels, top-left origin) for annotation.
    let uprightSize: CGSize
    let foreheadPoint: CGPoint
    let faceBox: CGRect
}

enum ForeheadError: LocalizedError {
    case noFrame, noDepth, noFace, insufficientDepth

    var errorDescription: String? {
        switch self {
        case .noFrame: "No camera frame yet"
        case .noDepth: "No LiDAR depth in this frame"
        case .noFace: "No face found — move closer or improve lighting"
        case .insufficientDepth: "Not enough reliable depth at the forehead — try 25–80 cm away"
        }
    }
}

enum ForeheadEstimator {
    /// Forehead offset above the eyebrow midpoint, as a fraction of face-box height.
    static let foreheadOffset = 0.12
    /// Half-size of the depth sampling patch, in depth-map pixels (256x192 map).
    static let patchRadius = 3

    private static let orientations: [CGImagePropertyOrientation] = [.right, .up, .left, .down]

    static func estimate(frame: ARFrame) throws -> ForeheadEstimate {
        guard let sceneDepth = frame.sceneDepth else { throw ForeheadError.noDepth }
        let image = frame.capturedImage
        let rawSize = CGSize(width: CVPixelBufferGetWidth(image), height: CVPixelBufferGetHeight(image))

        guard let (face, orientation) = detectFace(in: image) else { throw ForeheadError.noFace }

        let upright = uprightSize(rawSize, orientation)
        let (forehead, box) = foreheadLocation(face: face, uprightSize: upright)

        // Upright normalised (top-left) -> raw-sensor normalised (top-left).
        let rawN = rawNormalized(CGPoint(x: forehead.x / upright.width, y: forehead.y / upright.height),
                                 orientation)

        guard let (z, iqr, n) = sampleDepth(sceneDepth, at: rawN) else {
            throw ForeheadError.insufficientDepth
        }

        // z-depth -> Euclidean distance along the pixel's ray.
        let K = frame.camera.intrinsics
        let px = Double(rawN.x) * rawSize.width, py = Double(rawN.y) * rawSize.height
        let xn = (px - Double(K[2][0])) / Double(K[0][0])
        let yn = (py - Double(K[2][1])) / Double(K[1][1])
        let distance = z * (1 + xn * xn + yn * yn).squareRoot()

        return ForeheadEstimate(distance: distance, depth: z, spread: iqr, sampleCount: n,
                                orientation: orientation, uprightSize: upright,
                                foreheadPoint: forehead, faceBox: box)
    }

    // MARK: Face

    private static func detectFace(in image: CVPixelBuffer) -> (VNFaceObservation, CGImagePropertyOrientation)? {
        for orientation in orientations {
            let request = VNDetectFaceLandmarksRequest()
            try? VNImageRequestHandler(cvPixelBuffer: image, orientation: orientation).perform([request])
            if let face = request.results?.max(by: {
                $0.boundingBox.width * $0.boundingBox.height < $1.boundingBox.width * $1.boundingBox.height
            }) {
                return (face, orientation)
            }
        }
        return nil
    }

    /// Forehead point and face box in upright-image pixels, top-left origin.
    private static func foreheadLocation(face: VNFaceObservation, uprightSize s: CGSize) -> (CGPoint, CGRect) {
        let bb = face.boundingBox
        let box = CGRect(x: bb.minX * s.width, y: (1 - bb.maxY) * s.height,
                         width: bb.width * s.width, height: bb.height * s.height)

        func centroid(_ region: VNFaceLandmarkRegion2D?) -> SIMD2<Double>? {
            guard let pts = region?.pointsInImage(imageSize: s), !pts.isEmpty else { return nil }
            let sum = pts.reduce(SIMD2<Double>()) { $0 + SIMD2(Double($1.x), Double(s.height - $1.y)) }
            return sum / Double(pts.count)
        }

        let lm = face.landmarks
        guard let left = centroid(lm?.leftEyebrow), let right = centroid(lm?.rightEyebrow) else {
            // No landmarks: fall back to the upper part of the face box.
            return (CGPoint(x: box.midX, y: box.minY + 0.15 * box.height), box)
        }
        let brows = (left + right) / 2
        var up = SIMD2<Double>(0, -1)
        if let lips = centroid(lm?.outerLips), simd_length(brows - lips) > 1 {
            up = simd_normalize(brows - lips)
        }
        let p = brows + up * foreheadOffset * Double(box.height)
        return (CGPoint(x: p.x, y: p.y), box)
    }

    // MARK: Orientation mapping

    static func uprightSize(_ raw: CGSize, _ o: CGImagePropertyOrientation) -> CGSize {
        switch o {
        case .left, .right, .leftMirrored, .rightMirrored: CGSize(width: raw.height, height: raw.width)
        default: raw
        }
    }

    /// Maps a normalised top-left point in the upright image to the raw sensor image.
    static func rawNormalized(_ a: CGPoint, _ o: CGImagePropertyOrientation) -> CGPoint {
        switch o {
        case .right: CGPoint(x: a.y, y: 1 - a.x)       // raw is rotated 90° CCW from upright
        case .left:  CGPoint(x: 1 - a.y, y: a.x)       // raw is rotated 90° CW from upright
        case .down:  CGPoint(x: 1 - a.x, y: 1 - a.y)
        default:     a
        }
    }

    // MARK: Depth

    /// Median depth (m), IQR (m) and sample count in a patch around a raw-normalised point.
    private static func sampleDepth(_ sceneDepth: ARDepthData, at p: CGPoint) -> (Double, Double, Int)? {
        let depthMap = sceneDepth.depthMap
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        let conf = sceneDepth.confidenceMap
        if let conf { CVPixelBufferLockBaseAddress(conf, .readOnly) }
        defer { if let conf { CVPixelBufferUnlockBaseAddress(conf, .readOnly) } }

        let w = CVPixelBufferGetWidth(depthMap), h = CVPixelBufferGetHeight(depthMap)
        guard let base = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
        let rowBytes = CVPixelBufferGetBytesPerRow(depthMap)
        let confBase = conf.flatMap(CVPixelBufferGetBaseAddress)
        let confRowBytes = conf.map(CVPixelBufferGetBytesPerRow) ?? 0

        let cx = Int(Double(p.x) * Double(w)), cy = Int(Double(p.y) * Double(h))
        var high: [Double] = [], medium: [Double] = []
        for y in max(0, cy - patchRadius)...min(h - 1, cy + patchRadius) {
            let row = (base + y * rowBytes).assumingMemoryBound(to: Float32.self)
            for x in max(0, cx - patchRadius)...min(w - 1, cx + patchRadius) {
                let d = Double(row[x])
                guard d.isFinite, d > 0 else { continue }
                let c = confBase.map { ($0 + y * confRowBytes).assumingMemoryBound(to: UInt8.self)[x] }
                    ?? UInt8(ARConfidenceLevel.high.rawValue)
                if c >= UInt8(ARConfidenceLevel.high.rawValue) { high.append(d) }
                else if c >= UInt8(ARConfidenceLevel.medium.rawValue) { medium.append(d) }
            }
        }
        let samples = high.count >= 10 ? high : high + medium
        guard samples.count >= 5 else { return nil }
        let sorted = samples.sorted()
        func q(_ f: Double) -> Double { sorted[min(sorted.count - 1, Int(f * Double(sorted.count - 1) + 0.5))] }
        return (q(0.5), q(0.75) - q(0.25), sorted.count)
    }
}
