//
//  CaptureStore.swift
//  PhoneHRTFCapture
//
//  Saves each capture to Documents/Captures/<timestamp>/:
//    Forehead mode:
//      image.jpg      upright camera frame
//      annotated.jpg  with face box, forehead point and distance drawn on
//      capture.json   measurement + camera metadata
//    AprilTag mode:
//      capture.json   speaker direction in head coordinates (per frame during the
//                     sweep + summary), raw tag corners / intrinsics / ARKit pose
//                     for offline re-solving, and the belt calibration used
//  Belt calibrations are also saved on their own to Documents/Calibrations/<timestamp>.json.
//  The Documents folder is visible in the Files app and in Finder (iPhone > Files).
//

import ARKit
import CoreImage
import UIKit

struct ForeheadCapture {
    let estimate: ForeheadEstimate
    let annotated: UIImage
    let folder: URL
}

private struct CaptureRecord: Codable {
    let timestamp: String
    var method = "lidar-forehead"
    let distanceMeters: Double
    let depthMeters: Double
    let depthIQRMeters: Double
    let depthSamples: Int
    let foreheadPixel: [Double]       // upright image, top-left origin
    let faceBoxPixel: [Double]        // x, y, width, height (upright image)
    let uprightImageSize: [Double]
    let imageOrientation: UInt32      // CGImagePropertyOrientation raw value (raw sensor -> upright)
    let intrinsics: [Double]          // fx, fy, cx, cy for the raw sensor image
    let cameraTransform: [Float]      // 4x4 column-major, ARKit world frame
    var sweep = SweepRecord()
}

/// Sweep settings at capture time, so deconvolution can regenerate the exact signal.
struct SweepRecord: Codable {
    var sampleRate = SweepPlayer.sampleRate
    var startHz = SweepPlayer.startFrequency
    var endHz = SweepPlayer.endFrequency
    var durationSeconds = SweepPlayer.duration
    var fadeSeconds = SweepPlayer.fadeDuration
    var amplitude = SweepPlayer.amplitude
    var outputChannel = SweepPlayer.storedChannel.rawValue  // 0 = L, 1 = R (portrait)
    var hardwareSampleRate = SweepPlayer.lastHardwareSampleRate
}

/// One AprilTag-mode capture. Units: metres, degrees, seconds (system uptime, the
/// same clock as ARFrame.timestamp). Head frame: x forward, y left, z up, origin at
/// the ear-canal midpoint; azimuth counter-clockwise from front (SOFA convention).
struct MarkerCaptureRecord: Codable {
    var method = "apriltag-head-frame"
    let timestamp: String
    let target: TargetInfo?                // tracker position this sweep was taken for
    let accepted: Bool                     // false = moved / no tags: a redo follows
    let tagSizeMeters: Double
    let speakerChannel: Int                // 0 = L, 1 = R output channel (portrait)
    let speakerOffsetMM: SIMD3<Double>     // right, down, toward screen from the rear lens
    let speakerOffsetSet: Bool             // false = offset left at 0 (camera position used)
    let speakerInCamera: SIMD3<Double>     // same offset in the sensor camera frame, metres
    let moveThresholdDeg: Double
    let moveThresholdMeters: Double
    let sweepStartUptime: Double
    let sweepEndUptime: Double
    let summary: PoseSummary?              // nil if no tag was visible during the sweep
    let samples: [HeadPoseSample]
    let frames: [FrameObservation]
    let headFrame: HeadFrame
    var sweep = SweepRecord()
}

struct CalibrationRecord: Codable {
    let headFrame: HeadFrame
    let tagAzimuths: [Int: Double]
}

enum CaptureStore {
    static func save(frame: ARFrame, estimate e: ForeheadEstimate, date: Date = Date()) throws -> ForeheadCapture {
        let ci = CIImage(cvPixelBuffer: frame.capturedImage).oriented(e.orientation)
        guard let cg = CIContext().createCGImage(ci, from: ci.extent) else {
            throw CocoaError(.fileWriteUnknown)
        }
        let photo = UIImage(cgImage: cg)
        let annotated = annotate(photo, e)

        let stamp = ISO8601DateFormatter().string(from: date)
        let folder = URL.documentsDirectory
            .appending(path: "Captures")
            .appending(path: stamp.replacingOccurrences(of: ":", with: "-"))
        try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)

        try photo.jpegData(compressionQuality: 0.92)?.write(to: folder.appending(path: "image.jpg"))
        try annotated.jpegData(compressionQuality: 0.85)?.write(to: folder.appending(path: "annotated.jpg"))

        let K = frame.camera.intrinsics
        let T = frame.camera.transform
        let record = CaptureRecord(
            timestamp: stamp,
            distanceMeters: e.distance,
            depthMeters: e.depth,
            depthIQRMeters: e.spread,
            depthSamples: e.sampleCount,
            foreheadPixel: [e.foreheadPoint.x, e.foreheadPoint.y],
            faceBoxPixel: [e.faceBox.minX, e.faceBox.minY, e.faceBox.width, e.faceBox.height],
            uprightImageSize: [e.uprightSize.width, e.uprightSize.height],
            imageOrientation: e.orientation.rawValue,
            intrinsics: [K[0][0], K[1][1], K[2][0], K[2][1]].map(Double.init),
            cameraTransform: [T.columns.0, T.columns.1, T.columns.2, T.columns.3].flatMap { [$0.x, $0.y, $0.z, $0.w] }
        )
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        encoder.keyEncodingStrategy = .convertToSnakeCase
        try encoder.encode(record).write(to: folder.appending(path: "capture.json"))

        return ForeheadCapture(estimate: e, annotated: annotated, folder: folder)
    }

    // MARK: AprilTag mode

    static func saveMarker(_ record: MarkerCaptureRecord, date: Date) throws -> URL {
        let folder = try makeFolder("Captures", date: date)
        try encoder.encode(record).write(to: folder.appending(path: "capture.json"))
        return folder
    }

    static func saveCalibration(_ frame: HeadFrame) throws -> URL {
        let folder = URL.documentsDirectory.appending(path: "Calibrations")
        try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
        let url = folder.appending(path: stamp(frame.created) + ".json")
        try encoder.encode(CalibrationRecord(headFrame: frame, tagAzimuths: frame.tagAzimuths)).write(to: url)
        return url
    }

    private static func stamp(_ date: Date) -> String {
        ISO8601DateFormatter().string(from: date).replacingOccurrences(of: ":", with: "-")
    }

    private static func makeFolder(_ parent: String, date: Date) throws -> URL {
        let folder = URL.documentsDirectory.appending(path: parent).appending(path: stamp(date))
        try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
        return folder
    }

    private static var encoder: JSONEncoder {
        let e = JSONEncoder()
        e.outputFormatting = [.prettyPrinted, .sortedKeys]
        e.keyEncodingStrategy = .convertToSnakeCase
        e.dateEncodingStrategy = .iso8601
        return e
    }

    private static func annotate(_ photo: UIImage, _ e: ForeheadEstimate) -> UIImage {
        let format = UIGraphicsImageRendererFormat()
        format.scale = 1
        return UIGraphicsImageRenderer(size: photo.size, format: format).image { ctx in
            photo.draw(at: .zero)
            let line = max(3, photo.size.width / 300)

            UIColor.systemYellow.setStroke()
            let box = UIBezierPath(rect: e.faceBox)
            box.lineWidth = line
            box.stroke()

            let r = line * 5
            let dot = UIBezierPath(ovalIn: CGRect(x: e.foreheadPoint.x - r, y: e.foreheadPoint.y - r,
                                                  width: 2 * r, height: 2 * r))
            UIColor.systemRed.setFill()
            dot.fill()

            let text = String(format: "%.1f cm", e.distance * 100) as NSString
            let attrs: [NSAttributedString.Key: Any] = [
                .font: UIFont.monospacedDigitSystemFont(ofSize: photo.size.width / 14, weight: .bold),
                .foregroundColor: UIColor.white,
                .strokeColor: UIColor.black,
                .strokeWidth: -3,
            ]
            text.draw(at: CGPoint(x: e.faceBox.minX, y: max(0, e.faceBox.minY - photo.size.width / 10)),
                      withAttributes: attrs)
        }
    }
}
