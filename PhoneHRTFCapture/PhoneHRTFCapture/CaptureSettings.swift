//
//  CaptureSettings.swift
//  PhoneHRTFCapture
//
//  User-measured geometry, persisted in UserDefaults. SettingsView edits the same
//  keys with @AppStorage; everything else reads them through this enum (safe from
//  any thread).
//

import Foundation
import simd

nonisolated enum CaptureSettings {
    enum Key {
        static let tagSizeMM = "tagSizeMM"
        static let beltHeightMM = "beltHeightAboveEarCanalMM"
        static let moveThresholdDeg = "moveThresholdDeg"
        static let moveThresholdCM = "moveThresholdCM"
        static let targetToleranceDeg = "targetToleranceDeg"
        /// Speaker offset from the rear (main/wide) camera lens, per output channel.
        static func speakerRightMM(_ ch: Int) -> String { "speaker\(ch)RightMM" }
        static func speakerDownMM(_ ch: Int) -> String { "speaker\(ch)DownMM" }
        static func speakerFrontMM(_ ch: Int) -> String { "speaker\(ch)TowardScreenMM" }
    }

    enum Default {
        static let tagSizeMM = 32.0      // matches apriltag_36h11_sheet.pdf
        static let beltHeightMM = 0.0
        static let moveThresholdDeg = 1.5
        static let moveThresholdCM = 1.0
        static let targetToleranceDeg = 2.0
    }

    private static func double(_ key: String, _ fallback: Double) -> Double {
        UserDefaults.standard.object(forKey: key) == nil ? fallback : UserDefaults.standard.double(forKey: key)
    }

    static var tagSizeMeters: Double { double(Key.tagSizeMM, Default.tagSizeMM) / 1000 }
    static var beltHeightMeters: Double { double(Key.beltHeightMM, Default.beltHeightMM) / 1000 }
    static var moveThresholdDeg: Double { double(Key.moveThresholdDeg, Default.moveThresholdDeg) }
    static var moveThresholdMeters: Double { double(Key.moveThresholdCM, Default.moveThresholdCM) / 100 }
    static var targetToleranceDeg: Double { double(Key.targetToleranceDeg, Default.targetToleranceDeg) }

    /// Offset (right, down, toward screen) in mm, as the user measured it with the
    /// phone in portrait and the screen facing them.
    static func speakerOffsetMM(channel: Int) -> SIMD3<Double> {
        SIMD3(double(Key.speakerRightMM(channel), 0),
              double(Key.speakerDownMM(channel), 0),
              double(Key.speakerFrontMM(channel), 0))
    }

    static func speakerOffsetIsSet(channel: Int) -> Bool {
        speakerOffsetMM(channel: channel) != .zero
    }

    /// Speaker position in the camera frame used by AprilTag poses
    /// (OpenCV: x right / y down / z forward in the *sensor* image, metres).
    ///
    /// ARFrame.capturedImage is landscape; a portrait-upright view is that image
    /// rotated by EXIF orientation 6 (.right). So sensor +x = towards the bottom of
    /// the phone, sensor +y = towards the phone's left (seen from the screen), and
    /// +z = out of the back of the phone.
    static func speakerInCamera(channel: Int) -> SIMD3<Double> {
        let o = speakerOffsetMM(channel: channel) / 1000
        return SIMD3(o.y, -o.x, -o.z)
    }
}
