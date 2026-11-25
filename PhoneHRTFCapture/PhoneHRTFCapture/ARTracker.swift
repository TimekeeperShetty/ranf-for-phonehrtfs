//
//  ARTracker.swift
//  PhoneHRTFCapture
//
//  Created by Abhijeet Shetty on 18/10/26.
//
//  Runs the rear camera (with LiDAR depth) through ARKit and measures where the
//  phone is relative to the head, either:
//    - forehead: one-shot, marker-free, LiDAR depth at the detected forehead
//      (ForeheadEstimator.swift), or
//    - marker:   AprilTag 36h11 head belt (AprilTagDetector.swift). A per-subject
//      calibration maps the belt (HeadFrame.swift); after that every frame gives
//      the speaker's azimuth / elevation / distance from the head centre.
//

import ARKit
import Combine
import os

final class ARTracker: NSObject, ObservableObject, ARSessionDelegate {
    enum Mode: String, CaseIterable, Identifiable {
        case forehead = "Forehead (LiDAR)"
        case marker = "AprilTag"
        var id: Self { self }
    }

    let session = ARSession()

    @Published var mode: Mode = .forehead {
        didSet {
            let markerMode = mode == .marker
            markerDetectionOn.withLock { $0 = markerMode }
            recent.removeAll()
            live = nil
            status = idleStatus
        }
    }
    @Published private(set) var depthAvailable = false
    @Published private(set) var status = "Starting…"

    // Marker mode state
    @Published private(set) var isCalibrating = false
    @Published private(set) var mapCounts: [Int: Int] = [:]
    @Published private(set) var mappedIDs: Set<Int> = []
    @Published private(set) var headFrame: HeadFrame?
    @Published private(set) var calibrationError: String?
    @Published private(set) var visibleIDs: [Int] = []
    /// Speaker direction averaged over the last `smoothingWindow`, with its spread.
    @Published private(set) var live: PoseSummary?
    @Published private(set) var isRecording = false
    @Published private(set) var lastResult: MarkerCaptureResult?

    // Position tracker: the planned azimuths, in order, and which one is next.
    @Published private(set) var targets: [TargetPoint] = ARTracker.makeTargets(.rightFirst)
    @Published private(set) var sequenceOrder: SequenceOrder = .rightFirst
    /// Index into `targets`; nil once every position has a clean capture. Tap a chip to change it.
    @Published var currentTarget: Int? = 0

    nonisolated private static let detectionInterval: TimeInterval = 1.0 / 15
    private static let smoothingWindow: TimeInterval = 0.5
    /// Frames older than this (relative to the newest) are dropped from `recent`.
    private static let liveTimeout: TimeInterval = 0.6

    private let delegateQueue = DispatchQueue(label: "PhoneHRTFCapture.marker", qos: .userInitiated)
    private let markerDetectionOn = OSAllocatedUnfairLock<Bool>(initialState: false)
    nonisolated(unsafe) private var lastDetectionTime: TimeInterval = 0   // delegateQueue only
    nonisolated private let detector = AprilTagDetector()                 // delegateQueue only

    private var builder = TagMapBuilder()
    private var recent: [HeadPoseSample] = []
    private var recordedSamples: [HeadPoseSample] = []
    private var recordedFrames: [FrameObservation] = []

    func start() {
        guard ARWorldTrackingConfiguration.isSupported else {
            status = "World tracking unsupported on this device"
            return
        }
        session.delegate = self
        session.delegateQueue = delegateQueue
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        depthAvailable = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
        if depthAvailable { config.frameSemantics.insert(.sceneDepth) }
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        status = idleStatus
    }

    private var idleStatus: String {
        switch mode {
        case .forehead: depthAvailable ? "Aim at the forehead and tap Position OK" : "LiDAR depth not available on this device"
        case .marker: headFrame == nil ? "Calibrate the head belt first" : "Point the camera at the belt"
        }
    }

    /// Estimates forehead distance from the current frame and saves the capture.
    func captureForehead() throws -> ForeheadCapture {
        guard let frame = session.currentFrame else { throw ForeheadError.noFrame }
        let estimate = try ForeheadEstimator.estimate(frame: frame)
        return try CaptureStore.save(frame: frame, estimate: estimate)
    }

    func pause() {
        session.pause()
    }

    // MARK: Calibration

    func startCalibration() {
        builder = TagMapBuilder()
        mapCounts = [:]
        mappedIDs = []
        headFrame = nil
        calibrationError = nil
        lastResult = nil
        isCalibrating = true
        status = "Move the phone slowly around the head"
    }

    func cancelCalibration() {
        isCalibrating = false
        status = idleStatus
    }

    func finishCalibration() {
        do {
            let frame = try builder.finish(beltHeight: CaptureSettings.beltHeightMeters)
            headFrame = frame
            isCalibrating = false
            calibrationError = nil
            recent.removeAll()
            status = idleStatus
            do {
                let url = try CaptureStore.saveCalibration(frame)
                print("CALIBRATION saved \(url.lastPathComponent)  ear separation \(String(format: "%.1f cm", frame.earTagSeparation * 100))")
            } catch {
                print("Calibration save failed: \(error)")
            }
        } catch {
            calibrationError = error.localizedDescription
        }
    }

    // MARK: Position tracker

    enum SequenceOrder: String, Codable {
        case rightFirst, leftFirst
        var label: String {
            self == .rightFirst ? "Right side first (0, −15 … −90, +15 … +90, 0)"
                                : "Left side first (0, +15 … +90, −15 … −90, 0)"
        }
    }

    /// 0, one side out to 90, the other side out to 90, then 0 again as a repeatability check.
    static func makeTargets(_ order: SequenceOrder) -> [TargetPoint] {
        let side = stride(from: 15.0, through: 90, by: 15)
        let first = order == .rightFirst ? side.map { -$0 } : Array(side)
        let azimuths = [0.0] + first + first.map { -$0 } + [0.0]
        return azimuths.enumerated().map { TargetPoint(id: $0.offset, azimuth: $0.element) }
    }

    func restartSequence(_ order: SequenceOrder) {
        sequenceOrder = order
        targets = Self.makeTargets(order)
        currentTarget = 0
    }

    private func recordTargetResult(index: Int, summary: PoseSummary, folder: URL?) {
        guard targets.indices.contains(index) else { return }
        targets[index].result = summary
        targets[index].folderName = folder?.lastPathComponent
        // Next unfinished target after this one, wrapping round to any skipped earlier.
        currentTarget = targets.indices.first { $0 > index && targets[$0].result == nil }
            ?? targets.indices.first { targets[$0].result == nil }
    }

    // MARK: Capture

    /// Plays the sweep, records the head-relative speaker pose for its whole
    /// duration, then saves the capture.
    func captureMarker(with sweep: SweepPlayer) {
        guard let headFrame, !isRecording, !sweep.isPlaying else { return }
        recordedSamples = []
        recordedFrames = []
        isRecording = true
        let targetIndex = currentTarget
        let target = targetIndex.map {
            TargetInfo(index: $0, azimuth: targets[$0].azimuth, order: sequenceOrder,
                       toleranceDeg: CaptureSettings.targetToleranceDeg)
        }
        let started = Date()
        let startUptime = ProcessInfo.processInfo.systemUptime
        sweep.play { [weak self] in
            guard let self else { return }
            self.isRecording = false
            let endUptime = ProcessInfo.processInfo.systemUptime
            let samples = self.recordedSamples.filter { $0.timestamp >= startUptime && $0.timestamp <= endUptime }
            let frames = self.recordedFrames.filter { $0.timestamp >= startUptime && $0.timestamp <= endUptime }
            let summary = PoseSummary(samples,
                                      thresholdDeg: CaptureSettings.moveThresholdDeg,
                                      thresholdMeters: CaptureSettings.moveThresholdMeters)
            let channel = SweepPlayer.storedChannel.rawValue
            let accepted = summary.map { !$0.moved } ?? false
            let record = MarkerCaptureRecord(
                timestamp: ISO8601DateFormatter().string(from: started),
                target: target,
                accepted: accepted,
                tagSizeMeters: CaptureSettings.tagSizeMeters,
                speakerChannel: channel,
                speakerOffsetMM: CaptureSettings.speakerOffsetMM(channel: channel),
                speakerOffsetSet: CaptureSettings.speakerOffsetIsSet(channel: channel),
                speakerInCamera: CaptureSettings.speakerInCamera(channel: channel),
                moveThresholdDeg: CaptureSettings.moveThresholdDeg,
                moveThresholdMeters: CaptureSettings.moveThresholdMeters,
                sweepStartUptime: startUptime,
                sweepEndUptime: endUptime,
                summary: summary,
                samples: samples,
                frames: frames,
                headFrame: headFrame)
            var folder: URL?
            var saveError: String?
            do { folder = try CaptureStore.saveMarker(record, date: started) } catch {
                saveError = error.localizedDescription
            }
            self.lastResult = MarkerCaptureResult(summary: summary, folder: folder, error: saveError)
            if accepted, saveError == nil, let targetIndex, let summary {
                self.recordTargetResult(index: targetIndex, summary: summary, folder: folder)
            }
            if let s = summary {
                print(String(format: "CAPTURE apriltag  az %.1f°  el %.1f°  dist %.1f cm  drift az %.2f° el %.2f° d %.2f cm  noise az %.2f° el %.2f°  n=%d%@%@",
                             s.azimuth, s.elevation, s.distance * 100, s.azimuthDrift, s.elevationDrift,
                             s.distanceDrift * 100, s.azimuthSpread, s.elevationSpread, s.sampleCount,
                             s.moved ? "  MOVED" : "", s.noisy ? "  NOISY" : ""))
            } else {
                print("CAPTURE apriltag: no tags visible during sweep")
            }
        }
        if !sweep.isPlaying { isRecording = false }   // playback failed to start
    }

    // MARK: ARSessionDelegate (runs on delegateQueue)

    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard markerDetectionOn.withLock({ $0 }) else { return }
        // Throttle; skipped frames return immediately so ARKit never backs up.
        guard frame.timestamp - lastDetectionTime >= Self.detectionInterval else { return }
        lastDetectionTime = frame.timestamp

        let observation = detector.process(frame, tagSize: CaptureSettings.tagSizeMeters)
        Task { @MainActor in self.ingest(observation) }
    }

    nonisolated func session(_ session: ARSession, didFailWithError error: Error) {
        let message = error.localizedDescription
        Task { @MainActor in self.status = "AR session failed: \(message)" }
    }

    // MARK: Per-frame processing (main actor)

    private func ingest(_ frame: FrameObservation) {
        guard mode == .marker else { return }
        visibleIDs = frame.tags.map(\.id).sorted()

        if isCalibrating {
            builder.add(frame)
            mapCounts = builder.counts
            mappedIDs = builder.mappedIDs
            status = frame.tags.isEmpty ? "No tags in view" : "Tags in view: \(visibleIDs.map(String.init).joined(separator: ", "))"
            return
        }

        guard let headFrame else { return }
        let channel = SweepPlayer.storedChannel.rawValue
        let sample = headFrame.locate(frame, speakerInCamera: CaptureSettings.speakerInCamera(channel: channel))

        if isRecording {
            recordedFrames.append(frame)
            if let sample { recordedSamples.append(sample) }
        }

        if let sample { recent.append(sample) }
        recent.removeAll { frame.timestamp - $0.timestamp > Self.smoothingWindow }
        if let newest = recent.last, frame.timestamp - newest.timestamp > Self.liveTimeout { recent.removeAll() }

        live = PoseSummary(recent, thresholdDeg: CaptureSettings.moveThresholdDeg,
                           thresholdMeters: CaptureSettings.moveThresholdMeters)
        status = sample == nil
            ? (frame.tags.isEmpty ? "No tags in view" : "Tags \(visibleIDs) not in calibration")
            : "Tracking with tags \(sample!.tagIDs.map(String.init).joined(separator: ", "))"
    }
}

/// One planned position in the tracker.
struct TargetPoint: Identifiable {
    let id: Int
    let azimuth: Double
    var result: PoseSummary?
    var folderName: String?
}

/// Which tracker target a capture was taken for (saved in capture.json).
struct TargetInfo: Codable {
    let index: Int
    let azimuth: Double
    let order: ARTracker.SequenceOrder
    let toleranceDeg: Double
}

/// What the UI shows after a marker capture.
struct MarkerCaptureResult {
    let summary: PoseSummary?
    let folder: URL?
    let error: String?
}
