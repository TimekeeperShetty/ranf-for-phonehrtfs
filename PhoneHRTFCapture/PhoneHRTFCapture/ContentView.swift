//
//  ContentView.swift
//  PhoneHRTFCapture
//
//  Created by Abhijeet Shetty on 18/10/26.
//

import ARKit
import SwiftUI

struct ContentView: View {
    @StateObject private var tracker = ARTracker()
    @StateObject private var sweep = SweepPlayer()

    @State private var lastCapture: ForeheadCapture?
    @State private var captureError: String?
    @State private var showingCapture = false
    @State private var showingSettings = false

    var body: some View {
        ZStack {
            CameraPreview(session: tracker.session)
                .ignoresSafeArea()

            Image(systemName: "plus")
                .font(.system(size: 32, weight: .light))
                .foregroundStyle(.white.opacity(0.8))

            VStack {
                HStack {
                    Picker("Mode", selection: $tracker.mode) {
                        ForEach(ARTracker.Mode.allCases) { Text($0.rawValue).tag($0) }
                    }
                    .pickerStyle(.segmented)
                    Button { showingSettings = true } label: { Image(systemName: "gearshape") }
                }
                .padding()
                .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
                .padding(.horizontal)

                Spacer()

                VStack(spacing: 12) {
                    speakerControls
                    Divider()
                    switch tracker.mode {
                    case .forehead: foreheadControls
                    case .marker: markerControls
                    }
                }
                .padding()
                .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 20))
                .padding()
            }
        }
        .onAppear { tracker.start() }
        .onDisappear { tracker.pause() }
        .sheet(isPresented: $showingSettings) { SettingsView() }
        .sheet(isPresented: $showingCapture) {
            if let image = lastCapture?.annotated {
                Image(uiImage: image).resizable().scaledToFit().padding()
            }
        }
    }

    // MARK: Forehead (LiDAR) mode

    private var foreheadControls: some View {
        VStack(spacing: 12) {
            Text(captureError ?? tracker.status)
                .font(.subheadline)
                .foregroundStyle(captureError == nil ? Color.secondary : Color.red)
                .multilineTextAlignment(.center)

            HStack(spacing: 16) {
                if let c = lastCapture {
                    Button { showingCapture = true } label: {
                        Image(uiImage: c.annotated)
                            .resizable().scaledToFill()
                            .frame(width: 72, height: 96)
                            .clipShape(RoundedRectangle(cornerRadius: 8))
                    }
                }
                VStack(alignment: .leading, spacing: 4) {
                    Text(lastCapture.map { String(format: "%.1f cm", $0.estimate.distance * 100) } ?? "— cm")
                        .font(.system(size: 44, weight: .semibold, design: .monospaced))
                        .foregroundStyle(lastCapture == nil ? .secondary : .primary)
                    if let e = lastCapture?.estimate {
                        Text(String(format: "depth %.1f cm · spread %.1f cm · n=%d",
                                    e.depth * 100, e.spread * 100, e.sampleCount))
                            .font(.system(.caption, design: .monospaced))
                            .foregroundStyle(.secondary)
                    }
                }
            }

            okButton(enabled: tracker.depthAvailable, action: captureForehead)
        }
    }

    private func captureForehead() {
        do {
            let capture = try tracker.captureForehead()
            let e = capture.estimate
            lastCapture = capture
            captureError = nil
            print(String(format: "CAPTURE forehead  distance %.1f cm  (depth %.1f cm, spread %.1f cm, n=%d)  saved %@",
                         e.distance * 100, e.depth * 100, e.spread * 100, e.sampleCount,
                         capture.folder.lastPathComponent))
        } catch {
            captureError = error.localizedDescription
            print("CAPTURE failed: \(error.localizedDescription)")
        }
        sweep.play()
    }

    // MARK: AprilTag mode

    @ViewBuilder
    private var markerControls: some View {
        if tracker.isCalibrating || tracker.headFrame == nil {
            calibrationControls
        } else {
            measurementControls
        }
    }

    private var calibrationControls: some View {
        VStack(spacing: 10) {
            Text(tracker.isCalibrating
                 ? "Head upright, eyes level. Sweep the phone slowly from the right ear, round the front, to the left ear, keeping 2–3 tags in view."
                 : "Calibrate the belt for this subject. Tag \(Belt.rightEarID) must sit above the right ear canal, tag \(Belt.leftEarID) above the left, tag \(Belt.frontID) at the front.")
                .font(.subheadline)
                .multilineTextAlignment(.center)

            tagChips

            Text(tracker.calibrationError ?? tracker.status)
                .font(.caption)
                .foregroundStyle(tracker.calibrationError == nil ? Color.secondary : Color.red)

            if tracker.isCalibrating {
                HStack {
                    Button("Cancel", role: .cancel, action: tracker.cancelCalibration)
                        .buttonStyle(.bordered)
                    Button("Restart", action: tracker.startCalibration)
                        .buttonStyle(.bordered)
                    Button("Finish", action: tracker.finishCalibration)
                        .buttonStyle(.borderedProminent)
                        .disabled(!tracker.mappedIDs.isSuperset(of: [Belt.leftEarID, Belt.rightEarID]))
                }
                .controlSize(.large)
            } else {
                Button("Start calibration", action: tracker.startCalibration)
                    .buttonStyle(.borderedProminent)
                    .controlSize(.large)
            }
        }
    }

    /// One chip per belt tag: grey = unseen, orange = seen, green = mapped.
    private var tagChips: some View {
        HStack(spacing: 4) {
            ForEach(Array(AprilTagDetector.validIDs), id: \.self) { id in
                let mapped = tracker.mappedIDs.contains(id)
                let seen = (tracker.mapCounts[id] ?? 0) > 0
                let ear = id == Belt.leftEarID ? "L" : id == Belt.rightEarID ? "R" : nil
                VStack(spacing: 1) {
                    Text("\(id)").font(.system(.caption, design: .monospaced).bold())
                    Text(ear ?? " ").font(.system(size: 9).bold())
                }
                .frame(width: 24, height: 30)
                .background(mapped ? Color.green : seen ? Color.orange : Color.gray.opacity(0.4),
                            in: RoundedRectangle(cornerRadius: 5))
                .overlay(RoundedRectangle(cornerRadius: 5)
                    .stroke(tracker.visibleIDs.contains(id) ? Color.primary : Color.clear, lineWidth: 2))
            }
        }
    }

    private var measurementControls: some View {
        VStack(spacing: 8) {
            targetTracker
            Divider()
            if let p = tracker.live {
                Text(String(format: "Az %+.1f°", p.azimuth))
                    .font(.system(size: 44, weight: .semibold, design: .monospaced))
                Text(String(format: "El %+.1f° · %.1f cm", p.elevation, p.distance * 100))
                    .font(.system(.body, design: .monospaced))
                Text(String(format: "drift az %.1f° el %.1f° d %.1f cm", p.azimuthDrift, p.elevationDrift,
                            p.distanceDrift * 100))
                    .font(.system(.caption, design: .monospaced))
                    .foregroundStyle(p.moved ? Color.orange : Color.secondary)
                Text(String(format: "noise az %.1f° el %.1f°%@", p.azimuthSpread, p.elevationSpread,
                            p.noisy ? "  (high: move phone closer)" : ""))
                    .font(.system(.caption, design: .monospaced))
                    .foregroundStyle(p.noisy ? Color.orange : Color.secondary)
            } else {
                Text("Az —").font(.system(size: 44, weight: .semibold, design: .monospaced)).foregroundStyle(.secondary)
                Text(tracker.status).font(.caption).foregroundStyle(.secondary)
            }

            if !CaptureSettings.speakerOffsetIsSet(channel: sweep.channel.rawValue) {
                Text("Speaker offset for \(sweep.channel.label) not set: using camera position")
                    .font(.caption).foregroundStyle(.orange)
            }

            if let r = tracker.lastResult { lastResultView(r) }

            okButton(enabled: tracker.live != nil && !tracker.isRecording) {
                tracker.captureMarker(with: sweep)
            }

            Button("Recalibrate belt", action: tracker.startCalibration)
                .font(.caption)
        }
    }

    // MARK: Position tracker

    /// Next target, which way to turn, and a checklist of all positions.
    private var targetTracker: some View {
        VStack(spacing: 6) {
            HStack {
                if let i = tracker.currentTarget {
                    Text(String(format: "Next: %@°", signed(tracker.targets[i].azimuth)))
                        .font(.title2.bold())
                } else {
                    Text("All positions done").font(.title3.bold()).foregroundStyle(.green)
                }
                Spacer()
                Menu {
                    Button(ARTracker.SequenceOrder.rightFirst.label) { tracker.restartSequence(.rightFirst) }
                    Button(ARTracker.SequenceOrder.leftFirst.label) { tracker.restartSequence(.leftFirst) }
                } label: {
                    Label("Restart", systemImage: "arrow.counterclockwise").font(.caption)
                }
            }

            turnCue

            ScrollViewReader { proxy in
                ScrollView(.horizontal, showsIndicators: false) {
                    HStack(spacing: 4) {
                        ForEach(tracker.targets) { t in targetChip(t).id(t.id) }
                    }
                }
                .onChange(of: tracker.currentTarget) { _, i in
                    if let i { withAnimation { proxy.scrollTo(i, anchor: .center) } }
                }
            }
        }
    }

    /// The participant's own left/right: turning right moves the phone towards their left ear (+az).
    @ViewBuilder
    private var turnCue: some View {
        if let i = tracker.currentTarget, let p = tracker.live {
            let delta = remainder(tracker.targets[i].azimuth - p.azimuth, 360)
            if abs(delta) <= CaptureSettings.targetToleranceDeg {
                Text(p.moved ? "In range: hold still" : "In range: press Position OK")
                    .font(.headline).foregroundStyle(.green)
            } else {
                Text(String(format: "Participant turns %@ %.1f°", delta > 0 ? "RIGHT" : "LEFT", abs(delta)))
                    .font(.headline).foregroundStyle(.orange)
            }
        } else if tracker.currentTarget != nil {
            Text("Waiting for tags").font(.headline).foregroundStyle(.secondary)
        }
    }

    /// Grey = to do, green = clean capture (orange if it landed outside the tolerance), outlined = next.
    private func targetChip(_ t: TargetPoint) -> some View {
        let isCurrent = tracker.currentTarget == t.id
        let offTarget = t.result.map { abs(remainder($0.azimuth - t.azimuth, 360)) > CaptureSettings.targetToleranceDeg } ?? false
        return Button {
            tracker.currentTarget = t.id
        } label: {
            VStack(spacing: 1) {
                Text(signed(t.azimuth)).font(.system(.caption, design: .monospaced).bold())
                Text(t.result.map { String(format: "%.1f", $0.azimuth) } ?? " ")
                    .font(.system(size: 9, design: .monospaced))
            }
            .frame(width: 38, height: 32)
            .foregroundStyle(.primary)
            .background(t.result == nil ? Color.gray.opacity(0.35) : offTarget ? Color.orange : Color.green,
                        in: RoundedRectangle(cornerRadius: 6))
            .overlay(RoundedRectangle(cornerRadius: 6).stroke(isCurrent ? Color.blue : .clear, lineWidth: 3))
        }
        .buttonStyle(.plain)
    }

    private func signed(_ azimuth: Double) -> String {
        azimuth == 0 ? "0" : String(format: "%+.0f", azimuth)
    }

    @ViewBuilder
    private func lastResultView(_ r: MarkerCaptureResult) -> some View {
        if let error = r.error {
            Text("Save failed: \(error)").font(.caption).foregroundStyle(.red)
        } else if let s = r.summary {
            Text(s.moved
                 ? String(format: "Moved during sweep (drift az %.1f°, el %.1f°, %.1f cm): redo", s.azimuthDrift,
                          s.elevationDrift, s.distanceDrift * 100)
                 : String(format: "Saved az %+.1f° el %+.1f° %.1f cm (n=%d)%@", s.azimuth, s.elevation,
                          s.distance * 100, s.sampleCount, s.noisy ? " · noisy" : ""))
                .font(.caption.bold())
                .foregroundStyle(s.moved ? Color.red : Color.green)
        } else {
            Text("No tags visible during the sweep: redo").font(.caption.bold()).foregroundStyle(.red)
        }
    }

    // MARK: Shared

    /// Pick which output channel carries the sweep, and play a 1 kHz tone on it so
    /// you can cover each speaker and confirm which one is sounding.
    private var speakerControls: some View {
        VStack(spacing: 8) {
            HStack {
                Picker("Speaker", selection: $sweep.channel) {
                    ForEach(SweepPlayer.Channel.allCases) { Text($0.label).tag($0) }
                }
                .pickerStyle(.segmented)
                Button(sweep.isTonePlaying ? "Stop tone" : "Test tone", action: sweep.toggleTone)
                    .buttonStyle(.bordered)
                    .disabled(sweep.isPlaying)
            }
            Text(String(format: "Sweep %.0f Hz–%.0f kHz · %.0f s · %.0f kHz",
                        SweepPlayer.startFrequency, SweepPlayer.endFrequency / 1000,
                        SweepPlayer.duration, SweepPlayer.sampleRate / 1000))
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(.secondary)
            if let warning = sweep.routeWarning {
                Text(warning).font(.caption).foregroundStyle(.red).multilineTextAlignment(.center)
            }
        }
    }

    private func okButton(enabled: Bool, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Text(sweep.isPlaying ? "Playing sweep… hold still" : "Position OK")
                .frame(maxWidth: .infinity)
        }
        .buttonStyle(.borderedProminent)
        .controlSize(.large)
        .disabled(!enabled || sweep.isPlaying)
    }
}

/// Live rear-camera feed for an existing ARSession.
struct CameraPreview: UIViewRepresentable {
    let session: ARSession

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView()
        view.session = session
        view.automaticallyUpdatesLighting = false
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}

#Preview { ContentView() }
