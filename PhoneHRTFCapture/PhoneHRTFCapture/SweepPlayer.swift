//
//  SweepPlayer.swift
//  PhoneHRTFCapture
//
//  Plays an exponential (logarithmic) sine sweep (Farina, 2000) through ONE of
//  the phone's two built-in speakers.
//
//  Since iPhone 7, "stereo" playback uses the bottom speaker AND the earpiece
//  (~14 cm apart). A sweep on both would be two sources at two positions, and the
//  measured HRIR would be the sum of two directions. So the sweep is written into
//  a single output channel and the other channel is digital silence.
//
//  Which physical speaker a channel maps to depends on device orientation, so the
//  app is locked to portrait. Use the test tone + cover each speaker to confirm
//  which one is active, and measure phone-to-head distance from THAT speaker.
//

import AVFoundation
import Combine

final class SweepPlayer: ObservableObject {
    // Sweep parameters. Kept here so deconvolution can regenerate the same signal later.
    static let sampleRate = 48_000.0       // Hz; generated and played at exactly this rate
    static let startFrequency = 100.0      // Hz; phone speakers produce little below this
    static let endFrequency = 20_000.0     // Hz
    static let duration = 4.0              // s
    static let fadeDuration = 0.010        // s, raised-cosine fade at each end
    static let amplitude = 0.5             // linear, full scale = 1
    static let testToneFrequency = 1_000.0 // Hz

    enum Channel: Int, CaseIterable, Identifiable, Codable {
        case left = 0, right = 1
        var id: Int { rawValue }
        var label: String { self == .left ? "Ch 1 (L)" : "Ch 2 (R)" }
    }

    static let channelDefaultsKey = "sweepChannel"
    /// Read by CaptureStore so each capture records which speaker played.
    static var storedChannel: Channel {
        Channel(rawValue: UserDefaults.standard.integer(forKey: channelDefaultsKey)) ?? .left
    }
    /// Hardware rate seen at the last engine start (0 until the first play).
    private(set) static var lastHardwareSampleRate = 0.0
    private(set) static var lastOutputChannelCount = 0

    @Published private(set) var isPlaying = false
    @Published private(set) var isTonePlaying = false
    @Published private(set) var routeWarning: String?
    @Published var channel: Channel = SweepPlayer.storedChannel {
        didSet {
            UserDefaults.standard.set(channel.rawValue, forKey: Self.channelDefaultsKey)
            if isTonePlaying { stopTone(); startTone() }
        }
    }

    private let engine = AVAudioEngine()
    private let player = AVAudioPlayerNode()
    private let format = AVAudioFormat(standardFormatWithSampleRate: SweepPlayer.sampleRate, channels: 2)!
    private var configured = false

    // MARK: Public

    /// Plays the sweep once. `completion` runs on the main actor after the last
    /// sample has been played back (not just scheduled).
    func play(completion: (() -> Void)? = nil) {
        guard !isPlaying else { return }
        if isTonePlaying { stopTone() }
        do {
            try prepare()
            guard let buffer = Self.makeBuffer(format: format, channel: channel, samples: Self.sweepSamples) else {
                throw Self.error("Could not create sweep buffer")
            }
            isPlaying = true
            player.play()
            Task {
                await player.scheduleBuffer(buffer, at: nil, options: [],
                                            completionCallbackType: .dataPlayedBack)
                isPlaying = false
                completion?()
            }
        } catch {
            print("Sweep playback failed: \(error)")
            routeWarning = error.localizedDescription
            isPlaying = false
        }
    }

    func toggleTone() {
        isTonePlaying ? stopTone() : startTone()
    }

    // MARK: Tone (speaker identification)

    private func startTone() {
        guard !isPlaying else { return }
        do {
            try prepare()
            // Exactly 1 s of 1 kHz = whole number of cycles, so looping is click-free.
            let n = Int(Self.sampleRate)
            let tone = (0..<n).map { Float(0.3 * sin(2 * .pi * Self.testToneFrequency * Double($0) / Self.sampleRate)) }
            guard let buffer = Self.makeBuffer(format: format, channel: channel, samples: tone) else { return }
            player.scheduleBuffer(buffer, at: nil, options: [.loops, .interrupts])
            player.play()
            isTonePlaying = true
        } catch {
            routeWarning = error.localizedDescription
        }
    }

    private func stopTone() {
        player.stop()
        isTonePlaying = false
    }

    // MARK: Engine

    private func prepare() throws {
        if configured {
            if !engine.isRunning { try engine.start() }
            return
        }

        let session = AVAudioSession.sharedInstance()
        // .measurement turns off system voice processing / dynamics where possible.
        try session.setCategory(.playback, mode: .measurement)
        try session.setPreferredSampleRate(Self.sampleRate)
        try session.setActive(true)

        engine.attach(player)
        // Player (48 kHz stereo) -> mixer -> output. The mixer passes L->L and R->R,
        // so the silent channel stays silent.
        engine.connect(player, to: engine.mainMixerNode, format: format)
        player.pan = 0
        engine.mainMixerNode.outputVolume = 1
        try engine.start()
        configured = true

        let hw = engine.outputNode.outputFormat(forBus: 0)
        Self.lastHardwareSampleRate = hw.sampleRate
        Self.lastOutputChannelCount = Int(hw.channelCount)
        var problems: [String] = []
        if hw.sampleRate != Self.sampleRate {
            problems.append(String(format: "Output is %.0f Hz, not 48000 Hz (will be resampled)", hw.sampleRate))
        }
        if hw.channelCount < 2 {
            problems.append("Output is mono: cannot isolate one speaker")
        }
        if session.currentRoute.outputs.contains(where: { $0.portType != .builtInSpeaker }) {
            problems.append("Output is not the built-in speaker (disconnect headphones/Bluetooth)")
        }
        routeWarning = problems.isEmpty ? nil : problems.joined(separator: "\n")
        print(String(format: "Audio out: %.0f Hz, %d ch, route %@", hw.sampleRate, hw.channelCount,
                     session.currentRoute.outputs.map(\.portName).joined(separator: ",")))
    }

    // MARK: Signal generation

    /// Interleaves `samples` into `channel` of a stereo buffer; the other channel is zeros.
    private static func makeBuffer(format: AVAudioFormat, channel: Channel, samples: [Float]) -> AVAudioPCMBuffer? {
        guard let buffer = AVAudioPCMBuffer(pcmFormat: format, frameCapacity: AVAudioFrameCount(samples.count)),
              let data = buffer.floatChannelData else { return nil }
        buffer.frameLength = AVAudioFrameCount(samples.count)
        for c in 0..<Int(format.channelCount) {
            let dst = data[c]
            if c == channel.rawValue {
                samples.withUnsafeBufferPointer { dst.update(from: $0.baseAddress!, count: samples.count) }
            } else {
                dst.update(repeating: 0, count: samples.count)
            }
        }
        return buffer
    }

    /// x(t) = sin(2π f1 L (e^(t/L) − 1)),  L = T / ln(f2/f1)
    static let sweepSamples: [Float] = {
        let sr = sampleRate
        let f1 = startFrequency
        let f2 = endFrequency
        let count = Int(duration * sr)
        let L = duration / log(f2 / f1)
        let fadeCount = max(1, Int(fadeDuration * sr))
        return (0..<count).map { i in
            let t = Double(i) / sr
            var s = sin(2 * .pi * f1 * L * (exp(t / L) - 1))
            if i < fadeCount {
                s *= 0.5 * (1 - cos(.pi * Double(i) / Double(fadeCount)))
            } else if i >= count - fadeCount {
                s *= 0.5 * (1 - cos(.pi * Double(count - 1 - i) / Double(fadeCount)))
            }
            return Float(amplitude * s)
        }
    }()

    private static func error(_ message: String) -> NSError {
        NSError(domain: "SweepPlayer", code: 1, userInfo: [NSLocalizedDescriptionKey: message])
    }
}
