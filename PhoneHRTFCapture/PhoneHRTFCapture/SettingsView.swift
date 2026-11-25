//
//  SettingsView.swift
//  PhoneHRTFCapture
//
//  Measured geometry: tag size, belt height, speaker offsets, movement limits.
//  Keys are shared with CaptureSettings.
//

import SwiftUI

struct SettingsView: View {
    typealias Key = CaptureSettings.Key
    typealias Default = CaptureSettings.Default

    @Environment(\.dismiss) private var dismiss

    @AppStorage(Key.tagSizeMM) private var tagSizeMM = Default.tagSizeMM
    @AppStorage(Key.beltHeightMM) private var beltHeightMM = Default.beltHeightMM
    @AppStorage(Key.moveThresholdDeg) private var moveThresholdDeg = Default.moveThresholdDeg
    @AppStorage(Key.moveThresholdCM) private var moveThresholdCM = Default.moveThresholdCM
    @AppStorage(Key.targetToleranceDeg) private var targetToleranceDeg = Default.targetToleranceDeg

    @AppStorage(Key.speakerRightMM(0)) private var right0 = 0.0
    @AppStorage(Key.speakerDownMM(0)) private var down0 = 0.0
    @AppStorage(Key.speakerFrontMM(0)) private var front0 = 0.0
    @AppStorage(Key.speakerRightMM(1)) private var right1 = 0.0
    @AppStorage(Key.speakerDownMM(1)) private var down1 = 0.0
    @AppStorage(Key.speakerFrontMM(1)) private var front1 = 0.0

    var body: some View {
        NavigationStack {
            Form {
                Section {
                    field("Tag size", $tagSizeMM, "mm")
                } header: { Text("AprilTags") } footer: {
                    Text("Outer edge of the black square. The printed sheet is 32 mm. Check with a ruler.")
                }

                Section {
                    field("Belt above ear canal", $beltHeightMM, "mm")
                } header: { Text("Head") } footer: {
                    Text("Vertical distance from the ear canal (tragus) up to the centre of tags \(Belt.rightEarID) and \(Belt.leftEarID). "
                         + "Moves the head centre down to ear-canal height. Recalibrate after changing it.")
                }

                Section {
                    field("Target tolerance", $targetToleranceDeg, "°")
                } header: { Text("Position tracker") } footer: {
                    Text("The tracker turns green when the measured azimuth is within this of the next target.")
                }

                speakerSection(channel: 0, right: $right0, down: $down0, front: $front0)
                speakerSection(channel: 1, right: $right1, down: $down1, front: $front1)

                Section {
                    field("Max angle change", $moveThresholdDeg, "°")
                    field("Max distance change", $moveThresholdCM, "cm")
                } header: { Text("Movement during sweep") } footer: {
                    Text("A capture is flagged as moved if the median azimuth or elevation of the last third "
                         + "of the sweep differs from the first third by more than this, or distance does. "
                         + "Frame-to-frame tag noise is shown separately and does not reject a capture.")
                }
            }
            .navigationTitle("Geometry")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar { Button("Done") { dismiss() } }
        }
    }

    private func speakerSection(channel: Int, right: Binding<Double>, down: Binding<Double>,
                                front: Binding<Double>) -> some View {
        Section {
            field("Right of lens", right, "mm")
            field("Below lens", down, "mm")
            field("Toward screen", front, "mm")
        } header: {
            Text("Speaker on \(SweepPlayer.Channel(rawValue: channel)!.label)")
        } footer: {
            Text("Phone upright, screen facing you. Measure from the centre of the rear main camera lens "
                 + "to the centre of the speaker grille that the test tone plays from. Use negative values for left, "
                 + "above or toward the back.")
        }
    }

    private func field(_ label: String, _ value: Binding<Double>, _ unit: String) -> some View {
        LabeledContent(label) {
            HStack(spacing: 4) {
                TextField(label, value: value, format: .number.precision(.fractionLength(0...1)))
                    .keyboardType(.numbersAndPunctuation)
                    .multilineTextAlignment(.trailing)
                    .frame(maxWidth: 90)
                Text(unit).foregroundStyle(.secondary)
            }
        }
    }
}
