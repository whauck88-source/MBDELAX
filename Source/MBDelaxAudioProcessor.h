#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_dsp/juce_dsp.h>
#include <vector>

#include "dsp/MidSideMatrix.h"
#include "dsp/ThreeBandCrossover.h"
#include "dsp/EnvelopeFollower.h"
#include "dsp/TubeSaturator.h"
#include "dsp/Preset53.h"

/**
    MBDELAX - by Apolo Oliver Productions
    Multiband sidechain-surgery processor with a true external sidechain matrix.

    Signal flow per block (Preset 53 path):

        external SC --> [stereo peak] --> EnvelopeFollower (scalar, branchless)
                                                 |  per-sample linear gain[]
        main in --> [1.5ms lookahead delay] --> LR4 crossover (45 / 95 Hz)
                                                 |- sub  (<45)   > clean
                                                 |- tgt  (45-95) > SIMD gain[] >
                                                 |                  dynamic 65Hz notch >
                                                 |                  M/S width collapse
                                                 |- high (>95)   > tube saturation
                                                 v
                                          sub + tgt + high > dry/wet mix > output
*/
class MBDelaxAudioProcessor final : public juce::AudioProcessor
{
public:
    MBDelaxAudioProcessor();
    ~MBDelaxAudioProcessor() override = default;

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override
    {
        return new juce::GenericAudioProcessorEditor (*this);
    }
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return "MBDELAX"; }
    bool acceptsMidi() const override  { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return mbdelax::presets::Preset53::lookaheadMs * 0.001; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return "Preset 53 - Kick/Bass Surgery"; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock&) override;
    void setStateInformation (const void*, int) override;

    juce::AudioProcessorValueTreeState apvts;

private:
    //==============================================================================
    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

    void updateHostSyncedRelease();
    void applyPreset53Parameters();

    //==============================================================================
    double currentSampleRate = 44100.0;
    float  lookaheadSamples  = 0.0f;   // fractional: 1.5 ms is rarely an integer
    double lastKnownBpm      = 120.0;

    // --- DSP modules ----------------------------------------------------------
    mbdelax::dsp::ThreeBandCrossover crossover;
    mbdelax::dsp::EnvelopeFollower   envFollower;
    mbdelax::dsp::TubeSaturator      tubeSat;

    // FIR fractional delay (Lagrange) for the 1.5 ms main-path lookahead.
    juce::dsp::DelayLine<float, juce::dsp::DelayLineInterpolationTypes::Lagrange3rd> lookaheadDelay { 256 };
    juce::dsp::IIR::Filter<float> notch65[2];  // per-channel band-pass extractor @ 65 Hz

    juce::dsp::Gain<float> outputGain;

    // --- Work buffers (allocated in prepareToPlay) ----------------------------
    juce::AudioBuffer<float> dryBuffer;        // delayed dry main for the mix
    juce::AudioBuffer<float> detectionBuffer;  // mono stereo-peak detector signal
    juce::AudioBuffer<float> subBuffer;        // < 45 Hz
    juce::AudioBuffer<float> targetBuffer;     // 45-95 Hz
    juce::AudioBuffer<float> highBuffer;       // > 95 Hz

    std::vector<float> gainRedux;              // per-sample linear gain (detector output)
    std::vector<float> duckAmt;                // per-sample 1 - gain (0..1, drives the dynamic notch)
    std::vector<float> collapseAmt;            // per-sample 0..1, reaches 1 at >3 dB GR (width->mono)

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MBDelaxAudioProcessor)
};