#pragma once

#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>
#include <cmath>

namespace mbdelax::dsp
{
/**
    Asymmetric "tube" stage for the high band (> 95 Hz).

    Three parts:
      1) Input drive (click-free smoothed gain).
      2) An asymmetric soft-clip (tanh + DC bias). The bias makes the positive
         and negative halves clip differently, which is what generates the
         2nd-order (even) harmonics a triode is known for. We subtract the bias'
         DC offset so the stage stays centred. This is always on - it *is* the
         tube tone.
      3) A +3 dB high-shelf at 150 Hz that is engaged ONLY while the external
         sidechain is active ("bias shift pra 150Hz +3dB quando SC ativo"). To
         honour the zero-heap-alloc-in-processBlock rule we never recompute IIR
         coefficients on the audio thread: the shelf is built once in prepare(),
         and we crossfade between the dry (pre-shelf) and shelved signal with a
         smoothed 0..1 scalar. That makes the engage/disengage click-free and
         keeps a literal +0 dB -> +3 dB transition with a single filter.
*/
class TubeSaturator
{
public:
    void prepare (const juce::dsp::ProcessSpec& spec)
    {
        sr = spec.sampleRate;

        drive .prepare (spec);
        makeup.prepare (spec);
        shelf .prepare (spec);

        drive .setRampDurationSeconds (0.02);
        makeup.setRampDurationSeconds (0.02);
        drive .setGainLinear (1.0f);
        makeup.setGainLinear (1.0f);

        *shelf.state = *juce::dsp::IIR::Coefficients<float>::makeHighShelf (
            sr, 150.0f, 0.707f, juce::Decibels::decibelsToGain (3.0f));
        shelf.reset();

        scActive.reset (sr, 0.03);   // 30 ms engage/disengage glide
        scActive.setCurrentAndTargetValue (0.0f);

        // Scratch for the pre-shelf signal. Allocated here, reused in process()
        // with avoidReallocating == true so the audio thread never allocates.
        preShelf.setSize ((int) spec.numChannels,
                          (int) spec.maximumBlockSize, false, false, true);
    }

    /** 'amount' 0..1 maps to a musically useful 1x..4x input drive. */
    void setDrive (float amount)
    {
        const float linear = juce::jmap (juce::jlimit (0.0f, 1.0f, amount), 1.0f, 4.0f);
        drive.setGainLinear (linear);
        makeup.setGainLinear (1.0f / std::sqrt (linear));   // rough loudness compensation
    }

    void reset()
    {
        shelf.reset();
        scActive.setCurrentAndTargetValue (scActive.getTargetValue());
    }

    /**
        @param block            high band, processed in place.
        @param sidechainActive  true when the external SC matrix is live; engages
                                the +3 dB / 150 Hz shelf (smoothed, branchless).
    */
    void process (juce::dsp::AudioBlock<float>& block, bool sidechainActive)
    {
        const auto numCh   = block.getNumChannels();
        const auto numSamp = block.getNumSamples();

        scActive.setTargetValue (sidechainActive ? 1.0f : 0.0f);

        // 1) drive
        { juce::dsp::ProcessContextReplacing<float> c (block); drive.process (c); }

        // 2) asymmetric tube shaper (even harmonics) - explicit, no std::function
        //    indirection, no branches in the inner loop.
        constexpr float bias   = 0.18f;
        constexpr float k      = 1.6f;
        const float     offset = std::tanh (bias * k);
        for (size_t ch = 0; ch < numCh; ++ch)
        {
            auto* d = block.getChannelPointer (ch);
            for (size_t n = 0; n < numSamp; ++n)
                d[n] = std::tanh ((d[n] + bias) * k) - offset;
        }

        // 3) stash pre-shelf, apply the static +3 dB shelf, then crossfade by the
        //    smoothed sidechain-active scalar (advance once per frame, shared
        //    across channels so L/R stay phase-locked). Zero allocation.
        preShelf.setSize ((int) numCh, (int) numSamp, false, false, true);
        for (size_t ch = 0; ch < numCh; ++ch)
            juce::FloatVectorOperations::copy (preShelf.getWritePointer ((int) ch),
                                               block.getChannelPointer (ch),
                                               (int) numSamp);

        { juce::dsp::ProcessContextReplacing<float> c (block); shelf.process (c); }

        for (size_t n = 0; n < numSamp; ++n)
        {
            const float a  = scActive.getNextValue();   // 0 = dry, 1 = +3 dB shelved
            const float ia = 1.0f - a;
            for (size_t ch = 0; ch < numCh; ++ch)
            {
                auto* d = block.getChannelPointer (ch);
                d[n] = ia * preShelf.getReadPointer ((int) ch)[n] + a * d[n];
            }
        }

        // 4) makeup
        { juce::dsp::ProcessContextReplacing<float> c (block); makeup.process (c); }
    }

private:
    double sr = 44100.0;

    juce::dsp::Gain<float> drive, makeup;
    juce::dsp::ProcessorDuplicator<juce::dsp::IIR::Filter<float>,
                                   juce::dsp::IIR::Coefficients<float>> shelf;

    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> scActive;
    juce::AudioBuffer<float> preShelf;
};
} // namespace mbdelax::dsp
