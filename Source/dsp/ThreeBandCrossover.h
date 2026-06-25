#pragma once

#include <juce_dsp/juce_dsp.h>

namespace mbdelax::dsp
{
/**
    Phase-coherent 3-band crossover built from Linkwitz-Riley (LR4) sections.

        low  : < fLow            (e.g. < 45 Hz  -> clean sub, never ducked)
        mid  : fLow .. fHigh     (e.g. 45..95 Hz -> the surgery band, 24 dB/oct)
        high : > fHigh           (e.g. > 95 Hz  -> tube stage, never ducked)

    JUCE's LinkwitzRileyFilter is 4th-order => 24 dB/oct skirts, exactly the
    "Linkwitz-Riley 24dB/oct" band-pass the preset asks for around 45-95 Hz.

    LR4 sections are magnitude-complementary; summing low+mid+high reconstructs
    the input flat. The catch is phase: after we re-split the upper signal at
    fHigh, the low band has to be run through an all-pass tuned at fHigh so it
    stays time/phase-aligned with the mid+high recombination. That is the
    classic Linkwitz-Riley tree and it is what makes "frequencies above 95Hz
    remain completely bypassed by the ducking algorithm" actually true instead
    of just approximately true.
*/
class ThreeBandCrossover
{
public:
    void prepare (const juce::dsp::ProcessSpec& spec)
    {
        for (auto* f : { &lp1, &hp1, &lp2, &hp2, &ap2 })
            f->prepare (spec);

        lp1.setType (juce::dsp::LinkwitzRileyFilterType::lowpass);
        hp1.setType (juce::dsp::LinkwitzRileyFilterType::highpass);
        lp2.setType (juce::dsp::LinkwitzRileyFilterType::lowpass);
        hp2.setType (juce::dsp::LinkwitzRileyFilterType::highpass);
        ap2.setType (juce::dsp::LinkwitzRileyFilterType::allpass);
    }

    void setCrossoverFrequencies (float fLow, float fHigh)
    {
        lp1.setCutoffFrequency (fLow);
        hp1.setCutoffFrequency (fLow);

        ap2.setCutoffFrequency (fHigh);
        lp2.setCutoffFrequency (fHigh);
        hp2.setCutoffFrequency (fHigh);
    }

    void reset()
    {
        lp1.reset(); hp1.reset();
        lp2.reset(); hp2.reset();
        ap2.reset();
    }

    /**
        Splits 'input' into three pre-allocated, same-size band blocks.
        Pass three distinct buffers; 'high' is used as scratch for the >fLow
        signal before it is re-split.
    */
    void process (const juce::dsp::AudioBlock<const float>& input,
                  juce::dsp::AudioBlock<float>&             low,
                  juce::dsp::AudioBlock<float>&             mid,
                  juce::dsp::AudioBlock<float>&             high)
    {
        low.copyFrom (input);
        high.copyFrom (input);

        { juce::dsp::ProcessContextReplacing<float> c (low);  lp1.process (c); } // low  = < fLow
        { juce::dsp::ProcessContextReplacing<float> c (high); hp1.process (c); } // high = > fLow

        mid.copyFrom (high);                                                     // mid carries > fLow

        { juce::dsp::ProcessContextReplacing<float> c (mid);  lp2.process (c); } // mid  = fLow..fHigh
        { juce::dsp::ProcessContextReplacing<float> c (high); hp2.process (c); } // high = > fHigh

        { juce::dsp::ProcessContextReplacing<float> c (low);  ap2.process (c); } // phase-align low
    }

private:
    juce::dsp::LinkwitzRileyFilter<float> lp1, hp1, lp2, hp2, ap2;
};
} // namespace mbdelax::dsp
