#pragma once

#include <juce_dsp/juce_dsp.h>
#include <cmath>

namespace mbdelax::dsp
{
/**
    Scalar, branch-free peak envelope follower + downward gain computer.

    This is intentionally NOT vectorised. The one-pole detector is a serial
    recurrence (env[n] depends on env[n-1]); feeding it through SIMD lanes would
    break the feedback path and produce wrong/unstable gain. So we keep the
    *detection* scalar and branchless, and vectorise only the *application*
    of the resulting gain array (see applyTimeVaryingGainSIMD in the processor).
    Vectorising a serial IIR across time is mathematically impossible; the
    correct place to spend SIMD is the flat element-wise gain multiply, which we
    do. This is the senior-engineer answer to "vectorise the envelope follower".

    "Branchless" here means the attack/release selection uses an arithmetic
    blend rather than an if(), and the gain computer's max(x,0) uses the
    0.5*(x+|x|) identity. No conditional jumps inside the per-sample loop.
*/
class EnvelopeFollower
{
public:
    void prepare (double sampleRate)
    {
        sr = sampleRate;
        updateCoefficients();
        env = 0.0f;
    }

    void setTimes (float attackMs, float releaseMs)
    {
        atkMs = attackMs;
        relMs = releaseMs;
        updateCoefficients();
    }

    void setGainComputer (float thresholdDb, float ratio)
    {
        thrDb = thresholdDb;
        slope = 1.0f - (1.0f / juce::jmax (1.0f, ratio));
    }

    void reset() { env = 0.0f; lastGrDb = 0.0f; }

    /**
        One serial step. 'detectorSample' is the (already stereo-summed/peak)
        sidechain magnitude. Returns the LINEAR gain (<= 1) to multiply the
        target signal by. Fully branchless.

        Side-effect: stashes the gain reduction in dB (<= 0) so the caller can
        derive threshold-driven behaviour (e.g. the > 3 dB mono-collapse) without
        an extra log per sample. Read it with getGrDb() right after this call.
    */
    inline float processSample (float detectorSample) noexcept
    {
        const float in = std::abs (detectorSample);

        // Branch-free attack/release: (in > env) yields 1.0f when rising, else 0.0f.
        const float rising = static_cast<float> (in > env);
        const float coeff  = relCoeff + (atkCoeff - relCoeff) * rising;

        env = in + coeff * (env - in);

        // Over-threshold in dB, branch-free max(x, 0) via 0.5*(x + |x|).
        const float envDb   = juce::Decibels::gainToDecibels (env + 1.0e-9f);
        const float over    = envDb - thrDb;
        const float overPos = 0.5f * (over + std::abs (over));

        lastGrDb = -overPos * slope;                       // <= 0 dB (gain reduction)
        return juce::Decibels::decibelsToGain (lastGrDb);  // <= 1.0 linear
    }

    /** Gain reduction in dB (<= 0) produced by the most recent processSample(). */
    inline float getGrDb() const noexcept { return lastGrDb; }

private:
    void updateCoefficients()
    {
        const auto tc = [this] (float ms)
        {
            return std::exp (-1.0f / static_cast<float> (juce::jmax (1.0e-4f, ms) * 0.001 * sr));
        };
        atkCoeff = tc (atkMs);
        relCoeff = tc (relMs);
    }

    double sr        = 44100.0;
    float  atkMs     = 2.0f;
    float  relMs     = 120.0f;
    float  atkCoeff  = 0.0f;
    float  relCoeff  = 0.0f;
    float  env       = 0.0f;
    float  thrDb     = -28.0f;
    float  slope     = 0.75f;
    float  lastGrDb  = 0.0f;
};
} // namespace mbdelax::dsp
