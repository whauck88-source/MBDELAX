#include "MBDelaxAudioProcessor.h"

using namespace mbdelax;
using P53 = presets::Preset53;

namespace
{
    //==========================================================================
    // SIMD bulk application of a time-varying gain array onto one channel.
    //
    // The recursive detector stays scalar (see EnvelopeFollower); here we only
    // *apply* the already-computed gain[], which is a flat element-wise multiply
    // and therefore safe and profitable to vectorise with juce::dsp::SIMDRegister.
    //
    // We co-align both pointers before entering the vector loop. The band buffers
    // are dedicated juce::AudioBuffers processed from index 0, so in practice both
    // are aligned and the scalar prologue length is zero; if a host ever hands us
    // an unaligned gain vector we transparently fall back to scalar - still correct,
    // never a crash, never a denormal-spewing branch inside the hot loop.
    //==========================================================================
    inline void applyTimeVaryingGainSIMD (float* data, const float* gain, int n) noexcept
    {
        using SIMD = juce::dsp::SIMDRegister<float>;
        constexpr int N = static_cast<int> (SIMD::SIMDNumElements);

        auto aligned = [] (const void* p) noexcept
        {
            return (reinterpret_cast<std::uintptr_t> (p) % (N * sizeof (float))) == 0;
        };

        int i = 0;
        if (aligned (data) && aligned (gain))
        {
            const int vEnd = n - (n % N);
            for (; i < vEnd; i += N)
            {
                auto d = SIMD::fromRawArray (data + i);
                auto g = SIMD::fromRawArray (gain + i);
                (d * g).copyToRawArray (data + i);
            }
        }

        for (; i < n; ++i)            // scalar tail (or full fallback)
            data[i] *= gain[i];
    }
}

//==============================================================================
//  Multi-bus topology: 2-ch Main In, 2-ch Main Out, 2-ch External Sidechain In.
//==============================================================================
MBDelaxAudioProcessor::MBDelaxAudioProcessor()
    : juce::AudioProcessor (
          BusesProperties()
              .withInput  ("Input",      juce::AudioChannelSet::stereo(), true)
              .withOutput ("Output",     juce::AudioChannelSet::stereo(), true)
              .withInput  ("Sidechain",  juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "PARAMS", createParameterLayout())
{
}

juce::AudioProcessorValueTreeState::ParameterLayout
MBDelaxAudioProcessor::createParameterLayout()
{
    using namespace juce;
    AudioProcessorValueTreeState::ParameterLayout layout;

    layout.add (std::make_unique<AudioParameterBool>  (ParameterID { "extSc", 1 },
                    "External SC", true));
    layout.add (std::make_unique<AudioParameterFloat> (ParameterID { "mix", 1 },
                    "Mix", NormalisableRange<float> (0.0f, 1.0f, 0.001f), 1.0f));
    layout.add (std::make_unique<AudioParameterFloat> (ParameterID { "threshold", 1 },
                    "Threshold", NormalisableRange<float> (-60.0f, 0.0f, 0.1f), P53::thresholdDb));
    layout.add (std::make_unique<AudioParameterFloat> (ParameterID { "ratio", 1 },
                    "Ratio", NormalisableRange<float> (1.0f, 20.0f, 0.1f), P53::ratio));
    layout.add (std::make_unique<AudioParameterFloat> (ParameterID { "release", 1 },
                    "Release Div", NormalisableRange<float> (0.125f, 2.0f, 0.001f), P53::releaseDivision));
    layout.add (std::make_unique<AudioParameterFloat> (ParameterID { "sideCollapse", 1 },
                    "Mono Below", NormalisableRange<float> (0.0f, 1.0f, 0.001f), P53::sideCollapse));
    layout.add (std::make_unique<AudioParameterFloat> (ParameterID { "notchDepth", 1 },
                    "Notch Depth", NormalisableRange<float> (0.0f, 1.0f, 0.001f), P53::dynamicNotchDepth));
    layout.add (std::make_unique<AudioParameterFloat> (ParameterID { "drive", 1 },
                    "Saturn Drive", NormalisableRange<float> (0.0f, 1.0f, 0.001f), P53::tubeDrive));
    layout.add (std::make_unique<AudioParameterFloat> (ParameterID { "output", 1 },
                    "Output", NormalisableRange<float> (-24.0f, 24.0f, 0.1f), 0.0f));

    return layout;
}

//==============================================================================
bool MBDelaxAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    if (layouts.getMainInputChannelSet()  != juce::AudioChannelSet::stereo()) return false;
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo()) return false;

    // Sidechain may be disabled (host not routing), mono, or stereo.
    const auto sc = layouts.getChannelSet (true, 1);
    return sc == juce::AudioChannelSet::disabled()
        || sc == juce::AudioChannelSet::mono()
        || sc == juce::AudioChannelSet::stereo();
}

//==============================================================================
void MBDelaxAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    currentSampleRate = sampleRate;
    lookaheadSamples  = static_cast<float> (P53::lookaheadMs * 0.001 * sampleRate);

    juce::dsp::ProcessSpec stereoSpec { sampleRate,
                                        static_cast<juce::uint32> (samplesPerBlock),
                                        2 };
    juce::dsp::ProcessSpec monoSpec   { sampleRate,
                                        static_cast<juce::uint32> (samplesPerBlock),
                                        1 };

    crossover.prepare (stereoSpec);
    crossover.setCrossoverFrequencies (P53::crossoverLowHz, P53::crossoverHighHz);
    crossover.reset();

    envFollower.prepare (sampleRate);
    envFollower.setTimes (P53::attackMs, 120.0f);
    envFollower.setGainComputer (P53::thresholdDb, P53::ratio);

    tubeSat.prepare (stereoSpec);
    tubeSat.setDrive (P53::tubeDrive);

    lookaheadDelay.prepare (stereoSpec);
    lookaheadDelay.setMaximumDelayInSamples (juce::jmax (8, juce::roundToInt (lookaheadSamples) + 4));
    lookaheadDelay.setDelay (lookaheadSamples);
    lookaheadDelay.reset();

    for (auto& f : notch65)
    {
        f.prepare (monoSpec);
        f.coefficients = juce::dsp::IIR::Coefficients<float>::makeBandPass (
            sampleRate, P53::dynamicNotchHz, P53::dynamicNotchQ);
        f.reset();
    }

    outputGain.prepare (stereoSpec);
    outputGain.setRampDurationSeconds (0.02);

    dryBuffer      .setSize (2, samplesPerBlock);
    detectionBuffer.setSize (1, samplesPerBlock);
    subBuffer      .setSize (2, samplesPerBlock);
    targetBuffer   .setSize (2, samplesPerBlock);
    highBuffer     .setSize (2, samplesPerBlock);

    gainRedux  .assign (static_cast<size_t> (samplesPerBlock), 1.0f);
    duckAmt    .assign (static_cast<size_t> (samplesPerBlock), 0.0f);
    collapseAmt.assign (static_cast<size_t> (samplesPerBlock), 0.0f);
}

//==============================================================================
void MBDelaxAudioProcessor::updateHostSyncedRelease()
{
    double bpm = lastKnownBpm;

    if (auto* ph = getPlayHead())
        if (auto pos = ph->getPosition())          // JUCE 8 Optional<PositionInfo>
            if (auto b = pos->getBpm())
                bpm = *b;

    lastKnownBpm = juce::jlimit (20.0, 300.0, bpm);

    const float div         = apvts.getRawParameterValue ("release")->load();
    const double quarterMs  = 60000.0 / lastKnownBpm;
    const double releaseMs  = quarterMs * static_cast<double> (div);

    envFollower.setTimes (P53::attackMs, static_cast<float> (releaseMs));
}

//==============================================================================
void MBDelaxAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer,
                                          juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    const int numSamples = buffer.getNumSamples();

    // ---- 1. Resolve buses ---------------------------------------------------
    auto mainBus = getBusBuffer (buffer, true, 0);   // main I/O (in place)
    auto scBus   = getBusBuffer (buffer, true, 1);   // external sidechain

    auto* scBusObj = getBus (true, 1);
    const bool extScWanted    = apvts.getRawParameterValue ("extSc")->load() > 0.5f;
    const bool extScAvailable = scBusObj != nullptr
                                && scBusObj->isEnabled()
                                && scBus.getNumChannels() > 0;
    const bool useExternal    = extScWanted && extScAvailable;   // hoisted out of all loops

    // ---- 2. Host-synced auto-release ---------------------------------------
    updateHostSyncedRelease();

    // ---- 3. Build the detector signal (stereo peak), branch hoisted --------
    detectionBuffer.setSize (1, numSamples, false, false, true);
    auto* det = detectionBuffer.getWritePointer (0);

    const float* dL; const float* dR;
    if (useExternal)
    {
        dL = scBus.getReadPointer (0);
        dR = scBus.getReadPointer (scBus.getNumChannels() > 1 ? 1 : 0);
    }
    else
    {
        dL = mainBus.getReadPointer (0);
        dR = mainBus.getReadPointer (mainBus.getNumChannels() > 1 ? 1 : 0);
    }
    for (int n = 0; n < numSamples; ++n)
        det[n] = juce::jmax (std::abs (dL[n]), std::abs (dR[n]));

    // ---- 4. 1.5 ms lookahead delay on the MAIN path ------------------------
    // The detector runs on the UN-delayed sidechain; delaying the main makes the
    // ducking anticipate the kick by exactly the lookahead amount.
    {
        const int chs = mainBus.getNumChannels();
        for (int n = 0; n < numSamples; ++n)
            for (int ch = 0; ch < chs; ++ch)
            {
                lookaheadDelay.pushSample (ch, mainBus.getSample (ch, n));
                mainBus.setSample (ch, n, lookaheadDelay.popSample (ch));
            }
    }

    // ---- 5. Stash delayed dry for the mix ----------------------------------
    dryBuffer.setSize (2, numSamples, false, false, true);
    for (int ch = 0; ch < 2; ++ch)
        dryBuffer.copyFrom (ch, 0, mainBus, ch, 0, numSamples);

    // ---- 6. Crossover split at 45 / 95 Hz ----------------------------------
    subBuffer   .setSize (2, numSamples, false, false, true);
    targetBuffer.setSize (2, numSamples, false, false, true);
    highBuffer  .setSize (2, numSamples, false, false, true);

    juce::dsp::AudioBlock<float> mainBlk (mainBus);
    juce::dsp::AudioBlock<const float> inBlk (mainBlk);
    juce::dsp::AudioBlock<float> subBlk (subBuffer);
    juce::dsp::AudioBlock<float> tgtBlk (targetBuffer);
    juce::dsp::AudioBlock<float> hiBlk  (highBuffer);

    crossover.process (inBlk, subBlk, tgtBlk, hiBlk);

    // ---- 7. Scalar detector -> per-sample gain, duck depth & collapse amount -
    // collapseAmt ramps 0 -> 1 across the first 3 dB of gain reduction, so the
    // M/S width hits 0% (mono) exactly when GR > 3 dB. Branchless: jlimit only.
    constexpr float invCollapseDb = 1.0f / 3.0f;
    for (int n = 0; n < numSamples; ++n)
    {
        const float g = envFollower.processSample (det[n]);   // serial, branchless
        gainRedux  [(size_t) n] = g;
        duckAmt    [(size_t) n] = 1.0f - g;
        collapseAmt[(size_t) n] = juce::jlimit (0.0f, 1.0f,
                                                -envFollower.getGrDb() * invCollapseDb);
    }

    // ---- 8. SIMD bulk gain reduction on the TARGET band only ---------------
    applyTimeVaryingGainSIMD (tgtBlk.getChannelPointer (0), gainRedux.data(), numSamples);
    applyTimeVaryingGainSIMD (tgtBlk.getChannelPointer (1), gainRedux.data(), numSamples);

    // ---- 9. Dynamic 65 Hz notch + M/S width collapse (scalar stereo) -------
    const float notchDepth   = apvts.getRawParameterValue ("notchDepth")->load();
    const float sideCollapse = apvts.getRawParameterValue ("sideCollapse")->load();

    auto* tl = tgtBlk.getChannelPointer (0);
    auto* tr = tgtBlk.getChannelPointer (1);

    for (int n = 0; n < numSamples; ++n)
    {
        const float dk = duckAmt[(size_t) n];

        // High-Q dynamic attenuation at 65 Hz: pull out the 65 Hz energy and
        // subtract a duck-scaled amount of it -> a notch that deepens as the
        // kick hits, strictly inside the 45-95 Hz band.
        const float e0 = notch65[0].processSample (tl[n]);
        const float e1 = notch65[1].processSample (tr[n]);
        float l = tl[n] - dk * notchDepth * e0;
        float r = tr[n] - dk * notchDepth * e1;

        // Collapse side -> mono in this band: full mono once GR > 3 dB
        // (collapseAmt saturates at 1). Branchless.
        const float width = 1.0f - collapseAmt[(size_t) n] * sideCollapse;
        mbdelax::dsp::MidSideMatrix::applyWidth (l, r, width);

        tl[n] = l;
        tr[n] = r;
    }

    // ---- 10. Tube saturation on the HIGH band (> 95 Hz, never ducked) ------
    // The +3 dB / 150 Hz shelf engages only while the external SC is live.
    tubeSat.setDrive (apvts.getRawParameterValue ("drive")->load());
    tubeSat.process (hiBlk, useExternal);

    // ---- 11. Recombine bands into the main bus -----------------------------
    mainBlk.copyFrom (subBlk);
    mainBlk.add (tgtBlk);
    mainBlk.add (hiBlk);

    // ---- 12. Dry/wet mix (against the time-aligned delayed dry) -------------
    const float mix = apvts.getRawParameterValue ("mix")->load();
    if (mix < 0.999f)
    {
        const float dryGain = 1.0f - mix;
        for (int ch = 0; ch < 2; ++ch)
        {
            mainBus.applyGain (ch, 0, numSamples, mix);
            mainBus.addFrom  (ch, 0, dryBuffer, ch, 0, numSamples, dryGain);
        }
    }

    // ---- 13. Output trim ----------------------------------------------------
    outputGain.setGainDecibels (apvts.getRawParameterValue ("output")->load());
    juce::dsp::ProcessContextReplacing<float> outCtx (mainBlk);
    outputGain.process (outCtx);
}

//==============================================================================
void MBDelaxAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto xml = apvts.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void MBDelaxAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes))
        apvts.replaceState (juce::ValueTree::fromXml (*xml));
}

//==============================================================================
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new MBDelaxAudioProcessor();
}