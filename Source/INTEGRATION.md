# MBDELAX — External Sidechain Matrix (JUCE 8)

Drop the `Source/` folder into your JUCE project. Files:

```
Source/
├─ MBDelaxAudioProcessor.h / .cpp   ← bus layout + processBlock (Preset 53 routing)
└─ dsp/
   ├─ MidSideMatrix.h               ← branchless M/S encode/decode + width collapse
   ├─ ThreeBandCrossover.h          ← phase-coherent LR4 split @ 45 / 95 Hz
   ├─ EnvelopeFollower.h            ← scalar, branchless detector + gain computer
   ├─ TubeSaturator.h               ← asymmetric tube + 150 Hz high-shelf
   └─ Preset53.h                    ← all preset constants (pure data)
```

## CMake — the sidechain bus needs no special flag

The aux input comes from `BusesProperties` in the constructor, so the host
(FL Studio's "Sidechain to this track") sees it automatically. You only need a
standard `juce_add_plugin`:

```cmake
juce_add_plugin(MBDELAX
    COMPANY_NAME      "Apolo Oliver Productions"
    PLUGIN_MANUFACTURER_CODE  Apol
    PLUGIN_CODE       Mbdx
    FORMATS           VST3 AU
    PRODUCT_NAME      "MBDELAX"
    IS_SYNTH          FALSE
    NEEDS_MIDI_INPUT  FALSE
    VST3_CATEGORIES   "Fx" "Dynamics")

target_sources(MBDELAX PRIVATE Source/MBDelaxAudioProcessor.cpp)
target_compile_features(MBDELAX PRIVATE cxx_std_17)
target_link_libraries(MBDELAX PRIVATE
    juce::juce_audio_utils juce::juce_dsp juce::juce_audio_processors)
```

## How FL Studio routing reaches the code

1. Insert MBDELAX on the Bassline channel.
2. Right-click the Kick channel → "Sidechain to this track" → pick MBDELAX's sidechain input.
3. In processBlock, getBusBuffer(buffer, true, 1) now carries the kick. getBus(true, 1)->isEnabled() flips to true, so useExternal becomes true and the detector follows the kick instead of the bassline.

If the host does NOT route a sidechain, the plugin falls back to internal detection (the bassline ducks itself), so it never goes silent.

## Design notes

- getBusBuffer signature fixed to getBusBuffer(buffer, true, 1) (JUCE 8).
- Lookahead is on the MAIN path; detector reads the un-delayed sidechain.
- Internal/external switch hoisted out of every per-sample loop (branchless hot path).
- Recursive envelope stays scalar; SIMDRegister<float> only applies the finished gain array.
- "+3 dB at 150 Hz" interpreted as a high-shelf lift to keep bass presence during the duck.
- Auto-release recomputed each block from AudioPlayHead BPM × release division (default 1/8).