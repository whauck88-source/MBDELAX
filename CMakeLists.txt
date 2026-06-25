cmake_minimum_required(VERSION 3.22)
project(MBDELAX VERSION 1.0.0)

# The bundled JUCE checkout is the folder "JUCE-master".
add_subdirectory(JUCE-master)

juce_add_plugin(MBDELAX
    COMPANY_NAME              "Apolo Oliver Productions"
    PLUGIN_MANUFACTURER_CODE  Apol
    PLUGIN_CODE               Mbdx
    FORMATS                   VST3 AU
    PRODUCT_NAME              "MBDELAX"
    IS_SYNTH                  FALSE
    NEEDS_MIDI_INPUT          FALSE
    NEEDS_MIDI_OUTPUT         FALSE
    IS_MIDI_EFFECT            FALSE
    # Required so the host (FL Studio) exposes the external sidechain input bus
    # via "Sidechain to this track".
    VST3_CATEGORIES           "Fx" "Dynamics")

target_sources(MBDELAX PRIVATE
    Source/MBDelaxAudioProcessor.cpp)

# Header-only DSP modules: not compiled directly, but listed so they appear in
# IDE project trees and trigger rebuilds when edited.
target_sources(MBDELAX PRIVATE
    Source/MBDelaxAudioProcessor.h
    Source/dsp/MidSideMatrix.h
    Source/dsp/ThreeBandCrossover.h
    Source/dsp/EnvelopeFollower.h
    Source/dsp/TubeSaturator.h
    Source/dsp/Preset53.h)

target_compile_features(MBDELAX PRIVATE cxx_std_17)

target_compile_definitions(MBDELAX PRIVATE
    JUCE_VST3_CAN_REPLACE_VST2=0
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0)

target_link_libraries(MBDELAX PRIVATE
    juce::juce_audio_utils
    juce::juce_dsp
    juce::juce_audio_processors
    PUBLIC
    juce::juce_recommended_config_flags
    juce::juce_recommended_lto_flags
    juce::juce_recommended_warning_flags)
