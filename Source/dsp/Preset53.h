#pragma once

namespace mbdelax::presets
{
/**
    PRESET 53 - "KICK/BASS SIDECHAIN SURGERY" (Brazilian Tribal House)

    Inserted on the Bassline channel, receiving the Kick via the external
    sidechain bus (FL Studio: "Sidechain to this track"). Everything below is
    pure data so the values can be exposed in the UI later without touching the
    DSP graph.
*/
struct Preset53
{
    // --- Crossover (the band that actually ducks) -----------------------------
    // LR4 = 24 dB/oct -> the 45..95 Hz band-pass the spec calls for.
    static constexpr float crossoverLowHz   = 45.0f;   // below this = clean sub
    static constexpr float crossoverHighHz  = 95.0f;   // above this = bypassed/tube

    // --- Dynamic EQ surgery (Pultec-style bell) -------------------------------
    static constexpr float dynamicNotchHz   = 65.0f;   // high-Q attenuation centre
    static constexpr float dynamicNotchQ    = 10.0f;   // spec: Q = 10
    static constexpr float dynamicNotchDepth= 1.0f;    // 0..1 multiplier on the cut

    // --- Detector / envelope --------------------------------------------------
    static constexpr float lookaheadMs      = 1.5f;    // FIR delay on the MAIN path
    static constexpr float attackMs         = 2.0f;
    static constexpr float thresholdDb      = -28.0f;
    static constexpr float ratio            = 8.0f;

    // Host-synced auto-release: release time = quarterNoteMs * releaseDivision.
    // 0.25 -> 1/16 note (semicolcheia). Recomputed every block from the host BPM.
    static constexpr float releaseDivision  = 0.25f;

    // --- Stereo behaviour during the duck ------------------------------------
    // Width collapses to 0% (mono) once gain reduction exceeds 3 dB.
    static constexpr float sideCollapse     = 1.0f;    // 1 = force 100% mono at peak duck

    // --- Tube stage on the high band -----------------------------------------
    static constexpr float tubeDrive        = 0.35f;   // 0..1
};
} // namespace mbdelax::presets
