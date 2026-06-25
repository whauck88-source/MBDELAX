#pragma once

#include <juce_dsp/juce_dsp.h>

namespace mbdelax::dsp
{
/**
    Branch-free Mid/Side encoder & decoder.

        Encode:  M = (L + R) * 0.5     S = (L - R) * 0.5
        Decode:  L =  M + S            R =  M - S

    There is not a single conditional in here on purpose: the matrix is pure
    arithmetic so the compiler can keep it in registers and (where it is called
    in a flat loop) auto-vectorise it. We use it to collapse the side channel to
    0% width inside the ducking band, which removes the phase-cancellation risk
    you get when a mono kick fights a stereo bass below ~95 Hz.
*/
struct MidSideMatrix
{
    /** In place: on return, l holds Mid, r holds Side. */
    static inline void encode (float& l, float& r) noexcept
    {
        const float m = (l + r) * 0.5f;
        const float s = (l - r) * 0.5f;
        l = m;
        r = s;
    }

    /** In place: feed Mid in m and Side in s; on return m holds L, s holds R. */
    static inline void decode (float& m, float& s) noexcept
    {
        const float l = m + s;
        const float r = m - s;
        m = l;
        s = r;
    }

    /**
        Encode -> scale the side component by 'width' -> decode, all in one pass.
        width == 1.0f  -> untouched stereo
        width == 0.0f  -> 100% mono (side fully removed)
        Anything in between is a continuous, click-free narrowing. Branchless.
    */
    static inline void applyWidth (float& l, float& r, float width) noexcept
    {
        const float m = (l + r) * 0.5f;
        const float s = (l - r) * 0.5f * width;
        l = m + s;
        r = m - s;
    }
};
} // namespace mbdelax::dsp
