#pragma once

#include <windows.h>
#include "Game.h"

// ==========================================================
// TIME
// ==========================================================
using Sim_GetTime_t = float(__cdecl*)();
static Sim_GetTime_t Sim_GetTime = (Sim_GetTime_t)Game::SimGetTimeAddr; // verify in IDA

static float GetTimeSeconds_Safe()
{
    static bool qpcInit = false;
    static LARGE_INTEGER freq{};
    static LARGE_INTEGER t0{};

    // --- Engine time ONLY if stable ---
    if (Sim_GetTime)
    {
        __try
        {
            float t = Sim_GetTime();

            // Reject loading / pre-world values
            if (t > 0.1f && t < 1e7f)
                return t;
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
            // ignore
        }
    }

    // --- QPC fallback ---
    if (!qpcInit)
    {
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&t0);
        qpcInit = true;
        return 0.0f; // IMPORTANT
    }

    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);

    return float(double(now.QuadPart - t0.QuadPart) /
        double(freq.QuadPart));
}