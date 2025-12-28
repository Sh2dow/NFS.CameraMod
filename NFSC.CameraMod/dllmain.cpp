// dllmain.cpp - NFS Carbon: hook ApplyCameraShake at 0x0069ADF0 (your Carbon build)
// Minimal: no engine helpers, yaw applied directly.

#include <windows.h>
#include <cstdint>
#include <cmath>
#include "minhook/include/MinHook.h"

struct bMatrix4 { float m[4][4]; };

static inline float clampf(float v, float a, float b)
{
    return (v < a) ? a : (v > b) ? b : v;
}
// Carbon Sim::GetTime() - YOU MUST VERIFY THIS ADDRESS IN *CARBON* IDA.
// If unsure, set to nullptr and we’ll fall back to QPC time.
using Sim_GetTime_t = float(__cdecl*)();
static Sim_GetTime_t Sim_GetTime = (Sim_GetTime_t)0x0075CF60; // verify for Carbon!

using ApplyCameraShake_t = void(__cdecl*)(int id, bMatrix4* m);
static ApplyCameraShake_t oApplyCameraShake = nullptr;

static LARGE_INTEGER g_freq{};
static LARGE_INTEGER g_t0{};

static float GetTimeSeconds_Safe()
{
    // Preferred: engine time
    if (Sim_GetTime)
    {
        __try {
            float t = Sim_GetTime();
            // Some wrong addresses return constant 0 or insane values
            if (t > 0.001f && t < 1e7f) return t;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            // fallthrough
        }
    }

    // Fallback: QPC time
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    return (float)((double)(now.QuadPart - g_t0.QuadPart) / (double)g_freq.QuadPart);
}

struct v3 { float x,y,z; };

static inline v3 v3_add(v3 a, v3 b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }
static inline v3 v3_sub(v3 a, v3 b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
static inline v3 v3_mul(v3 a, float s){ return {a.x*s,a.y*s,a.z*s}; }
static inline float v3_dot(v3 a, v3 b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3_len(v3 a){ return sqrtf(v3_dot(a,a)); }
static inline v3 v3_norm(v3 a){
    float l = v3_len(a);
    return (l > 1e-6f) ? v3_mul(a, 1.0f/l) : v3{0,0,0};
}

static inline v3 getRow(const bMatrix4& m, int r){ return {m.m[r][0], m.m[r][1], m.m[r][2]}; }
static inline void setRow(bMatrix4& m, int r, v3 v){ m.m[r][0]=v.x; m.m[r][1]=v.y; m.m[r][2]=v.z; }

// Yaw in the (Right,Forward) plane around local Up.
// This preserves translation (row3) and keeps Up unchanged.
static void ApplyYaw_LocalUp_RowMajor(bMatrix4& m, float yaw)
{
    float s = sinf(yaw);
    float c = cosf(yaw);

    v3 r = getRow(m, 0); // Right
    v3 f = getRow(m, 1); // Forward
    v3 u = getRow(m, 2); // Up

    // Ensure basis is sane-ish (optional but helps if engine gives slightly non-ortho)
    r = v3_norm(r);
    f = v3_norm(f);
    u = v3_norm(u);

    // Rotate r and f around u (since r,f are perpendicular to u)
    v3 r2 = v3_add(v3_mul(r, c), v3_mul(f, s));
    v3 f2 = v3_sub(v3_mul(f, c), v3_mul(r, s));

    // Re-orthonormalize lightly (prevents drift / weird scale)
    r2 = v3_norm(r2);
    f2 = v3_norm(f2);

    setRow(m, 0, r2);
    setRow(m, 1, f2);
    setRow(m, 2, u);   // unchanged
    // row3 (translation) untouched
}

static void __cdecl hkApplyCameraShake(int id, bMatrix4* m)
{
    oApplyCameraShake(id, m);
    if (!m || m->m[3][3] != 1.0f)
        return;

    float t = GetTimeSeconds_Safe();

    float yaw =
        sinf(t * 0.85f) * 0.0022f +
        sinf(t * 0.45f) * 0.0014f;

    float scale = 1.0f;
    if (id == 1) scale = 0.6f;
    if (id == 2) scale = 0.35f;

    yaw *= scale;
    yaw = clampf(yaw, -0.003f, 0.003f);

    ApplyYaw_LocalUp_RowMajor(*m, yaw);
}

static DWORD WINAPI HookThread(LPVOID)
{
    QueryPerformanceFrequency(&g_freq);
    QueryPerformanceCounter(&g_t0);

    constexpr uintptr_t kApplyCameraShake = 0x0069ADF0; // your Carbon address

    if (MH_Initialize() != MH_OK) return 0;
    if (MH_CreateHook((LPVOID)kApplyCameraShake, (LPVOID)&hkApplyCameraShake, (LPVOID*)&oApplyCameraShake) != MH_OK) return 0;
    if (MH_EnableHook((LPVOID)kApplyCameraShake) != MH_OK) return 0;

    return 0;
}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(hModule);
        CreateThread(nullptr, 0, HookThread, nullptr, 0, nullptr);
    }
    return TRUE;
}
