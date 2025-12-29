// dllmain.cpp - NFS Carbon: UG2-style look-into-turn
// Hook: Camera::SetCameraMatrix @ 0x004822F0

#include <windows.h>
#include <cstdint>
#include <cmath>
#include "minhook/include/MinHook.h"

struct bMatrix4
{
    float m[4][4];
};

// ---------------- hook ----------------
using SetCameraMatrix_t = void(__thiscall*)(void* cam, const bMatrix4* m, float fov);
static SetCameraMatrix_t oSetCameraMatrix = nullptr;

struct v3
{
    float x, y, z;
};

// ---------------- math helpers ----------------
static inline v3 v3_sub(v3 a, v3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline v3 v3_add(v3 a, v3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline v3 v3_mul(v3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float v3_dot(v3 a, v3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float v3_len(v3 a) { return sqrtf(v3_dot(a, a)); }

static inline v3 v3_norm(v3 a)
{
    float l = v3_len(a);
    return (l > 1e-6f) ? v3_mul(a, 1.0f / l) : v3{0, 0, 0};
}

static inline float clampf(float v, float a, float b) { return (v < a) ? a : ((v > b) ? b : v); }
static inline float signf(float v) { return (v < 0.0f) ? -1.0f : 1.0f; }

static inline v3 v3_cross(v3 a, v3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline v3 getRow(const bMatrix4& m, int r) { return {m.m[r][0], m.m[r][1], m.m[r][2]}; }

static inline void setRow(bMatrix4& m, int r, v3 v)
{
    m.m[r][0] = v.x;
    m.m[r][1] = v.y;
    m.m[r][2] = v.z;
}

// ---------------- yaw around LOCAL up (same as your working hook) ----------------
static void ApplyYaw_LocalUp_RowMajor(bMatrix4& m, float yaw)
{
    float s = sinf(yaw);
    float c = cosf(yaw);

    v3 r = v3_norm(getRow(m, 0)); // Right
    v3 f = v3_norm(getRow(m, 1)); // Forward
    v3 u = v3_norm(getRow(m, 2)); // Up

    v3 r2 = v3_add(v3_mul(r, c), v3_mul(f, s));
    v3 f2 = v3_add(v3_mul(f, c), v3_mul(r, -s));

    setRow(m, 0, v3_norm(r2));
    setRow(m, 1, v3_norm(f2));
    setRow(m, 2, u); // unchanged
    // row3 untouched
}

static void ApplyRoll_ViewSpace_RowMajor(bMatrix4& m, float roll)
{
    float s = sinf(roll);
    float c = cosf(roll);

    v3 r = v3_norm(getRow(m, 0)); // Right
    v3 f = v3_norm(getRow(m, 1)); // Forward (AXIS!)
    v3 u = v3_norm(getRow(m, 2)); // Up

    // Rodrigues rotation around Forward
    v3 r2 = v3_add(
        v3_mul(r, c),
        v3_add(
            v3_mul(v3_cross(f, r), s),
            v3_mul(f, v3_dot(f, r) * (1.0f - c))
        )
    );

    v3 u2 = v3_cross(r2, f); // re-derive up to keep orthonormal

    setRow(m, 0, v3_norm(r2));
    setRow(m, 2, v3_norm(u2));
    // Forward stays EXACTLY the same
}


// ---------------- timing (QPC dt) ----------------
static LARGE_INTEGER g_freq{};
static LARGE_INTEGER g_prev{};

static float GetDt()
{
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);

    if (g_prev.QuadPart == 0)
    {
        g_prev = now;
        return 0.0f;
    }

    double denom = double(g_freq.QuadPart);
    if (denom <= 0.0)
        return 0.0f;

    float dt = (float)(double(now.QuadPart - g_prev.QuadPart) / denom);
    g_prev = now;

    // clamp
    if (dt < 1.0f / 240.0f) dt = 1.0f / 240.0f;
    if (dt > 1.0f / 20.0f) dt = 1.0f / 20.0f;

    return dt;
}

using Sim_GetTime_t = float(__cdecl*)();
static Sim_GetTime_t Sim_GetTime = (Sim_GetTime_t)0x0075CF60; // verify in IDA

static float GetTimeSeconds_Safe()
{
    // 1) Try engine time
    if (Sim_GetTime)
    {
        __try
        {
            float t = Sim_GetTime();
            if (t > 0.001f && t < 1e7f)
                return t;
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
        }
    }

    // 2) Fallback: QPC
    if (g_freq.QuadPart == 0)
    {
        QueryPerformanceFrequency(&g_freq);
        QueryPerformanceCounter(&g_prev);

        if (g_freq.QuadPart == 0)
            return 0.0f;
    }

    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);

    return (float)(
        double(now.QuadPart - g_prev.QuadPart) /
        double(g_freq.QuadPart)
    );
}

// ---------------- look-into-turn state ----------------
struct CamState
{
    bool inited = false;

    // timing
    float lastSimT = 0.0f;
    float lastUpdateT = 0.0f;

    // motion
    v3 prevPos{};
    v3 prevVelDir{}; // normalized horizontal velocity dir
    v3 prevPosReset{};

    // outputs
    float yawRateRaw = 0.0f; // rad/s (raw, no deadzone)
    float yawBias = 0.0f; // radians (look-into-turn)
    float rollBias = 0.0f; // radians (bank)
    float gNorm = 0.0f; // 0..1 lateral load proxy

    // cached per-frame so second call uses same values
    float cachedYaw = 0.0f;
    float cachedRoll = 0.0f;
    float cachedLateral = 0.0f;
} g_cam;

// ---------------- compute from VELOCITY (UG2-style) ----------------
static void ComputeUG2Like(float dt, const v3& posNow)
{
    // horizontal velocity dir
    v3 vel = v3_sub(posNow, g_cam.prevPos);
    g_cam.prevPos = posNow;

    // if basically not moving, decay effects
    float speedU = v3_len(vel) / dt; // units/sec
    v3 vdir = vel;
    vdir.y = 0.0f;

    if (v3_len(vdir) < 1e-3f || speedU < 1e-2f)
    {
        // decay to neutral
        float a = 1.0f - expf(-6.0f * dt);
        g_cam.yawRateRaw += (0.0f - g_cam.yawRateRaw) * a;
        g_cam.gNorm += (0.0f - g_cam.gNorm) * a;
        g_cam.yawBias += (0.0f - g_cam.yawBias) * a;
        g_cam.rollBias += (0.0f - g_cam.rollBias) * a;
        return;
    }

    vdir = v3_norm(vdir);

    // yaw rate from change in horizontal velocity direction
    // signed angle rate around world Y: cross(prev,cur).y / dt
    v3 prev = g_cam.prevVelDir;
    if (v3_len(prev) < 0.5f) prev = vdir; // safety

    float yawS = (prev.x * vdir.z - prev.z * vdir.x); // == cross(prev,cur).y
    float yawRateRaw = yawS / dt; // rad/s (small-angle)
    g_cam.prevVelDir = vdir;

    g_cam.yawRateRaw = yawRateRaw;

    // ---- deadzone only for VISUAL yaw, not for load ----
    const float kYawDeadzone = 0.02f; // rad/s
    float yawRateFiltered = (fabsf(yawRateRaw) < kYawDeadzone) ? 0.0f : yawRateRaw;

    // ---- lateral load proxy (UG2 feel) ----
    // proportional to |yawRate| * speed
    float lateralForce = fabsf(yawRateRaw) * speedU; // "units/s^2" proxy

    // normalize (tune this ONE number!)
    // start with 1e-5..1e-4 depending on your world scale
    const float kLoadNorm = 0.00006f;
    float gTarget = clampf(lateralForce * kLoadNorm, 0.0f, 1.0f);

    // smooth gNorm (heavy camera)
    float aG = 1.0f - expf(-3.0f * dt);
    g_cam.gNorm += (gTarget - g_cam.gNorm) * aG;

    // ---- yaw look-ahead (UG2-ish) ----
    const float kLookAhead = 0.14f;
    const float kMaxYaw = 0.045f; // ~2.6°
    const float kRespYaw = 10.0f;

    float yawTarget = clampf(yawRateFiltered * kLookAhead, -kMaxYaw, kMaxYaw);
    float aYaw = 1.0f - expf(-kRespYaw * dt);
    g_cam.yawBias += (yawTarget - g_cam.yawBias) * aYaw;

    // ---- roll bank from load (UG2 part) ----
    const float kMaxRoll = 0.45f;

    float rollTarget =
        signf(yawRateRaw) *
        (0.25f * g_cam.gNorm + 0.75f * powf(g_cam.gNorm, 1.5f)) *
        kMaxRoll;

    // suppress roll at very low speed
    float speedFade = clampf(speedU / 18.0f, 0.0f, 1.0f);
    rollTarget *= speedFade;

    // asym response
    float resp = (fabsf(rollTarget) < fabsf(g_cam.rollBias)) ? 9.0f : 4.0f;
    float aRoll = 1.0f - expf(-resp * dt);
    g_cam.rollBias += (rollTarget - g_cam.rollBias) * aRoll;
}

// ---------------- hook ----------------
static void __fastcall hkSetCameraMatrix(void* cam, void*, const bMatrix4* m, float fov)
{
    if (!m)
    {
        oSetCameraMatrix(cam, m, fov);
        return;
    }

    // Gameplay-ish FOV filter
    if (fov < 0.015f || fov > 0.040f)
    {
        oSetCameraMatrix(cam, m, fov);
        return;
    }

    // ------------------------------------------------------------
    // Make a LOCAL COPY that we will MODIFY and PASS INTO ENGINE
    // ------------------------------------------------------------
    bMatrix4 mm = *m;
    bMatrix4& live = mm;

    // ------------------------------------------------------------
    // Position
    // ------------------------------------------------------------
    v3 pos = {live.m[3][0], live.m[3][1], live.m[3][2]};

    // ------------------------------------------------------------
    // Time
    // ------------------------------------------------------------
    if (!Sim_GetTime)
    {
        oSetCameraMatrix(cam, &mm, fov);
        return;
    }

    float t = Sim_GetTime();

    // ------------------------------------------------------------
    // First-time init
    // ------------------------------------------------------------
    if (!g_cam.inited)
    {
        g_cam.inited = true;
        g_cam.lastSimT = t;
        g_cam.lastUpdateT = t;

        g_cam.prevPos = pos;
        g_cam.prevPosReset = pos;
        g_cam.prevVelDir = v3{1, 0, 0};

        g_cam.yawRateRaw = 0.0f;
        g_cam.yawBias = 0.0f;
        g_cam.rollBias = 0.0f;
        g_cam.gNorm = 0.0f;

        g_cam.cachedYaw = 0.0f;
        g_cam.cachedRoll = 0.0f;
        g_cam.cachedLateral = 0.0f;

        oSetCameraMatrix(cam, &mm, fov);
        return;
    }

    // ------------------------------------------------------------
    // dt
    // ------------------------------------------------------------
    float dt = t - g_cam.lastSimT;
    g_cam.lastSimT = t;
    dt = clampf(dt, 1.0f / 240.0f, 1.0f / 30.0f);

    // ------------------------------------------------------------
    // Teleport / reset detection
    // ------------------------------------------------------------
    v3 dposReset = v3_sub(pos, g_cam.prevPosReset);
    float dpReset = v3_len(dposReset);
    g_cam.prevPosReset = pos;

    if (dpReset > 50000.0f)
    {
        g_cam.prevPos = pos;
        g_cam.prevVelDir = v3{1, 0, 0};
        g_cam.yawRateRaw = 0.0f;
        g_cam.yawBias = 0.0f;
        g_cam.rollBias = 0.0f;
        g_cam.gNorm = 0.0f;

        g_cam.cachedYaw = 0.0f;
        g_cam.cachedRoll = 0.0f;
        g_cam.cachedLateral = 0.0f;

        oSetCameraMatrix(cam, &mm, fov);
        return;
    }

    // ------------------------------------------------------------
    // Update ONCE per sim frame
    // ------------------------------------------------------------
    const float kSameFrameEps = 1e-5f;
    if (fabsf(t - g_cam.lastUpdateT) > kSameFrameEps)
    {
        g_cam.lastUpdateT = t;

        ComputeUG2Like(dt, pos);

        const float kMaxLateral = 1200.0f; // Carbon units
        const float kLatFromLoad = 1.10f;

        float lateral =
            signf(g_cam.yawRateRaw) *
            g_cam.gNorm *
            (kLatFromLoad * kMaxLateral);

        g_cam.cachedYaw = g_cam.yawBias;
        g_cam.cachedRoll = g_cam.rollBias;
        g_cam.cachedLateral = clampf(lateral, -kMaxLateral, kMaxLateral);
    }

    float yaw = g_cam.cachedYaw;
    float roll = g_cam.cachedRoll;
    float lateral = g_cam.cachedLateral;

    // ------------------------------------------------------------
    // Translation (lateral + small lift)
    // ------------------------------------------------------------
    v3 right = v3_norm({live.m[0][0], live.m[0][1], live.m[0][2]});
    v3 up = v3_norm({live.m[2][0], live.m[2][1], live.m[2][2]});

    pos = v3_add(pos, v3_mul(right, lateral));
    pos = v3_add(pos, v3_mul(up, fabsf(lateral) * 0.12f));

    live.m[3][0] = pos.x;
    live.m[3][1] = pos.y;
    live.m[3][2] = pos.z;

    // ------------------------------------------------------------
    // Orientation
    // ------------------------------------------------------------
    if (fabsf(yaw) > 1e-6f)
        ApplyYaw_LocalUp_RowMajor(live, yaw);

    if (fabsf(roll) > 1e-6f)
        ApplyRoll_ViewSpace_RowMajor(live, roll);

    // ------------------------------------------------------------
    // PASS MODIFIED MATRIX INTO ENGINE
    // ------------------------------------------------------------
    oSetCameraMatrix(cam, &mm, fov);
}

// ---------------- init ----------------
static DWORD WINAPI HookThread(LPVOID)
{
    QueryPerformanceFrequency(&g_freq);
    QueryPerformanceCounter(&g_prev); // initialize prev so first dt=0

    constexpr uintptr_t kAddr = 0x004822F0; // YOUR Carbon build

    if (MH_Initialize() != MH_OK) return 0;
    if (MH_CreateHook((LPVOID)kAddr, &hkSetCameraMatrix, (LPVOID*)&oSetCameraMatrix) != MH_OK) return 0;
    if (MH_EnableHook((LPVOID)kAddr) != MH_OK) return 0;

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
