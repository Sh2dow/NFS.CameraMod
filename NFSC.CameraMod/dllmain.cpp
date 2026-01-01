#include <algorithm>
#include <windows.h>
#include <cstdint>
#include <cmath>
#include "Game.h"
#include "minhook/include/MinHook.h"

// ==========================================================
// MATH
// ==========================================================
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329577f)
#endif

struct Mat4
{
    float m[4][4];
};

struct Vec3
{
    float x, y, z;
};

static inline Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline Vec3 mul(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static inline Vec3 cross(Vec3 a, Vec3 b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

static inline float len(Vec3 a) { return sqrtf(dot(a, a)); }

static inline Vec3 norm(Vec3 a)
{
    float l = len(a);
    return (l > 1e-6f) ? mul(a, 1.0f / l) : Vec3{0, 0, 0};
}

static inline Vec3 rotate(Vec3 v, Vec3 axis, float a)
{
    axis = norm(axis);
    float s = sinf(a), c = cosf(a);
    return add(
        add(mul(v, c), mul(cross(axis, v), s)),
        mul(axis, dot(axis, v) * (1.0f - c))
    );
}

static inline float signf(float v)
{
    return (v > 0.0f) ? 1.0f : (v < 0.0f ? -1.0f : 0.0f);
}

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static Mat4 Mul(const Mat4& A, const Mat4& B)
{
    Mat4 R = {};

    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            R.m[r][c] =
                A.m[r][0] * B.m[0][c] +
                A.m[r][1] * B.m[1][c] +
                A.m[r][2] * B.m[2][c] +
                A.m[r][3] * B.m[3][c];
        }
    }

    return R;
}

static Mat4 MakeAxisRotation(const Vec3& axis, float angle)
{
    Vec3 a = norm(axis); // MUST be normalized

    float s = sinf(angle);
    float c = cosf(angle);
    float t = 1.0f - c;

    Mat4 R = {};

    R.m[0][0] = t * a.x * a.x + c;
    R.m[0][1] = t * a.x * a.y + s * a.z;
    R.m[0][2] = t * a.x * a.z - s * a.y;
    R.m[0][3] = 0.0f;

    R.m[1][0] = t * a.x * a.y - s * a.z;
    R.m[1][1] = t * a.y * a.y + c;
    R.m[1][2] = t * a.y * a.z + s * a.x;
    R.m[1][3] = 0.0f;

    R.m[2][0] = t * a.x * a.z + s * a.y;
    R.m[2][1] = t * a.y * a.z - s * a.x;
    R.m[2][2] = t * a.z * a.z + c;
    R.m[2][3] = 0.0f;

    R.m[3][0] = 0.0f;
    R.m[3][1] = 0.0f;
    R.m[3][2] = 0.0f;
    R.m[3][3] = 1.0f;

    return R;
}

static Mat4 MakeMirrorY()
{
    Mat4 M = {};
    M.m[0][0] = 1.0f;
    M.m[1][1] = -1.0f; // <--- mirror Y
    M.m[2][2] = 1.0f;
    M.m[3][3] = 1.0f;
    return M;
}

// ==========================================================
// GLOBAL STRUCTS
// ==========================================================

// Set by wrapper hook, consumed by look-at hook
static volatile LONG gApplyUG2Flag = 0;

struct CameraState
{
    bool inited;
    float lastT;

    Vec3 prevFrom;
    Vec3 prevF;
    bool hasPrevF;

    Vec3 velDirFilt;
    Vec3 prevVelDirFilt;

    float speedFilt;
    float yawRateFilt;

    float yawBias;
    float rollBias;

    float gNorm;
    float horizonLock;

    // NEW (UG2)
    Vec3 upVis;
    Vec3 upVel;

    Vec3 upBase;
    bool hasUpBase;
} g_cam;

LONG InterlockedExchange(
    volatile LONG* Target,
    LONG Value
);

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

// ==========================================================
// WRAPPER HOOK (NAKED, SAFE)
// ==========================================================

static void* oCameraWrapper = nullptr; // trampoline

__declspec(naked) void hkCameraWrapper_Naked()
{
    __asm {
        pushfd
        pushad

        mov dword ptr [gApplyUG2Flag], 1

        popad
        popfd

        jmp dword ptr [oCameraWrapper]
        }
}

static bool HookWrapper(uintptr_t wrapperAddr)
{
    if (MH_CreateHook((LPVOID)wrapperAddr, &hkCameraWrapper_Naked, &oCameraWrapper) != MH_OK)
        return false;
    if (MH_EnableHook((LPVOID)wrapperAddr) != MH_OK)
        return false;
    return true;
}


// ------------------------------------------------------------
// INIT CAMERA STATE
// ------------------------------------------------------------
static void init_camera(void* outMatrix, Vec3* from, Vec3* to, Vec3* up)
{
    Vec3 fromEngine = *from;
    Vec3 toEngine = *to;
    Vec3 upEngine = norm(*up); // Carbon: Z-up

    Vec3 fEngine = norm(sub(toEngine, fromEngine));
    Vec3 rEngine = norm(cross(fEngine, upEngine)); // engine right
    Vec3 uEngine = norm(cross(rEngine, fEngine)); // re-orthogonalized up

    {
        g_cam.inited = true;
        g_cam.prevFrom = fromEngine;
        g_cam.prevVelDirFilt = fEngine;
        g_cam.velDirFilt = fEngine;
        g_cam.speedFilt = 0.0f;
        g_cam.yawRateFilt = 0.0f;
        g_cam.yawBias = 0.0f;
        g_cam.rollBias = 0.0f;
        g_cam.gNorm = 0.0f;
        g_cam.horizonLock = 1.0f;
    }
}

// ==========================================================
// LOOK-AT HOOK
// ==========================================================

using CreateLookAt_t = int(__cdecl*)(void* out, Vec3* from, Vec3* to, Vec3* up);
static CreateLookAt_t oCreateLookAt = nullptr;

static int __cdecl hkCreateLookAt(void* outMatrix, Vec3* from, Vec3* to, Vec3* up)
{
    if (!from || !to || !up)
        return oCreateLookAt(outMatrix, from, to, up);

    // Let Carbon build whatever it wants (director, blends, etc.)
    int ret = oCreateLookAt(outMatrix, from, to, up);

    // Only apply on "authoritative" camera update frames (set by wrapper)
    if (InterlockedExchange((volatile LONG*)&gApplyUG2Flag, 0) == 0)
        return ret;

    // Time
    float t = GetTimeSeconds_Safe();
    float dt = clampf(t - g_cam.lastT, 1.0f / 240.0f, 1.0f / 30.0f);
    g_cam.lastT = t;

    // Init once (AFTER engine call so pointers are valid and stable)
    if (!g_cam.inited)
    {
        init_camera(outMatrix, from, to, up);
        return ret;
    }

    // ------------------------------------------------------------
    // 1) UPDATE DYNAMICS FROM ENGINE CAMERA (Z-up correct)
    // ------------------------------------------------------------
    Vec3 fromEngine = *from;
    Vec3 toEngine = *to;

    // physical velocity from camera position
    Vec3 vel = sub(fromEngine, g_cam.prevFrom);
    g_cam.prevFrom = fromEngine;

    float speedU = len(vel) / dt;

    // ground direction (Z-up => XY plane)
    Vec3 vdir = vel;
    vdir.z = 0.0f;

    float vdirLen = len(vdir);
    float velResp = 1.0f - expf(-6.0f * dt);

    if (vdirLen > 1e-3f && speedU > 1e-2f)
    {
        vdir = mul(vdir, 1.0f / vdirLen);

        // filter direction + speed
        g_cam.velDirFilt = norm(add(mul(g_cam.velDirFilt, 1.0f - velResp),
                                    mul(vdir, velResp)));
        g_cam.speedFilt += (speedU - g_cam.speedFilt) * velResp;

        // yaw rate from filtered velocity direction (Z-up: XY cross)
        Vec3 prevF = g_cam.prevVelDirFilt;
        Vec3 curF = g_cam.velDirFilt;

        if (len(prevF) < 0.5f)
            prevF = curF;

        float yawS = (prevF.x * curF.y - prevF.y * curF.x);
        float yawRateRaw = yawS / dt;

        g_cam.prevVelDirFilt = curF;

        float yawResp = 1.0f - expf(-8.0f * dt);
        g_cam.yawRateFilt += (yawRateRaw - g_cam.yawRateFilt) * yawResp;
    }
    else
    {
        // decay when stopped
        g_cam.yawRateFilt += (0.0f - g_cam.yawRateFilt) * velResp;
        g_cam.speedFilt += (0.0f - g_cam.speedFilt) * velResp;
    }

    // ------------------------------------------------------------
    // 2) UG2-ish intents (stable, Carbon-safe)
    // ------------------------------------------------------------
    float yaw01 = clampf(fabsf(g_cam.yawRateFilt) / 0.6f, 0.0f, 1.0f);
    float speed01 = clampf(g_cam.speedFilt / 45.0f, 0.0f, 1.0f);

    float targetLock = 1.0f - 0.50f * yaw01 - 0.25f * speed01;
    targetLock = clampf(targetLock, 0.25f, 1.0f);
    g_cam.horizonLock += (targetLock - g_cam.horizonLock) * (1.0f - expf(-2.5f * dt));

    float lateralForce = fabsf(g_cam.yawRateFilt) * g_cam.speedFilt;
    float gTarget = clampf(lateralForce * 0.00006f, 0.0f, 1.0f);
    g_cam.gNorm += (gTarget - g_cam.gNorm) * (1.0f - expf(-3.0f * dt));

    // roll bias (lean into corner)
    float rollTarget =
        signf(-g_cam.yawRateFilt) *
        (0.4f * g_cam.gNorm + 0.6f * powf(g_cam.gNorm, 1.3f)) *
        12.0f; // instead of 3.0f

    g_cam.rollBias += (rollTarget - g_cam.rollBias) * (1.0f - expf(-6.0f * dt));

    // ------------------------------------------------------------
    // 3) PATCH FINAL VIEW MATRIX (post-director)
    // ------------------------------------------------------------
    Mat4& V = *(Mat4*)outMatrix;

    // --- OLD basis from view matrix (columns) ---
    Vec3 oldR = {V.m[0][0], V.m[1][0], V.m[2][0]};
    Vec3 oldU = {V.m[0][1], V.m[1][1], V.m[2][1]};
    Vec3 oldB = {V.m[0][2], V.m[1][2], V.m[2][2]}; // this is "back" in D3D view
    Vec3 oldT = {V.m[3][0], V.m[3][1], V.m[3][2]};

    // --- Recover camera world position from view matrix ---
    // For orthonormal view: CamPos = -(T.x*R + T.y*U + T.z*B)
    Vec3 camPos = mul(oldR, -oldT.x);
    camPos = add(camPos, mul(oldU, -oldT.y));
    camPos = add(camPos, mul(oldB, -oldT.z));

    // ------------------------------------------------------------
    // build UG2-like rolled basis using WORLD UP + forward
    // ------------------------------------------------------------
    const Vec3 worldUp = {0.0f, 0.0f, 1.0f};

    float lateral = g_cam.gNorm * 120.0f;
    float lift = g_cam.gNorm * clampf(g_cam.speedFilt / 18.0f, 0.0f, 1.0f) * 0.35f;

    // --- UG2 STYLE ROLL (NO HORIZON LOCK, NO FILTER) ---
    // lateral acceleration estimate (UU / s^2)
    float latAccel = fabsf(g_cam.yawRateFilt) * g_cam.speedFilt;
    float g01 = clampf(latAccel / 80.0f, 0.0f, 1.0f);

    float roll =
        -signf(g_cam.yawRateFilt) *
        powf(g01, 0.85f) *
        DEG2RAD(7.5f);

    roll = clampf(roll, -DEG2RAD(7.5f), DEG2RAD(7.5f));

    // Forward in WORLD space (look direction)
    Vec3 f = norm(sub(toEngine, fromEngine));
    if (len(f) < 1e-4f) f = oldB; // fallback to engine's column2

    // Horizon up: worldUp projected onto plane ⟂ forward
    Vec3 upProj = sub(worldUp, mul(f, dot(worldUp, f)));
    if (len(upProj) < 1e-4f) upProj = worldUp;
    upProj = norm(upProj);

    // Roll the horizon up around forward
    Vec3 u = rotate(upProj, f, roll);

    // IMPORTANT: match Carbon convention (col2 = forward):
    // Right = Up × Forward
    Vec3 r = norm(cross(u, f));
    u = norm(cross(f, r)); // re-orthogonalize

    // --- Align sign with engine basis to avoid mirror flips ---
    if (dot(r, oldR) < 0.0f)
    {
        r = mul(r, -1.0f);
        u = mul(u, -1.0f); // flip both to keep det positive
    }

    // Write new basis (columns): col0=right, col1=up, col2=forward
    V.m[0][0] = r.x;
    V.m[1][0] = r.y;
    V.m[2][0] = r.z;
    V.m[0][1] = u.x;
    V.m[1][1] = u.y;
    V.m[2][1] = u.z;
    V.m[0][2] = f.x;
    V.m[1][2] = f.y;
    V.m[2][2] = f.z;

    // Rebuild translation to keep SAME camera position
    V.m[3][0] = -dot(r, camPos);
    V.m[3][1] = -dot(u, camPos);
    V.m[3][2] = -dot(f, camPos);

    // Apply offsets by moving the camera position in WORLD, then recompute translation
    camPos = add(camPos, add(mul(r, lateral), mul(worldUp, lift)));

    V.m[3][0] = -dot(r, camPos);
    V.m[3][1] = -dot(u, camPos);
    V.m[3][2] = -dot(f, camPos);

    // DO NOT touch V.m[3][*] here

    return ret;
}

// ==========================================================
// INSTALL
// ==========================================================

static bool InstallHooks()
{
    if (MH_Initialize() != MH_OK)
        return false;

    // 1) Hook wrapper (authoritative camera update)
    if (!HookWrapper(Game::CameraWrapperAddr))
        return false;

    // 2) Hook CreateLookAt builder
    if (MH_CreateHook((LPVOID)Game::CreateLookAtAddr, &hkCreateLookAt, (LPVOID*)&oCreateLookAt) != MH_OK)
        return false;
    if (MH_EnableHook((LPVOID)Game::CreateLookAtAddr) != MH_OK)
        return false;

    // 3) Optional: disable tilts
    if (Game::DisableTiltsAddr)
    {
        static float DisableTilts = -1000.0f;
        *(float**)Game::DisableTiltsAddr = &DisableTilts;
    }

    OutputDebugStringA("[UG2Cam] Hooks installed.\n");
    return true;
}

// ==========================================================
// DLL ENTRY
// ==========================================================
BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(hModule);

        uintptr_t base = (uintptr_t)GetModuleHandleA(NULL);
        auto* dos = (IMAGE_DOS_HEADER*)base;
        auto* nt = (IMAGE_NT_HEADERS*)(base + dos->e_lfanew);

        if ((base + nt->OptionalHeader.AddressOfEntryPoint + (0x400000 - base)) == Game::Entry)
        {
            InstallHooks();
        }
        else
        {
            MessageBoxA(NULL, Game::Error, Game::Name, MB_ICONERROR);
            return FALSE;
        }
    }
    return TRUE;
}
