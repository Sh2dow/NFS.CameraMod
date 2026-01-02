#include <windows.h>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <mutex>
#include "Math.h"
#include "Game.h"
#include "Log.h"
#include "minhook/include/MinHook.h"
#include <d3d9.h>

// ==========================================================
// GLOBAL STRUCTS
// ==========================================================
// Set by wrapper hook, consumed by look-at hook
static volatile LONG gApplyUG2Flag = 0;

constexpr float PI = 3.14159265358979323846f;

using CubicUpdate_t = int(__thiscall*)(void* self, float dt);
using CreateLookAt_t = void(__cdecl*)(Mat4* mat, Vec3* eye, Vec3* center, Vec3* up);

static CubicUpdate_t  oCubicUpdate  = nullptr;
static CreateLookAt_t oCreateLookAt = nullptr;

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

// ------------------------------------------------------------
// INIT CAMERA STATE
// ------------------------------------------------------------
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
    
    bool timeValid;
} g_cam;

static void init_camera(Vec3* eye)
{
    g_cam.inited = true;

    g_cam.prevFrom = *eye;

    // Seed velocity direction safely (no impulses)
    g_cam.velDirFilt     = v3(0.0f, 0.0f, 1.0f); // MW forward (XZ)
    g_cam.prevVelDirFilt = g_cam.velDirFilt;

    g_cam.speedFilt   = 0.0f;
    g_cam.yawRateFilt = 0.0f;
    g_cam.rollBias    = 0.0f;
    g_cam.gNorm       = 0.0f;
    g_cam.horizonLock = 1.0f;

    // force dt resync next frame
    g_cam.timeValid = false;
}

// ==========================================================
// LOOK-AT HOOK
// ==========================================================
int __fastcall hkCubicUpdate(void* self, void*, float dt)
{
    InterlockedExchange(&gApplyUG2Flag, 1);
    return oCubicUpdate(self, dt);
}

// MW look-at writes basis into COLUMNS:
// col0 = right, col1 = up, col2 = back (D3D view convention)
static void ApplyRollKeepPos(Mat4* V, float rollRad)
{
    // basis is in COLUMNS
    Vec3 R = v3(V->m[0][0], V->m[1][0], V->m[2][0]);
    Vec3 U = v3(V->m[0][1], V->m[1][1], V->m[2][1]);
    Vec3 B = v3(V->m[0][2], V->m[1][2], V->m[2][2]); // "back" (D3D view convention)

    // translation is in ROW 3 (matches your Mat4 usage in MW code)
    float Tx = V->m[3][0];
    float Ty = V->m[3][1];
    float Tz = V->m[3][2];

    // Recover camera world position from view (orthonormal)
    // camPos = -(Tx*R + Ty*U + Tz*B)
    Vec3 camPos = add(add(mul(R, -Tx), mul(U, -Ty)), mul(B, -Tz));

    // forward axis = -B
    Vec3 F = norm(mul(B, -1.0f));

    // rotate R and U around F
    Vec3 R2 = rotate(R, F, rollRad);
    Vec3 U2 = rotate(U, F, rollRad);
    Vec3 B2 = mul(norm(cross(R2, U2)), 1.0f); // this is FORWARD; we need BACK:

    // write new basis columns
    V->m[0][0] = R2.x; V->m[1][0] = R2.y; V->m[2][0] = R2.z;
    V->m[0][1] = U2.x; V->m[1][1] = U2.y; V->m[2][1] = U2.z;
    V->m[0][2] = B2.x; V->m[1][2] = B2.y; V->m[2][2] = B2.z;

    // rebuild translation so camera stays at camPos
    V->m[3][0] = -dot(R2, camPos);
    V->m[3][1] = -dot(U2, camPos);
    V->m[3][2] = -dot(B2, camPos);
}

static float gPrevYaw = 0.0f;

float ComputeYawDelta(const Mat4* V)
{
    // Forward = -B
    Vec3 B = v3(V->m[0][2], V->m[1][2], V->m[2][2]);
    Vec3 F = norm(mul(B, -1.0f));

    float yaw = atan2f(F.x, F.z);
    float dyaw = yaw - gPrevYaw;
    gPrevYaw = yaw;

    // unwrap
    if (dyaw >  PI) dyaw -= 2.0f * PI;
    if (dyaw < -PI) dyaw += 2.0f * PI;

    return dyaw;
}

static float gCamRoll = 0.0f;
static float gCamRollVel = 0.0f;

void UpdateCameraRoll(float yawDelta, float speed, float dt)
{
    const float rollStrength = 2.2f;   // UG2-ish
    const float rollMax      = 0.35f;  // ~20 degrees
    const float stiffness    = 8.0f;   // response
    const float damping      = 2.0f;

    float targetRoll = clampf(yawDelta * speed * rollStrength,
                             -rollMax, rollMax);

    // critically damped spring
    float accel = (targetRoll - gCamRoll) * stiffness
                  - gCamRollVel * damping;

    gCamRollVel += accel * dt;
    gCamRoll    += gCamRollVel * dt;
}

void __cdecl hkCreateLookAtMatrix(Mat4* mat, Vec3* eye, Vec3* center, Vec3* up)
{
    // Let MW build the camera fully (director, blends, etc.)
    oCreateLookAt(mat, eye, center, up);

    if (InterlockedExchange(&gApplyUG2Flag, 0) == 0)
        return;

    if (!g_cam.inited)
    {
        init_camera(eye);
        return;
    }
    // ------------------------------------------------------------
    // TIME (safe)
    // ------------------------------------------------------------
    float t = GetTimeSeconds_Safe();

    if (!g_cam.timeValid)
    {
        g_cam.lastT = t;
        g_cam.timeValid = true;
        g_cam.prevFrom = *eye;
        return;
    }

    float rawDt = t - g_cam.lastT;
    g_cam.lastT = t;

    if (rawDt <= 0.0f || rawDt > 0.5f)
        return;

    float dt = clampf(rawDt, 1.0f / 240.0f, 1.0f / 30.0f);

    // ------------------------------------------------------------
    // 1) CAMERA-DERIVED VELOCITY (MW: Y-up → XZ plane)
    // ------------------------------------------------------------
    Vec3 fromNow = *eye;
    Vec3 vel = sub(fromNow, g_cam.prevFrom);
    g_cam.prevFrom = fromNow;

    float speed = len(vel) / dt; // world units / sec

    Vec3 vdir = vel;
    vdir.y = 0.0f; // MW ground plane

    float vdirLen = len(vdir);

    float velResp = 1.0f - expf(-6.0f * dt);

    if (vdirLen > 1e-3f && speed > 1e-2f)
    {
        vdir = mul(vdir, 1.0f / vdirLen);

        // filter direction
        g_cam.velDirFilt = norm(add(
            mul(g_cam.velDirFilt, 1.0f - velResp),
            mul(vdir, velResp)));

        // filter speed
        g_cam.speedFilt += (speed - g_cam.speedFilt) * velResp;

        // --------------------------------------------------------
        // 2) YAW RATE FROM VELOCITY DIRECTION (MW axes!)
        // --------------------------------------------------------
        Vec3 prevF = g_cam.prevVelDirFilt;
        Vec3 curF  = g_cam.velDirFilt;

        if (len(prevF) < 0.5f)
            prevF = curF;

        // Y-up cross (XZ plane)
        float yawS = (prevF.x * curF.z - prevF.z * curF.x);
        float yawRateRaw = yawS / dt;

        g_cam.prevVelDirFilt = curF;

        float yawResp = 1.0f - expf(-8.0f * dt);
        g_cam.yawRateFilt += (yawRateRaw - g_cam.yawRateFilt) * yawResp;
    }
    else
    {
        // decay when stopped
        g_cam.yawRateFilt += (0.0f - g_cam.yawRateFilt) * velResp;
        g_cam.speedFilt   += (0.0f - g_cam.speedFilt)   * velResp;
    }

    // ------------------------------------------------------------
    // 3) UG2-ACCURATE ROLL INTENT
    // ------------------------------------------------------------
    float yawAbs  = fabsf(g_cam.yawRateFilt);
    float yaw01   = saturate(yawAbs / 0.12f);   // UG2 sensitivity
    float speed01 = saturate(g_cam.speedFilt / 38.0f);

    float rollTarget =
        signf(-g_cam.yawRateFilt) *
        yaw01 *
        speed01 *
        DEG2RAD(8.5f);   // UG2 max roll

    float speedDamp = 1.0f - 0.25f * speed01;
    rollTarget *= speedDamp;

    g_cam.rollBias += (rollTarget - g_cam.rollBias) *
                      (1.0f - expf(-7.5f * dt));

    // ------------------------------------------------------------
    // 4) APPLY ROLL (KEEP POSITION)
    // ------------------------------------------------------------
    ApplyRollKeepPos(mat, g_cam.rollBias);
}

// ==========================================================
// INSTALL
// ==========================================================

static bool InstallHooks()
{
    MH_Initialize();

    MH_CreateHook(
        (LPVOID)Game::DisableTiltsAddr,
        hkCubicUpdate,
        (LPVOID*)&oCubicUpdate
    );

    MH_CreateHook(
        (LPVOID)Game::CreateLookAtAddr,
        hkCreateLookAtMatrix,
        (LPVOID*)&oCreateLookAt
    );

    MH_EnableHook(MH_ALL_HOOKS);
    
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
