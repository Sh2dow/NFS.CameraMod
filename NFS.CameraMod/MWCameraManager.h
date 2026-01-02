#pragma once
#include "Math.h"
#include "MostWanted.h"
#include <d3d9.h>

#include "TimeManager.h"
#include "minhook/include/MinHook.h"

// ==========================================================
// GLOBAL STRUCTS
// ==========================================================
// Set by wrapper hook, consumed by look-at hook
static volatile LONG gApplyUG2Flag = 0;

constexpr float PI = 3.14159265358979323846f;

using CubicUpdate_t = int(__thiscall*)(void* self, float dt);
using CreateLookAt_t = void(__cdecl*)(Mat4* mat, Vec3* eye, Vec3* center, Vec3* up);

static CubicUpdate_t oCubicUpdate = nullptr;
static CreateLookAt_t oCreateLookAt = nullptr;

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
    float rollTargetFilt;

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
    g_cam.velDirFilt = v3(0.0f, 0.0f, 1.0f); // MW forward (XZ)
    g_cam.prevVelDirFilt = g_cam.velDirFilt;
    g_cam.rollTargetFilt = 0.0f;

    g_cam.speedFilt = 0.0f;
    g_cam.yawRateFilt = 0.0f;
    g_cam.rollBias = 0.0f;
    g_cam.gNorm = 0.0f;
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
    // --- read basis (COLUMNS) ---
    Vec3 R = v3(V->m[0][0], V->m[1][0], V->m[2][0]); // right
    Vec3 U = v3(V->m[0][1], V->m[1][1], V->m[2][1]); // up
    Vec3 B = v3(V->m[0][2], V->m[1][2], V->m[2][2]); // back

    // --- translation ---
    float Tx = V->m[3][0];  
    float Ty = V->m[3][1];
    float Tz = V->m[3][2];

    // Recover camera world position
    Vec3 camPos = add(add(mul(R, -Tx), mul(U, -Ty)), mul(B, -Tz));

    // ------------------------------------------------------------
    // 1) HORIZON LOCK (MW / UG2: Z-up)
    // ------------------------------------------------------------
    Vec3 F = norm(mul(B, -1.0f)); // forward
    // Use whichever world-up points closest to current camera up
    Vec3 UpWorldPos = v3(0.0f, 0.0f, 1.0f);
    Vec3 UpWorldNeg = v3(0.0f, 0.0f, -1.0f);
    Vec3 UpW = (dot(U, UpWorldPos) > dot(U, UpWorldNeg)) ? UpWorldPos : UpWorldNeg;

    // project world up onto plane perpendicular to F
    Vec3 UpProj = sub(UpW, mul(F, dot(UpW, F)));
    if (len(UpProj) < 1e-3f)
        UpProj = U;
    else
        UpProj = norm(UpProj);

    // blend UP toward projected world up
    U = norm(add(
        mul(U, 1.0f - g_cam.horizonLock),
        mul(UpProj, g_cam.horizonLock)
    ));

    // rebuild orthonormal basis
    R = norm(cross(U, F));
    U = norm(cross(F, R));

    // ------------------------------------------------------------
    // 2) ROLL AROUND FORWARD AXIS
    // ------------------------------------------------------------
    Vec3 R2 = rotate(R, F, rollRad);
    Vec3 U2 = rotate(U, F, rollRad);

    Vec3 F2 = norm(cross(R2, U2));
    Vec3 B2 = mul(F2, -1.0f);

    // ------------------------------------------------------------
    // 3) WRITE BASIS BACK
    // ------------------------------------------------------------
    V->m[0][0] = R2.x;
    V->m[1][0] = R2.y;
    V->m[2][0] = R2.z;
    V->m[0][1] = U2.x;
    V->m[1][1] = U2.y;
    V->m[2][1] = U2.z;
    V->m[0][2] = B2.x;
    V->m[1][2] = B2.y;
    V->m[2][2] = B2.z;

    // ------------------------------------------------------------
    // 4) KEEP CAMERA POSITION
    // ------------------------------------------------------------
    V->m[3][0] = -dot(R2, camPos);
    V->m[3][1] = -dot(U2, camPos);
    V->m[3][2] = -dot(B2, camPos);
}

static float gPrevYaw = 0.0f;

float ComputeYawDelta(const Mat4* V)
{
    // Forward = -B
    Vec3 B = v3(V->m[0][2], V->m[1][2], V->m[2][2]);
    Vec3 Fcam = norm(mul(B, -1.0f));
    Vec3 Fvel = g_cam.velDirFilt; // already XZ
    Fvel.y = 0.0f;
    if (len(Fvel) > 1e-3f) Fvel = norm(Fvel);
    else Fvel = Fcam;

    Vec3 Fax = norm(add(mul(Fcam, 0.7f), mul(Fvel, 0.3f)));


    float yaw = atan2f(Fax.x, Fax.z);
    float dyaw = yaw - gPrevYaw;
    gPrevYaw = yaw;

    // unwrap
    if (dyaw > PI) dyaw -= 2.0f * PI;
    if (dyaw < -PI) dyaw += 2.0f * PI;

    return dyaw;
}

static float gCamRoll = 0.0f;
static float gCamRollVel = 0.0f;

void UpdateCameraRoll(float yawDelta, float speed, float dt)
{
    const float rollStrength = 2.2f; // UG2-ish
    const float rollMax = 0.35f; // ~20 degrees
    const float stiffness = 8.0f; // response
    const float damping = 2.0f;

    float targetRoll = clampf(yawDelta * speed * rollStrength,
                              -rollMax, rollMax);

    // critically damped spring
    float accel = (targetRoll - gCamRoll) * stiffness
        - gCamRollVel * damping;

    gCamRollVel += accel * dt;
    gCamRoll += gCamRollVel * dt;
}

void __cdecl hkCreateLookAtMatrix(Mat4* mat, Vec3* eye, Vec3* center, Vec3* up)
{
    // Let MW build the camera fully (director, blends, etc.)
    oCreateLookAt(mat, eye, center, up);

    bool applyRoll = (InterlockedExchange(&gApplyUG2Flag, 0) != 0);

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
    // 1) CAMERA-DERIVED VELOCITY (MW: XZ plane)
    // ------------------------------------------------------------
    Vec3 fromNow = *eye;

    // HARD CUT DETECTION (must be before prevFrom overwrite)
    Vec3 delta = sub(fromNow, g_cam.prevFrom);
    float cutDist = len(delta);

    // Tune threshold to your units (start with 30..80)
    if (cutDist > 60.0f)
    {
        // Reset dynamic state on cuts (prevents snaps)
        g_cam.velDirFilt = v3(0.0f, 0.0f, 1.0f);
        g_cam.prevVelDirFilt = g_cam.velDirFilt;
        g_cam.yawRateFilt = 0.0f;
        g_cam.rollBias = 0.0f;

        // Important: resync position and exit this frame (no dt-based spike)
        g_cam.prevFrom = fromNow;
        return;
    }

    // Use delta as velocity step
    Vec3 vel = delta;
    g_cam.prevFrom = fromNow;

    float speed = len(vel) / dt;

    Vec3 vdir = vel;
    vdir.y = 0.0f;

    float vdirLen = len(vdir);

    float dirResp = 1.0f - expf(-10.0f * dt);
    float speedResp = 1.0f - expf(-6.0f * dt);

    if (vdirLen > 1e-3f && speed > 1e-2f)
    {
        vdir = mul(vdir, 1.0f / vdirLen);

        g_cam.velDirFilt = norm(add(mul(g_cam.velDirFilt, 1.0f - dirResp),
                                    mul(vdir, dirResp)));

        g_cam.speedFilt += (speed - g_cam.speedFilt) * speedResp;

        Vec3 prevF = g_cam.prevVelDirFilt;
        Vec3 curF = g_cam.velDirFilt;

        if (len(prevF) < 0.5f)
            prevF = curF;

        float yawS = (prevF.x * curF.z - prevF.z * curF.x);
        float yawRateRaw = yawS / dt;

        g_cam.prevVelDirFilt = curF;

        float yawResp = 1.0f - expf(-8.0f * dt);
        g_cam.yawRateFilt += (yawRateRaw - g_cam.yawRateFilt) * yawResp;
    }
    else
    {
        g_cam.yawRateFilt += (0.0f - g_cam.yawRateFilt) * speedResp;
        g_cam.speedFilt += (0.0f - g_cam.speedFilt) * speedResp;
        g_cam.prevVelDirFilt = g_cam.velDirFilt;
    }

    // kill tiny oscillations (UG2 deadzone)
    const float yawDead = 0.015f; // rad/s-ish, tune 0.01..0.03
    if (fabsf(g_cam.yawRateFilt) < yawDead)
        g_cam.yawRateFilt = 0.0f;

    // ------------------------------------------------------------
    // 3) UG2-ACCURATE ROLL INTENT
    // ------------------------------------------------------------

    // Lateral-G proxy
    float latG = fabsf(g_cam.yawRateFilt) * g_cam.speedFilt;

    // MW camera-derived scale usually needs lower knee:
    const float G0 = 2.5f;
    const float G1 = 8.0f;

    float g01 = saturate((latG - G0) / (G1 - G0));
    g01 = g01 * g01 * (3.0f - 2.0f * g01);

    // roll target
    float rollTarget = signf(-g_cam.yawRateFilt) * g01 * DEG2RAD(8.5f);

    // filter command (UG2 smoothness)
    float cmdResp = 1.0f - expf(-12.0f * dt);
    g_cam.rollTargetFilt += (rollTarget - g_cam.rollTargetFilt) * cmdResp;
    rollTarget = g_cam.rollTargetFilt;

    // rate limit command
    const float maxRateIn  = DEG2RAD(90.0f);
    const float maxRateOut = DEG2RAD(60.0f);
    float maxRate = (fabsf(rollTarget) > fabsf(g_cam.rollBias)) ? maxRateIn : maxRateOut;

    float maxStep = maxRate * dt;
    float diff = rollTarget - g_cam.rollBias;
    diff = clampf(diff, -maxStep, +maxStep);
    rollTarget = g_cam.rollBias + diff;

    // asymmetric response (you already had)
    float kIn = 10.0f;
    float kOut = 4.0f;
    float k = (fabsf(rollTarget) > fabsf(g_cam.rollBias)) ? kIn : kOut;

    if (!applyRoll)
        g_cam.rollBias += (0.0f - g_cam.rollBias) * (1.0f - expf(-3.0f * dt));
    else
        g_cam.rollBias += (rollTarget - g_cam.rollBias) * (1.0f - expf(-k * dt));

    // horizon lock factor
    float yaw01 = saturate(fabsf(g_cam.yawRateFilt) / 0.12f);
    float speed01 = saturate(g_cam.speedFilt / 38.0f);

    float targetLock = 0.95f - 0.35f * yaw01 + 0.10f * speed01;
    targetLock = clampf(targetLock, 0.55f, 0.98f);
    g_cam.horizonLock += (targetLock - g_cam.horizonLock) * (1.0f - expf(-3.5f * dt));

    // ------------------------------------------------------------
    // 4) APPLY ROLL (KEEP POSITION)
    // ------------------------------------------------------------
    if (applyRoll)
    {
        ApplyRollKeepPos(mat, g_cam.rollBias);
    }
}

// ==========================================================
// Hook
// ==========================================================

static bool InstallHooks()
{
    if (MH_Initialize() != MH_OK)
        return false;

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
