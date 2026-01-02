#pragma once
#include "Carbon.h"
#include "Math.h"
#include "TimeManager.h"
#include "minhook/include/MinHook.h"

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
// Hook
// ==========================================================

static bool InstallHooks()
{
    if (MH_Initialize() != MH_OK)
        return false;
    
    // 1) Hook wrapper (authoritative camera update)
    MH_CreateHook(
        (LPVOID)Game::CameraWrapperAddr,
        &hkCameraWrapper_Naked,
        &oCameraWrapper
    );

    // 2) Hook CreateLookAt builder
    MH_CreateHook(
        (LPVOID)Game::CreateLookAtAddr,
        &hkCreateLookAt,
        (LPVOID*)&oCreateLookAt
    );

    // 3) Optional: disable tilts
    if (Game::DisableTiltsAddr)
    {
        static float DisableTilts = -1000.0f;
        *(float**)Game::DisableTiltsAddr = &DisableTilts;
    }
    
    MH_EnableHook(MH_ALL_HOOKS);

    return true;
}
