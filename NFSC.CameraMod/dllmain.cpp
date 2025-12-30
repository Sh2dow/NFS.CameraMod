#include <algorithm>
#include <windows.h>
#include <cstdint>
#include <cmath>
#include "Game.h"
#include "Log.h"
#include "minhook/include/MinHook.h"

// ==========================================================
// MATH
// ==========================================================

struct Vec3
{
    float x, y, z;
};

struct bMatrix4
{
    float m[4][4];
};

// row-major: row0=Right, row1=Forward, row2=Up, row3=Pos
static inline Vec3 GetRow(const bMatrix4& M, int r) { return {M.m[r][0], M.m[r][1], M.m[r][2]}; }

static inline void SetRow(bMatrix4& M, int r, Vec3 v)
{
    M.m[r][0] = v.x;
    M.m[r][1] = v.y;
    M.m[r][2] = v.z;
}

static inline Vec3 GetCol(const bMatrix4& M, int c) { return {M.m[0][c], M.m[1][c], M.m[2][c]}; }

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

static inline Vec3 pickHemisphere(Vec3 v, Vec3 ref)
{
    v = norm(v);
    if (dot(v, ref) < 0.0f) v = mul(v, -1.0f);
    return v;
}

static inline float signf(float v)
{
    return (v < 0.0f) ? -1.0f : 1.0f;
}

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline float clampDelta(float v, float maxDelta)
{
    if (v > maxDelta) return maxDelta;
    if (v < -maxDelta) return -maxDelta;
    return v;
}

static inline Vec3 clampAngularStep(Vec3 prev, Vec3 cur, float maxAngleRad)
{
    prev = norm(prev);
    cur = norm(cur);

    float d = clampf(dot(prev, cur), -1.0f, 1.0f);
    float ang = acosf(d);

    if (ang <= maxAngleRad)
        return cur;

    float t = maxAngleRad / ang;
    return norm(add(mul(prev, 1.0f - t), mul(cur, t)));
}

// ==========================================================
// GLOBAL STRUCTS
// ==========================================================

static void* g_CamAnchor_Tramp = nullptr; // trampoline to original bytes + back jump
static uint8_t g_CamAnchor_Orig[8] = {};
static size_t g_CamAnchor_PatchLen = 0;

static void* oCameraWrapper = nullptr; // trampoline
static volatile LONG gCamFrameStamp = 0;
static void* gMainAnchorThis = nullptr; // anchor used by main gameplay camera for current frame
static LONG gMainAnchorStamp = 0; // stamp when gMainAnchorThis was captured

struct CamState
{
    bool inited;
    // --- frame identity ---
    LONG lastStamp;
    LONG lastAppliedStamp;

    bool layoutKnown;
    bool colMajor;

    // --- motion state ---
    Vec3 prevForward;
    Vec3 prevFrom;
    Vec3 prevGravUp;
    Vec3 velDirFilt;
    Vec3 prevVelDirFilt;

    float speedFilt;
    float yawRateFilt;

    float yawBias;
    float rollBias;
    float gNorm;
    float horizonLock;

    float cachedYaw;
    float cachedRoll;
    float cachedLateral;
    float lateralFilt;
} g_cam;

struct CameraAnchor
{
    /* 0x00 */
    char pad_00[0x20];

    /* 0x20 */
    Vec3 position; // copied from input vector (edi+30h)
    // this is the anchor target

    /* 0x2C */
    float pad_2C;

    /* 0x30 */
    bMatrix4 matrix; // CopyMatrix(esi+30h, edi)
    // THIS is the final camera matrix

    /* 0x70 */
    Vec3 inertiaOffset; // ZEROED every frame
    // [esi+70h], [74h], [78h]

    /* 0x7C */
    float pad_7C;

    /* 0x80 */
    Vec3 someVector; // loaded from arg_8
    // probably forward / velocity

    /* 0x8C */
    float pad_8C;

    /* 0x90 */
    float distance; // camera distance (smoothed)

    /* 0x94 */
    char pad_94[0x48];

    /* 0xDC */
    float resetTimer; // accumulates dt
    /* 0xE0 */
    char pad_E0[0x0B];

    /* 0xEB */
    uint8_t resetFlag; // if set → inertia cleared
};

static void ResetCamState(const Vec3& camPos,
                          const Vec3& target,
                          const bMatrix4& m)
{
    g_cam.inited = true;

    g_cam.lastStamp = 0;
    g_cam.lastAppliedStamp = 0;

    g_cam.layoutKnown = false;
    g_cam.colMajor = false;

    // ---- CORRECT forward init ----
    Vec3 lookDir = norm(sub(target, camPos));

    // extract forward candidates
    Vec3 fRow = norm(GetRow(m, 1));
    Vec3 fCol = norm(GetCol(m, 1));

    // choose one that matches real look direction
    Vec3 f = (fabsf(dot(fCol, lookDir)) > fabsf(dot(fRow, lookDir))) ? fCol : fRow;
    f = pickHemisphere(f, lookDir);

    g_cam.prevForward = f;

    g_cam.prevFrom = camPos;
    g_cam.prevGravUp = Vec3{0, 0, 1};

    g_cam.velDirFilt = {1, 0, 0};
    g_cam.prevVelDirFilt = {1, 0, 0};

    g_cam.speedFilt = 0.0f;
    g_cam.yawRateFilt = 0.0f;

    g_cam.yawBias = 0.0f;
    g_cam.rollBias = 0.0f;
    g_cam.gNorm = 0.0f;
    g_cam.horizonLock = 1.0f;

    g_cam.cachedYaw = 0.0f;
    g_cam.cachedRoll = 0.0f;
    g_cam.cachedLateral = 0.0f;
    g_cam.lateralFilt = 0.0f;
}

// update+apply happen here (ONCE PER ANCHOR UPDATE)
static void __cdecl OnCameraAnchor_AfterCopyMatrix(void* anchorThis, float dt)
{
    if (!anchorThis) return;
    if (!(dt > 0.0f)) return;

    // ---------------------------------------------------------
    // MAIN-CAMERA ONLY GATE (prevents mirrors/reflections)
    // We only allow ONE anchor per wrapper tick to be modified.
    // ---------------------------------------------------------
    LONG stamp = gCamFrameStamp;
    if (stamp == 0) return;

    if (gMainAnchorStamp != stamp)
    {
        // first anchor update observed this frame -> treat as main
        gMainAnchorStamp = stamp;
        gMainAnchorThis = anchorThis;
    }
    else
    {
        // same frame: only modify the same anchor; others are mirrors/secondary cameras
        if (anchorThis != gMainAnchorThis)
            return;
    }

    dt = clampf(dt, 1.0f / 240.0f, 1.0f / 30.0f);


    dt = clampf(dt, 1.0f / 240.0f, 1.0f / 30.0f);

    auto* M = (bMatrix4*)((uint8_t*)anchorThis + 0x30); // matrix copied by CopyMatrix
    bMatrix4& m = *M;

    // ---- physical camera position from matrix ----
    Vec3 pos = {m.m[3][0], m.m[3][1], m.m[3][2]};

    // ---- anchor target (what the camera should look at) ----
    Vec3 target = ((CameraAnchor*)anchorThis)->position;

    // real look direction (target - camera)
    Vec3 lookDir = sub(target, pos);
    float lookLen = len(lookDir);
    if (lookLen > 1e-4f) lookDir = mul(lookDir, 1.0f / lookLen);
    else lookDir = Vec3{1, 0, 0}; // fallback

    const Vec3 worldUp = {0, 0, 1};

    // ---- UG2 physics update based on this pos (your existing code, adapted) ----
    if (!g_cam.inited)
    {
        ResetCamState(pos, target, m);
        return;
    }

    // velocity
    Vec3 vel = sub(pos, g_cam.prevFrom);
    g_cam.prevFrom = pos;

    float speedU = len(vel) / dt;

    Vec3 vdir = vel;
    vdir.z = 0.0f; // Z-up flatten

    if (len(vdir) > 1e-3f && speedU > 1e-2f)
    {
        vdir = norm(vdir);

        float a = 1.0f - expf(-6.0f * dt);

        g_cam.velDirFilt = norm(add(mul(g_cam.velDirFilt, 1.0f - a), mul(vdir, a)));
        g_cam.speedFilt += (speedU - g_cam.speedFilt) * a;

        Vec3 p = g_cam.prevVelDirFilt;
        Vec3 c = g_cam.velDirFilt;
        if (len(p) < 0.5f) p = c;

        float yawS = (p.x * c.y - p.y * c.x);
        float yawRateRaw = yawS / dt;

        g_cam.prevVelDirFilt = c;

        float ay = 1.0f - expf(-8.0f * dt);
        g_cam.yawRateFilt += (yawRateRaw - g_cam.yawRateFilt) * ay;
    }
    else
    {
        float a = 1.0f - expf(-6.0f * dt);
        g_cam.yawRateFilt += (0.0f - g_cam.yawRateFilt) * a;
        g_cam.speedFilt += (0.0f - g_cam.speedFilt) * a;
    }

    // horizon lock + load
    float yaw01 = clampf(fabsf(g_cam.yawRateFilt) / 0.6f, 0.0f, 1.0f);
    float speed01 = clampf(g_cam.speedFilt / 45.0f, 0.0f, 1.0f);

    float lockTarget = clampf(1.0f - (0.50f * yaw01 + 0.25f * speed01), 0.25f, 1.0f);
    g_cam.horizonLock += (lockTarget - g_cam.horizonLock) * (1.0f - expf(-2.5f * dt));

    float load = fabsf(g_cam.yawRateFilt) * g_cam.speedFilt;
    float gTarget = clampf(load * 0.00006f, 0.0f, 1.0f);
    g_cam.gNorm += (gTarget - g_cam.gNorm) * (1.0f - expf(-3.0f * dt));

    // yaw/roll/lateral (keep your tuned numbers)
    float yawTarget = clampf(g_cam.yawRateFilt * 0.10f, -0.03f, 0.03f);
    g_cam.yawBias += (yawTarget - g_cam.yawBias) * (1.0f - expf(-10.0f * dt));

    float rollTarget =
        signf(g_cam.yawRateFilt) *
        (0.35f * g_cam.gNorm + 0.65f * powf(g_cam.gNorm, 1.4f)) *
        0.45f;

    float speedFade = clampf(g_cam.speedFilt / 18.0f, 0.0f, 1.0f);
    rollTarget *= speedFade;

    g_cam.rollBias += (rollTarget - g_cam.rollBias) * (1.0f - expf(-6.0f * dt));

    float latTarget = signf(g_cam.yawRateFilt) * g_cam.gNorm * 900.0f;
    g_cam.lateralFilt += (latTarget - g_cam.lateralFilt) * (1.0f - expf(-4.0f * dt));

    // ---- APPLY to matrix (basis extraction must be robust) ----
    // Candidates: row-forward vs col-forward (Carbon can be either depending on camera path)
    Vec3 fRow = norm(GetRow(m, 1));
    Vec3 fCol = norm(GetCol(m, 1));

    // Decide layout ONLY ONCE (first meaningful frame after init)
    if (!g_cam.layoutKnown)
    {
        // Use "more horizontal" as a hint + continuity if available
        float hRow = 1.0f - fabsf(fRow.z);
        float hCol = 1.0f - fabsf(fCol.z);

        // If prevForward is valid, use it; otherwise use horizontality
        float dRow = fabsf(dot(fRow, g_cam.prevForward));
        float dCol = fabsf(dot(fCol, g_cam.prevForward));

        // pick the winner with a margin to avoid ambiguous toggles
        if (fabsf(dCol - dRow) > 0.10f)
            g_cam.colMajor = (dCol > dRow);
        else
            g_cam.colMajor = (hCol > hRow);

        g_cam.layoutKnown = true;
    }

    bool colMajor = g_cam.colMajor;
    Vec3 f = colMajor ? fCol : fRow;

    // Continuity: never allow forward to flip 180°
    f = pickHemisphere(f, g_cam.prevForward);
    g_cam.prevForward = f;

    Vec3 r = cross(worldUp, f);
    if (len(r) < 1e-3f)
        r = colMajor ? GetCol(m, 0) : GetRow(m, 0);
    r = norm(r);

    // IMPORTANT: u = f × r
    Vec3 u = norm(cross(f, r));

    // yaw around worldUp (rotate r/f)
    float yaw = g_cam.yawBias;
    if (fabsf(yaw) > 1e-6f)
    {
        float s = sinf(yaw), c = cosf(yaw);
        Vec3 r2 = add(mul(r, c), mul(f, s));
        Vec3 f2 = add(mul(f, c), mul(r, -s));
        r = norm(r2);
        f = norm(f2);
        u = norm(cross(r, f));
    }

    // lateral translate along right (matrix position row)
    float lateral = clampf(g_cam.lateralFilt, -900.0f, 900.0f);
    pos = add(pos, mul(r, lateral));

    // small lift
    float lift = g_cam.gNorm * speedFade * 0.35f;
    pos = add(pos, mul(worldUp, lift));

    // roll around forward (bank) — reduce by horizon lock
    float effRoll = g_cam.rollBias * (1.0f - g_cam.horizonLock);
    if (fabsf(effRoll) > 1e-6f)
    {
        u = rotate(u, f, effRoll);
        u = norm(sub(u, mul(f, dot(u, f))));
        r = norm(cross(u, f));
        f = norm(cross(r, u));
    }

    // write back matrix (match layout we detected)
    if (!colMajor)
    {
        SetRow(m, 0, r);
        SetRow(m, 1, f);
        SetRow(m, 2, u);
    }
    else
    {
        // set columns
        m.m[0][0] = r.x;
        m.m[1][0] = r.y;
        m.m[2][0] = r.z;
        m.m[0][1] = f.x;
        m.m[1][1] = f.y;
        m.m[2][1] = f.z;
        m.m[0][2] = u.x;
        m.m[1][2] = u.y;
        m.m[2][2] = u.z;
    }

    m.m[3][0] = pos.x;
    m.m[3][1] = pos.y;
    m.m[3][2] = pos.z;
}

// ==========================================================
// WRAPPERS + HOOKS (NAKED, SAFE)
// ==========================================================

__declspec(naked) void hkCameraWrapper_Naked()
{
    __asm {
        pushfd
        pushad
        }

    gCamFrameStamp++; // frame boundary ONLY

    __asm {
        popad
        popfd
        jmp oCameraWrapper
        }
}

__declspec(naked) void hkCameraAnchor_AfterCopyMatrix()
{
    __asm {
        // we are at 0x0047968C, ESI = this, EBP is valid
        pushfd
        pushad

        // args for __cdecl: (void* this, float dt)
        mov eax, [ebp+8] // dt (arg_0)
        push eax
        push esi
        call OnCameraAnchor_AfterCopyMatrix
        add esp, 8

        popad
        popfd

        // jump to trampoline (executes stolen bytes + returns to original flow)
        jmp dword ptr [g_CamAnchor_Tramp]
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

// ==========================================================
// CameraAnchor_Update hook
// ==========================================================
static void WriteRelJmp(uint8_t* at, void* to)
{
    DWORD old{};
    VirtualProtect(at, 16, PAGE_EXECUTE_READWRITE, &old);

    intptr_t rel = (intptr_t)to - ((intptr_t)at + 5);
    at[0] = 0xE9;
    *(int32_t*)(at + 1) = (int32_t)rel;

    VirtualProtect(at, 16, old, &old);
    FlushInstructionCache(GetCurrentProcess(), at, 16);
}

static void* MakeTrampoline(uint8_t* src, size_t len)
{
    // allocate executable memory
    uint8_t* tramp = (uint8_t*)VirtualAlloc(nullptr, 64, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    if (!tramp) return nullptr;

    // copy stolen bytes
    memcpy(tramp, src, len);

    // jmp back to src+len
    WriteRelJmp(tramp + len, src + len);

    return tramp;
}

static bool HookMid(uintptr_t addr, void* hook, void** outTramp)
{
    // We will steal exactly 5 bytes at 0x0047968C:
    // xor ecx, ecx  (2 bytes)
    // mov [esi+68], ecx (3 bytes)
    // Total = 5 bytes (perfect for JMP)
    constexpr size_t kLen = 5;

    uint8_t* p = (uint8_t*)addr;

    // backup
    memcpy(g_CamAnchor_Orig, p, kLen);
    g_CamAnchor_PatchLen = kLen;

    // trampoline
    void* tramp = MakeTrampoline(p, kLen);
    if (!tramp) return false;

    // patch site -> hook
    WriteRelJmp(p, hook);

    *outTramp = tramp;
    return true;
}

// ==========================================================
// INSTALL
// ==========================================================

static bool InstallHooks()
{
    if (MH_Initialize() != MH_OK)
        return false;

    // wrapper stamp hook (keep yours)
    if (!HookWrapper(Game::CameraWrapperAddr))
        return false;

    // IMPORTANT: stop using CreateLookAt for camera mods when using Anchor hook
    // You can still hook CreateLookAt for logging, but DO NOT apply there.

    if (!HookMid(Game::CameraAnchor_Update, (void*)&hkCameraAnchor_AfterCopyMatrix, &g_CamAnchor_Tramp))
        return false;

    asi_log::Log("[UG2Cam] CameraAnchor mid-hook installed @ 0x%08X\n", (unsigned)Game::CameraAnchor_Update);
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
