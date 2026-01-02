#include "Log.h"

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

static inline Vec3 v3(float x,float y,float z){ return {x,y,z}; }

static inline Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline Vec3 mul(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline Vec3 neg(Vec3 a){ return {-a.x,-a.y,-a.z}; }
static inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline bool is_finite(float f) { return std::isfinite(f) != 0; }
static inline float absf(float x) { return x < 0 ? -x : x; }

// Wrap angle to [-pi, pi]
static inline float WrapPi(float a)
{
    while (a >  3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

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

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline float saturate(float x)
{
    return clampf(x, 0.0f, 1.0f);
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

static inline Vec3 Normalize(const Vec3& v)
{
    float len = std::sqrt(dot(v, v));
    if (len < 1e-6f) return {0,0,0};
    return { v.x/len, v.y/len, v.z/len };
}

static void Normalize3(float& x, float& y, float& z)
{
    float len2 = x*x + y*y + z*z;
    if (len2 <= 1e-12f) { x = 1.0f; y = 0.0f; z = 0.0f; return; }
    float inv = 1.0f / sqrtf(len2);
    x *= inv; y *= inv; z *= inv;
}

static void Cross3(
    float ax, float ay, float az,
    float bx, float by, float bz,
    float& rx, float& ry, float& rz)
{
    rx = ay*bz - az*by;
    ry = az*bx - ax*bz;
    rz = ax*by - ay*bx;
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

static bool LooksOrthonormal_Rows(const Mat4& M)
{
    Vec3 r{M.m[0][0],M.m[0][1],M.m[0][2]};
    Vec3 u{M.m[1][0],M.m[1][1],M.m[1][2]};
    Vec3 f{M.m[2][0],M.m[2][1],M.m[2][2]};
    return absf(len(r)-1.f)<0.2f && absf(len(u)-1.f)<0.2f && absf(len(f)-1.f)<0.2f &&
           absf(dot(r,u))<0.2f && absf(dot(r,f))<0.2f && absf(dot(u,f))<0.2f;
}

static bool LooksOrthonormal_Cols(const Mat4& M)
{
    Vec3 r{M.m[0][0],M.m[1][0],M.m[2][0]};
    Vec3 u{M.m[0][1],M.m[1][1],M.m[2][1]};
    Vec3 f{M.m[0][2],M.m[1][2],M.m[2][2]};
    return absf(len(r)-1.f)<0.2f && absf(len(u)-1.f)<0.2f && absf(len(f)-1.f)<0.2f &&
           absf(dot(r,u))<0.2f && absf(dot(r,f))<0.2f && absf(dot(u,f))<0.2f;
}

static inline void Roll3x3_Columns(Mat4& M, float roll)
{
    Vec3 r = { M.m[0][0], M.m[1][0], M.m[2][0] };
    Vec3 u = { M.m[0][1], M.m[1][1], M.m[2][1] };

    float s = sinf(roll), c = cosf(roll);

    Vec3 r2 = add(mul(r, c), mul(u, s));
    Vec3 u2 = add(mul(u, c), mul(r, -s));

    M.m[0][0]=r2.x; M.m[1][0]=r2.y; M.m[2][0]=r2.z;
    M.m[0][1]=u2.x; M.m[1][1]=u2.y; M.m[2][1]=u2.z;
}

static inline void Roll3x3_Rows(Mat4& M, float roll)
{
    // rotate row0/row1 around forward
    Vec3 r = { M.m[0][0], M.m[0][1], M.m[0][2] };
    Vec3 u = { M.m[1][0], M.m[1][1], M.m[1][2] };

    float s = sinf(roll), c = cosf(roll);

    Vec3 r2 = add(mul(r, c), mul(u, s));
    Vec3 u2 = add(mul(u, c), mul(r, -s));

    M.m[0][0]=r2.x; M.m[0][1]=r2.y; M.m[0][2]=r2.z;
    M.m[1][0]=u2.x; M.m[1][1]=u2.y; M.m[1][2]=u2.z;
}

static inline float OrthoError_Columns(const Mat4& M)
{
    Vec3 r = { M.m[0][0], M.m[1][0], M.m[2][0] };
    Vec3 u = { M.m[0][1], M.m[1][1], M.m[2][1] };
    Vec3 f = { M.m[0][2], M.m[1][2], M.m[2][2] };
    return fabsf(dot(r,u)) + fabsf(dot(r,f)) + fabsf(dot(u,f)) +
           fabsf(len(r)-1.0f) + fabsf(len(u)-1.0f) + fabsf(len(f)-1.0f);
}

static inline float OrthoError_Rows(const Mat4& M)
{
    Vec3 r = { M.m[0][0], M.m[0][1], M.m[0][2] };
    Vec3 u = { M.m[1][0], M.m[1][1], M.m[1][2] };
    Vec3 f = { M.m[2][0], M.m[2][1], M.m[2][2] };
    return fabsf(dot(r,u)) + fabsf(dot(r,f)) + fabsf(dot(u,f)) +
           fabsf(len(r)-1.0f) + fabsf(len(u)-1.0f) + fabsf(len(f)-1.0f);
}

struct Basis { Vec3 r,u,f; }; // orthonormal

static inline Vec3 getT_row3(const Mat4& M) { return { M.m[3][0], M.m[3][1], M.m[3][2] }; }
static inline void setT_row3(Mat4& M, const Vec3& t){ M.m[3][0]=t.x; M.m[3][1]=t.y; M.m[3][2]=t.z; }

static Basis BasisRows(const Mat4& M)
{
    // rows 0..2 are basis
    Vec3 r{M.m[0][0],M.m[0][1],M.m[0][2]};
    Vec3 u{M.m[1][0],M.m[1][1],M.m[1][2]};
    Vec3 f{M.m[2][0],M.m[2][1],M.m[2][2]};
    return {r,u,f};
}
static Basis BasisCols(const Mat4& M)
{
    // cols 0..2 are basis
    Vec3 r{M.m[0][0],M.m[1][0],M.m[2][0]};
    Vec3 u{M.m[0][1],M.m[1][1],M.m[2][1]};
    Vec3 f{M.m[0][2],M.m[1][2],M.m[2][2]};
    return {r,u,f};
}

static bool OrthoOK(const Basis& b)
{
    return fabsf(len(b.r)-1.f)<0.2f && fabsf(len(b.u)-1.f)<0.2f && fabsf(len(b.f)-1.f)<0.2f &&
           fabsf(dot(b.r,b.u))<0.2f && fabsf(dot(b.r,b.f))<0.2f && fabsf(dot(b.u,b.f))<0.2f;
}

static void WriteRows(Mat4& M, const Basis& b)
{
    M.m[0][0]=b.r.x; M.m[0][1]=b.r.y; M.m[0][2]=b.r.z;
    M.m[1][0]=b.u.x; M.m[1][1]=b.u.y; M.m[1][2]=b.u.z;
    M.m[2][0]=b.f.x; M.m[2][1]=b.f.y; M.m[2][2]=b.f.z;
}
static void WriteCols(Mat4& M, const Basis& b)
{
    M.m[0][0]=b.r.x; M.m[1][0]=b.r.y; M.m[2][0]=b.r.z;
    M.m[0][1]=b.u.x; M.m[1][1]=b.u.y; M.m[2][1]=b.u.z;
    M.m[0][2]=b.f.x; M.m[1][2]=b.f.y; M.m[2][2]=b.f.z;
}

static inline float orthoScore(Vec3 r, Vec3 u, Vec3 f)
{
    // lower is better
    float lr = fabsf(len(r) - 1.0f);
    float lu = fabsf(len(u) - 1.0f);
    float lf = fabsf(len(f) - 1.0f);
    float o  = fabsf(dot(r,u)) + fabsf(dot(r,f)) + fabsf(dot(u,f));
    return lr + lu + lf + o;
}

// For a standard view matrix: t = (-dot(r,camPos), -dot(u,camPos), -dot(f,camPos))
static Vec3 ExtractCamPos_View(const Basis& b, const Vec3& t)
{
    // camPos = -(t.x*r + t.y*u + t.z*f)
    Vec3 p = mul(b.r, -t.x);
    p = add(p, mul(b.u, -t.y));
    p = add(p, mul(b.f, -t.z));
    return p;
}

static Vec3 BuildT_View(const Basis& b, const Vec3& camPos)
{
    return { -dot(b.r, camPos), -dot(b.u, camPos), -dot(b.f, camPos) };
}

static void RollAroundForward(Basis& b, float roll)
{
    if (fabsf(roll) < 1e-6f) return;

    float s = sinf(roll), c = cosf(roll);
    // rotate r/u in their plane around f
    Vec3 r2 = add(mul(b.r, c), mul(b.u, s));
    Vec3 u2 = add(mul(b.u, c), mul(b.r, -s));
    b.r = r2;
    b.u = u2;
}

static inline bool IsFiniteFloat(float x)
{
    return (x == x) && (x > -3.4e38f) && (x < 3.4e38f); // simple NaN/Inf-ish filter
}