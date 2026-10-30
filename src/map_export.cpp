#include "map_export.hpp"
#include "bsp.hpp"

#include <cctype>
#include <cstdlib>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kOnPlane = 0.2;
constexpr double kInsideSlop = 1.0;
constexpr int kMaxPatch = 16;

struct Vec3 {
    double x, y, z;
};

struct PlaneD {
    double n[3];
    double dist;
    double Dist(const Vec3& p) const
    {
        return n[0] * p.x + n[1] * p.y + n[2] * p.z - dist;
    }
};

struct FaceOut {
    Vec3 p[3];
    string material;
    double scaleS = 128, scaleT = 128;
    double shiftS = 0, shiftT = 0;
    double rotate = 0, skew = 0;
    double lmScaleS = 16384, lmScaleT = 16384;
    double lmShiftS = 0, lmShiftT = 0;
    double lmRotate = 0, lmSkew = 0;
};

struct BrushOut {
    unsigned contents = 0;
    bool portal = false;
    vector<FaceOut> faces;
};

struct MeshVertOut {
    Vec3 p;
    float st[2];
    float lm[2];
    float n[3];
    unsigned char color[4];
    int flag = 0;
};

struct MeshOut {
    unsigned contents = 0;
    string material;
    int width = 0;
    int height = 0;
    vector<MeshVertOut> verts;
};

struct ModelGeo {
    vector<BrushOut> brushes;
    vector<MeshOut> meshes;
};

struct Pair {
    string key;
    string value;
};

struct Ent {
    vector<Pair> pairs;
};

struct WeldVert {
    Vec3 p;
    float st[2];
    float lm[2];
    float n[3];
    unsigned char color[4];
};

struct Tri {
    int a, b, c;
};

struct Quad {
    int v[4];
    int tri[2];
};

struct SideGeom {
    int nameId = 0;
    int drawNameId = 0;
    unsigned surfaceFlags = 0;
    PlaneD plane;
    double mins[3];
    double maxs[3];
    vector<Vec3> winding;
    int faceIndex = -1;
    int brushIndex = -1;
};

bool SideIsPortal(unsigned surfaceFlags)
{
    return (surfaceFlags & kSurfPortal) != 0;
}

bool SideIsVisibleSolid(unsigned surfaceFlags)
{
    return (surfaceFlags & kSurfPortal) == 0 && (surfaceFlags & kSurfNodraw) == 0;
}

bool IsPortalMapMaterial(unsigned surfaceFlags, const string& name)
{
    if ((surfaceFlags & kSurfPortal) != 0)
        return true;
    return name == "portal" || name == "portal_nodraw";
}

Vec3 Add(const Vec3& a, const Vec3& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
Vec3 Sub(const Vec3& a, const Vec3& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
Vec3 Mul(const Vec3& a, double s) { return { a.x * s, a.y * s, a.z * s }; }
double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
double Len(const Vec3& a) { return sqrt(Dot(a, a)); }

Vec3 FromVert(const BspDrawVert& v, const Vec3& origin)
{
    return { v.xyz[0] + origin.x, v.xyz[1] + origin.y, v.xyz[2] + origin.z };
}

double SnapNear(double v)
{
    double rounded = round(v);
    if (fabs(v - rounded) < 0.001)
        return rounded;
    return v;
}

void WriteNum(FILE* f, double v)
{
    if (!isfinite(v))
        v = 0;
    double rounded = round(v);
    if (fabs(v - rounded) < 1e-4) {
        fprintf(f, "%.0f", rounded);
        return;
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "%.6f", v);
    char* end = buf + strlen(buf);
    while (end > buf && end[-1] == '0')
        --end;
    if (end > buf && end[-1] == '.')
        --end;
    *end = 0;
    fputs(buf, f);
}

bool IsWeaponClipFlags(unsigned flags)
{
    const unsigned clips = (unsigned)kContentsClipShot | (unsigned)kContentsMissileClip;
    return (flags & clips) == clips && (flags & (unsigned)kContentsSolid) == 0;
}

const char* ContentsToken(unsigned flags)
{
    if (IsWeaponClipFlags(flags))
        return "weaponClip";
    if (flags & (unsigned)kContentsNonColliding)
        return "nonColliding";
    if (flags & (unsigned)kContentsDetail)
        return "detail";
    return nullptr;
}

vector<Vec3> BaseWinding(const PlaneD& plane)
{
    double ax = fabs(plane.n[0]);
    double ay = fabs(plane.n[1]);
    double az = fabs(plane.n[2]);
    Vec3 up;
    if (ax <= ay && ax <= az)
        up = { 1, 0, 0 };
    else if (ay <= ax && ay <= az)
        up = { 0, 1, 0 };
    else
        up = { 0, 0, 1 };

    Vec3 n = { plane.n[0], plane.n[1], plane.n[2] };
    Vec3 right = Cross(up, n);
    double rightLen = Len(right);
    if (rightLen < 1e-12)
        return {};
    right = Mul(right, 1.0 / rightLen);
    up = Cross(n, right);

    Vec3 org = Mul(n, plane.dist);
    const double bound = 8.0e6;
    vector<Vec3> w(4);
    w[0] = Add(org, Add(Mul(up, bound), Mul(right, -bound)));
    w[1] = Add(org, Add(Mul(up, bound), Mul(right, bound)));
    w[2] = Add(org, Add(Mul(up, -bound), Mul(right, bound)));
    w[3] = Add(org, Add(Mul(up, -bound), Mul(right, -bound)));

    Vec3 facing = Cross(Sub(w[1], w[0]), Sub(w[2], w[0]));
    if (Dot(facing, n) < 0)
        reverse(w.begin(), w.end());
    return w;
}

vector<Vec3> ClipWinding(const vector<Vec3>& in, const PlaneD& plane)
{
    vector<Vec3> out;
    if (in.empty())
        return out;
    const double eps = 0.05;
    for (size_t i = 0; i < in.size(); ++i) {
        const Vec3& a = in[i];
        const Vec3& b = in[(i + 1) % in.size()];
        double da = plane.Dist(a);
        double db = plane.Dist(b);
        if (da <= eps)
            out.push_back(a);
        if ((da > eps && db < -eps) || (da < -eps && db > eps)) {
            double t = da / (da - db);
            out.push_back({ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t });
        }
    }
    return out;
}

bool BestTriangle(const vector<Vec3>& w, Vec3 out[3])
{
    double best = 0.01;
    bool found = false;
    if (w.size() < 3)
        return false;
    for (size_t i = 0; i < w.size(); ++i) {
        Vec3 a = w[i];
        Vec3 b = w[(i + 1) % w.size()];
        Vec3 c = w[(i + 2) % w.size()];
        double area = Len(Cross(Sub(b, a), Sub(c, a)));
        if (area > best) {
            best = area;
            out[0] = a;
            out[1] = b;
            out[2] = c;
            found = true;
        }
    }
    return found;
}

Vec3 PlaneFromPointsNormal(const Vec3& p0, const Vec3& p1, const Vec3& p2)
{
    return Cross(Sub(p2, p0), Sub(p1, p0));
}

bool FacePoints(const vector<Vec3>& winding, const PlaneD& plane, Vec3 out[3])
{
    if (!BestTriangle(winding, out))
        return false;
    Vec3 n = { plane.n[0], plane.n[1], plane.n[2] };
    if (Dot(PlaneFromPointsNormal(out[0], out[1], out[2]), n) < 0.0) {
        Vec3 mid = out[1];
        out[1] = out[2];
        out[2] = mid;
    }
    return Dot(PlaneFromPointsNormal(out[0], out[1], out[2]), n) > 0.0;
}

double RoundTripCoord(double v)
{
    if (!isfinite(v))
        return 0;
    double rounded = round(v);
    if (fabs(v - rounded) < 1e-4)
        return rounded;
    char buf[64];
    snprintf(buf, sizeof(buf), "%.6f", v);
    char* end = buf + strlen(buf);
    while (end > buf && end[-1] == '0')
        --end;
    if (end > buf && end[-1] == '.')
        --end;
    *end = 0;
    return strtod(buf, nullptr);
}

int RoundToInt(float value)
{
    volatile union {
        double d;
        int i[2];
    } u;
    u.d = (double)value + 9.313225746154785e-10 + 6755399441055744.0;
    return u.i[0];
}

float Rint(float value)
{
    return floorf(value + 0.5f);
}

void LoadFacePoint(const Vec3& p, float out[3])
{
    out[0] = (float)RoundTripCoord(p.x);
    out[1] = (float)RoundTripCoord(p.y);
    out[2] = (float)RoundTripCoord(p.z);
}

void SnapFloatPlane(float p[4])
{
    for (int i = 0; i < 3; ++i) {
        if (fabsf(p[i] - 1.f) < 1e-7f) {
            p[0] = p[1] = p[2] = 0.f;
            p[i] = 1.f;
            break;
        }
        if (fabsf(p[i] + 1.f) < 1e-7f) {
            p[0] = p[1] = p[2] = 0.f;
            p[i] = -1.f;
            break;
        }
    }
    float snapped = Rint(p[3]);
    if (fabsf(p[3] - snapped) < 0.001f)
        p[3] = snapped;
}

float FloatDot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void FloatCross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

double FloatDetTerm(float a, float b, float c, float d, float e)
{
    float ab = a * b;
    float cd = c * d;
    float sub = ab - cd;
    float prod = sub * e;
    return (double)prod;
}

bool PlaneFromFacePoints(const float p0[3], const float p1[3], const float p2[3], float out[4])
{
    float d1[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
    float d2[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
    float n[3];
    FloatCross(d2, d1, n);
    float lenSq = FloatDot(n, n);
    if (lenSq < 2.f) {
        if (lenSq == 0.f)
            return false;
        if (lenSq < FloatDot(d1, d1) * FloatDot(d2, d2) * 0.000001f) {
            float e1[3] = { p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2] };
            float e2[3] = { p0[0] - p1[0], p0[1] - p1[1], p0[2] - p1[2] };
            FloatCross(e2, e1, n);
            lenSq = FloatDot(n, n);
            if (lenSq == 0.f || lenSq < FloatDot(e1, e1) * FloatDot(e2, e2) * 0.000001f)
                return false;
        }
    }
    float len = sqrtf(lenSq);
    out[0] = n[0] / len;
    out[1] = n[1] / len;
    out[2] = n[2] / len;
    out[3] = FloatDot(p0, out);
    SnapFloatPlane(out);
    return true;
}

bool PickFaceFromWinding(const vector<Vec3>& winding, const PlaneD& plane, Vec3 out[3])
{
    const int n = (int)winding.size();
    if (n < 3)
        return false;
    float src[4] = { (float)plane.n[0], (float)plane.n[1], (float)plane.n[2], (float)plane.dist };
    double bestArea = 0;
    bool found = false;
    Vec3 best[3];
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            for (int k = j + 1; k < n; ++k) {
                Vec3 tri[3] = { winding[i], winding[j], winding[k] };
                Vec3 cr = Cross(Sub(tri[1], tri[0]), Sub(tri[2], tri[0]));
                double along = Dot(cr, Vec3{ plane.n[0], plane.n[1], plane.n[2] });
                if (along < 0) {
                    swap(tri[1], tri[2]);
                    along = -along;
                }
                if (along <= bestArea)
                    continue;
                float p0[3], p1[3], p2[3], recovered[4];
                LoadFacePoint(tri[0], p0);
                LoadFacePoint(tri[1], p1);
                LoadFacePoint(tri[2], p2);
                if (!PlaneFromFacePoints(p0, p1, p2, recovered))
                    continue;
                if (FloatDot(recovered, src) < 0.999f)
                    continue;
                float distTol = 0.01f;
                float scaled = fabsf(src[3]) * 0.00001f;
                if (scaled > distTol)
                    distTol = scaled;
                if (fabsf(recovered[3] - src[3]) > distTol)
                    continue;
                bestArea = along;
                best[0] = tri[0];
                best[1] = tri[1];
                best[2] = tri[2];
                found = true;
            }
        }
    }
    if (!found)
        return false;
    out[0] = best[0];
    out[1] = best[1];
    out[2] = best[2];
    return true;
}

bool SameFloatPlane(const float* a, const float* b)
{
    return fabsf(a[0] - b[0]) < 1e-7f && fabsf(a[1] - b[1]) < 1e-7f
        && fabsf(a[2] - b[2]) < 1e-7f && fabsf(a[3] - b[3]) < 0.001f;
}

bool IntersectFloatPlanes(const float* p0, const float* p1, const float* p2, float out[3])
{
    double det = FloatDetTerm(p1[1], p2[2], p2[1], p1[2], p0[0])
               + FloatDetTerm(p2[1], p0[2], p0[1], p2[2], p1[0])
               + FloatDetTerm(p0[1], p1[2], p1[1], p0[2], p2[0]);
    if (fabs(det) < 0.001)
        return false;
    double inv = 1.0 / det;
    out[0] = (float)((FloatDetTerm(p1[1], p2[2], p2[1], p1[2], p0[3])
                    + FloatDetTerm(p2[1], p0[2], p0[1], p2[2], p1[3])
                    + FloatDetTerm(p0[1], p1[2], p1[1], p0[2], p2[3])) * inv);
    out[1] = (float)((FloatDetTerm(p1[2], p2[0], p2[2], p1[0], p0[3])
                    + FloatDetTerm(p2[2], p0[0], p0[2], p2[0], p1[3])
                    + FloatDetTerm(p0[2], p1[0], p1[2], p0[0], p2[3])) * inv);
    out[2] = (float)((FloatDetTerm(p1[0], p2[1], p2[0], p1[1], p0[3])
                    + FloatDetTerm(p2[0], p0[1], p0[0], p2[1], p1[3])
                    + FloatDetTerm(p0[0], p1[1], p1[0], p0[1], p2[3])) * inv);
    return true;
}

void SnapFloatPoint(float point[3], const float* p0, const float* p1, const float* p2)
{
    float snapped[3];
    bool changed = false;
    for (int i = 0; i < 3; ++i) {
        float coord = (float)RoundToInt(point[i] / 0.25f) * 0.25f;
        if (fabsf(coord - point[i]) < 0.01f) {
            snapped[i] = coord;
            changed = true;
        } else {
            snapped[i] = point[i];
        }
    }
    if (!changed)
        return;
    const float* planes[3] = { p0, p1, p2 };
    float maxError = 0.f;
    float maxAllowed = 0.01f;
    for (int i = 0; i < 3; ++i) {
        float onSnap = fabsf(FloatDot(planes[i], snapped) - planes[i][3]);
        if (maxError < onSnap)
            maxError = onSnap;
        float onPoint = fabsf(FloatDot(planes[i], point) - planes[i][3]);
        if (maxAllowed < onPoint)
            maxAllowed = onPoint;
    }
    if (maxError < maxAllowed) {
        point[0] = snapped[0];
        point[1] = snapped[1];
        point[2] = snapped[2];
    }
}

struct PlaneHit {
    float slack = -1.f;
    int tri[3] = { -1, -1, -1 };
    int which = 0;
};

int BrushSolidPoints(const vector<FaceOut>& faces, float* worstSlack)
{
    int count = (int)faces.size();
    if (worstSlack)
        *worstSlack = -1.f;
    if (count < 4)
        return 0;
    vector<float> planes((size_t)count * 4);
    for (int i = 0; i < count; ++i) {
        float p0[3], p1[3], p2[3];
        LoadFacePoint(faces[i].p[0], p0);
        LoadFacePoint(faces[i].p[1], p1);
        LoadFacePoint(faces[i].p[2], p2);
        if (!PlaneFromFacePoints(p0, p1, p2, &planes[(size_t)i * 4]))
            return 0;
    }
    auto planeAt = [&](int i) -> const float* { return &planes[(size_t)i * 4]; };
    int points = 0;
    float slack = 1e30f;
    for (int i = 0; i < count - 2; ++i) {
        for (int j = i + 1; j < count - 1; ++j) {
            if (SameFloatPlane(planeAt(i), planeAt(j)))
                continue;
            for (int k = j + 1; k < count; ++k) {
                if (SameFloatPlane(planeAt(i), planeAt(k)) || SameFloatPlane(planeAt(j), planeAt(k)))
                    continue;
                float point[3];
                if (!IntersectFloatPlanes(planeAt(i), planeAt(j), planeAt(k), point))
                    continue;
                SnapFloatPoint(point, planeAt(i), planeAt(j), planeAt(k));
                bool outside = false;
                for (int t = 0; t < count; ++t) {
                    if (SameFloatPlane(planeAt(t), planeAt(i)) || SameFloatPlane(planeAt(t), planeAt(j))
                        || SameFloatPlane(planeAt(t), planeAt(k)))
                        continue;
                    if (FloatDot(planeAt(t), point) - planeAt(t)[3] > 0.1f) {
                        outside = true;
                        break;
                    }
                }
                if (outside)
                    continue;
                points++;
                int ids[3] = { i, j, k };
                for (int s = 0; s < 3; ++s) {
                    const float* plane = planeAt(ids[s]);
                    float err = FloatDot(plane, point) - plane[3];
                    float tol = 0.1f;
                    float scaled = fabsf(plane[3]) * 0.00001f;
                    if (scaled > tol)
                        tol = scaled;
                    slack = min(slack, tol - fabsf(err));
                }
            }
        }
    }
    if (worstSlack)
        *worstSlack = points > 0 ? slack : -1.f;
    return points;
}

PlaneHit AuditFacePlanes(const vector<FaceOut>& faces)
{
    PlaneHit hit;
    int count = (int)faces.size();
    if (count < 4)
        return hit;
    vector<float> planes((size_t)count * 4);
    for (int i = 0; i < count; ++i) {
        float p0[3], p1[3], p2[3];
        LoadFacePoint(faces[i].p[0], p0);
        LoadFacePoint(faces[i].p[1], p1);
        LoadFacePoint(faces[i].p[2], p2);
        if (!PlaneFromFacePoints(p0, p1, p2, &planes[(size_t)i * 4])) {
            hit.slack = -1000.f;
            hit.tri[0] = i;
            return hit;
        }
    }
    auto planeAt = [&](int i) -> const float* { return &planes[(size_t)i * 4]; };
    int inside = 0;
    float worst = 1e30f;
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            if (SameFloatPlane(planeAt(i), planeAt(j)))
                continue;
            for (int k = j + 1; k < count; ++k) {
                if (SameFloatPlane(planeAt(i), planeAt(k)) || SameFloatPlane(planeAt(j), planeAt(k)))
                    continue;
                float point[3];
                if (!IntersectFloatPlanes(planeAt(i), planeAt(j), planeAt(k), point))
                    continue;
                SnapFloatPoint(point, planeAt(i), planeAt(j), planeAt(k));
                bool outside = false;
                for (int t = 0; t < count; ++t) {
                    if (t == i || t == j || t == k)
                        continue;
                    if (SameFloatPlane(planeAt(t), planeAt(i)) || SameFloatPlane(planeAt(t), planeAt(j))
                        || SameFloatPlane(planeAt(t), planeAt(k)))
                        continue;
                    float dist = FloatDot(planeAt(t), point) - planeAt(t)[3];
                    if (dist > 0.1f) {
                        outside = true;
                        break;
                    }
                }
                if (outside)
                    continue;
                inside++;
                int ids[3] = { i, j, k };
                for (int s = 0; s < 3; ++s) {
                    const float* plane = planeAt(ids[s]);
                    float err = FloatDot(plane, point) - plane[3];
                    float tol = 0.1f;
                    float scaled = fabsf(plane[3]) * 0.00001f;
                    if (scaled > tol)
                        tol = scaled;
                    float slack = tol - fabsf(err);
                    if (slack < worst) {
                        worst = slack;
                        hit.slack = slack;
                        hit.tri[0] = i;
                        hit.tri[1] = j;
                        hit.tri[2] = k;
                        hit.which = s;
                    }
                }
            }
        }
    }
    if (inside == 0) {
        hit.slack = -1.f;
        hit.tri[0] = hit.tri[1] = hit.tri[2] = -1;
    }
    return hit;
}

bool PointsOnPlane(const PlaneD& plane, const Vec3& anchor, float span, float spin, Vec3 out[3])
{
    float n[3] = { (float)plane.n[0], (float)plane.n[1], (float)plane.n[2] };
    float len = sqrtf(FloatDot(n, n));
    if (len < 1e-12f)
        return false;
    n[0] /= len;
    n[1] /= len;
    n[2] /= len;
    float dist = n[0] * (float)anchor.x + n[1] * (float)anchor.y + n[2] * (float)anchor.z;

    int axis = 0;
    if (fabsf(n[1]) > fabsf(n[axis]))
        axis = 1;
    if (fabsf(n[2]) > fabsf(n[axis]))
        axis = 2;
    float tmp[3] = {};
    tmp[(axis + 1) % 3] = 1.f;
    float along = FloatDot(tmp, n);
    float u[3] = { tmp[0] - along * n[0], tmp[1] - along * n[1], tmp[2] - along * n[2] };
    float ulen = sqrtf(FloatDot(u, u));
    if (ulen < 1e-12f)
        return false;
    u[0] /= ulen;
    u[1] /= ulen;
    u[2] /= ulen;
    float v[3];
    FloatCross(n, u, v);

    float cs = cosf(spin);
    float sn = sinf(spin);
    float ur[3], vr[3];
    for (int i = 0; i < 3; ++i) {
        ur[i] = cs * u[i] + sn * v[i];
        vr[i] = -sn * u[i] + cs * v[i];
    }

    float center[3] = { (float)anchor.x, (float)anchor.y, (float)anchor.z };
    float shift = FloatDot(center, n) - dist;
    center[0] -= shift * n[0];
    center[1] -= shift * n[1];
    center[2] -= shift * n[2];
    float room = 120000.f;
    for (int i = 0; i < 3; ++i)
        room = min(room, 120000.f - fabsf(center[i]));
    if (span > room)
        span = room;
    if (span < 64.f)
        span = 64.f;

    auto place = [&](float su, float sv, float dest[3]) {
        dest[0] = center[0] + ur[0] * su + vr[0] * sv;
        dest[1] = center[1] + ur[1] * su + vr[1] * sv;
        dest[2] = center[2] + ur[2] * su + vr[2] * sv;
        int uAxis = (axis + 1) % 3;
        int vAxis = (axis + 2) % 3;
        dest[axis] = (dist - n[uAxis] * dest[uAxis] - n[vAxis] * dest[vAxis]) / n[axis];
    };
    float raw[3][3];
    place(0.f, 0.f, raw[0]);
    place(0.f, span, raw[1]);
    place(span, 0.f, raw[2]);
    for (int i = 0; i < 3; ++i) {
        out[i].x = RoundTripCoord(raw[i][0]);
        out[i].y = RoundTripCoord(raw[i][1]);
        out[i].z = RoundTripCoord(raw[i][2]);
    }
    float p0[3], p1[3], p2[3], recovered[4];
    LoadFacePoint(out[0], p0);
    LoadFacePoint(out[1], p1);
    LoadFacePoint(out[2], p2);
    if (!PlaneFromFacePoints(p0, p1, p2, recovered))
        return false;
    float align = FloatDot(recovered, n);
    if (align < 0.f) {
        swap(out[1], out[2]);
        align = -align;
    }
    return align > 0.999f;
}

void ApplyPlanePoints(vector<FaceOut>& faces, const vector<PlaneD>& planes, const vector<Vec3>& anchors,
                      float span, float spin)
{
    for (int i = 0; i < (int)faces.size(); ++i) {
        Vec3 tri[3];
        if (PointsOnPlane(planes[i], anchors[i], span, spin, tri)) {
            faces[i].p[0] = tri[0];
            faces[i].p[1] = tri[1];
            faces[i].p[2] = tri[2];
        }
    }
}

bool SnapNearAxial(PlaneD& plane, const Vec3& anchor, double extent)
{
    int axis = 0;
    double dominant = fabs(plane.n[0]);
    for (int i = 1; i < 3; ++i) {
        if (fabs(plane.n[i]) > dominant) {
            dominant = fabs(plane.n[i]);
            axis = i;
        }
    }
    double off = sqrt(max(0.0, 1.0 - dominant * dominant));
    if (off * max(extent, 1.0) > 0.05 || dominant < 0.5)
        return false;
    double sign = plane.n[axis] < 0.0 ? -1.0 : 1.0;
    double anchorCoord = axis == 0 ? anchor.x : axis == 1 ? anchor.y : anchor.z;
    plane.n[0] = plane.n[1] = plane.n[2] = 0.0;
    plane.n[axis] = sign;
    plane.dist = sign * anchorCoord;
    return true;
}

void CollapseIllConditioned(vector<FaceOut>& faces, const vector<Vec3>& anchors)
{
    if (faces.size() < 4)
        return;
    for (int step = 0; step < (int)faces.size(); ++step) {
        PlaneHit hit = AuditFacePlanes(faces);
        if (hit.tri[0] < 0 || hit.tri[1] < 0 || hit.tri[2] < 0 || hit.slack >= 0.05f)
            return;
        int id[3] = { hit.tri[0], hit.tri[1], hit.tri[2] };
        float pl[3][4];
        for (int s = 0; s < 3; ++s) {
            if (id[s] < 0 || id[s] >= (int)faces.size())
                return;
            float p0[3], p1[3], p2[3];
            LoadFacePoint(faces[id[s]].p[0], p0);
            LoadFacePoint(faces[id[s]].p[1], p1);
            LoadFacePoint(faces[id[s]].p[2], p2);
            if (!PlaneFromFacePoints(p0, p1, p2, pl[s]))
                return;
        }
        int keep = id[0];
        int drop = id[1];
        int keepLocal = 0;
        int dropLocal = 1;
        float bestDot = -2.f;
        for (int a = 0; a < 3; ++a) {
            for (int b = a + 1; b < 3; ++b) {
                float dot = FloatDot(pl[a], pl[b]);
                if (dot > bestDot) {
                    bestDot = dot;
                    keep = id[a];
                    drop = id[b];
                    keepLocal = a;
                    dropLocal = b;
                }
            }
        }
        if (bestDot < 0.75f)
            return;
        if (anchors.size() == faces.size() && bestDot < 0.999f) {
            auto gap = [](const float* plane, const Vec3& p) {
                return fabs(plane[0] * p.x + plane[1] * p.y + plane[2] * p.z - plane[3]);
            };
            double sep = max(gap(pl[keepLocal], anchors[drop]), gap(pl[dropLocal], anchors[keep]));
            if (sep > 0.1)
                return;
        }
        Vec3 saved[3] = { faces[drop].p[0], faces[drop].p[1], faces[drop].p[2] };
        if (saved[0].x == faces[keep].p[0].x && saved[0].y == faces[keep].p[0].y && saved[0].z == faces[keep].p[0].z
            && saved[1].x == faces[keep].p[1].x && saved[1].y == faces[keep].p[1].y && saved[1].z == faces[keep].p[1].z
            && saved[2].x == faces[keep].p[2].x && saved[2].y == faces[keep].p[2].y && saved[2].z == faces[keep].p[2].z)
            return;
        faces[drop].p[0] = faces[keep].p[0];
        faces[drop].p[1] = faces[keep].p[1];
        faces[drop].p[2] = faces[keep].p[2];
        PlaneHit next = AuditFacePlanes(faces);
        if (next.slack <= hit.slack) {
            faces[drop].p[0] = saved[0];
            faces[drop].p[1] = saved[1];
            faces[drop].p[2] = saved[2];
            return;
        }
    }
}

void FitBrushPlanes(vector<FaceOut>& faces, const vector<PlaneD>& sourcePlanes, const vector<Vec3>& anchors)
{
    struct SealIllConditioned {
        vector<FaceOut>* faces;
        const vector<Vec3>* anchors;
        ~SealIllConditioned()
        {
            if (faces && anchors)
                CollapseIllConditioned(*faces, *anchors);
        }
    } seal{ &faces, &anchors };
    if (faces.size() < 4 || faces.size() != sourcePlanes.size() || faces.size() != anchors.size())
        return;
    vector<PlaneD> planes = sourcePlanes;
    vector<Vec3> onPlane(anchors.size());
    double extent = 0;
    for (int i = 0; i < (int)planes.size(); ++i) {
        double dist = planes[i].Dist(anchors[i]);
        onPlane[i] = { anchors[i].x - planes[i].n[0] * dist, anchors[i].y - planes[i].n[1] * dist,
                       anchors[i].z - planes[i].n[2] * dist };
        for (int j = 0; j < i; ++j)
            extent = max(extent, Len(Sub(onPlane[i], onPlane[j])));
    }
    if (extent < 1)
        extent = 1;
    bool snapped = false;
    for (int i = 0; i < (int)planes.size(); ++i)
        snapped = SnapNearAxial(planes[i], onPlane[i], extent) || snapped;

    auto capture = [&]() {
        vector<Vec3> saved;
        saved.reserve(faces.size() * 3);
        for (const FaceOut& face : faces)
            for (int k = 0; k < 3; ++k)
                saved.push_back(face.p[k]);
        return saved;
    };
    auto restore = [&](const vector<Vec3>& saved) {
        int at = 0;
        for (FaceOut& face : faces)
            for (int k = 0; k < 3; ++k)
                face.p[k] = saved[at++];
    };

    vector<Vec3> bestPoints = capture();
    PlaneHit best = AuditFacePlanes(faces);
    if (best.slack >= 0.04f)
        return;
    float brushSpan = (float)max(extent, 16.0);
    if (snapped) {
        ApplyPlanePoints(faces, planes, onPlane, brushSpan, 0.f);
        PlaneHit hit = AuditFacePlanes(faces);
        if (hit.slack >= best.slack) {
            best = hit;
            bestPoints = capture();
        }
        restore(bestPoints);
    }
    if (best.slack >= 0.04f)
        return;

    const float spans[] = { brushSpan * 0.5f, brushSpan, brushSpan * 2.f };
    const float spins[] = { 0.f, 0.2f, 0.45f, 0.7f, 1.0f, 1.4f, 1.9f, 2.4f };
    float useSpan = brushSpan;
    float useSpin = 0.f;
    for (float span : spans) {
        for (float spin : spins) {
            ApplyPlanePoints(faces, planes, onPlane, span, spin);
            PlaneHit hit = AuditFacePlanes(faces);
            if (hit.slack > best.slack) {
                best = hit;
                bestPoints = capture();
                useSpan = span;
                useSpin = spin;
            }
            if (best.slack >= 0.04f) {
                restore(bestPoints);
                return;
            }
        }
    }
    restore(bestPoints);

    vector<PlaneD> live = planes;
    double moveCap = 0.5 / extent;
    for (int step = 0; step < 12 && best.slack < 0.04f && best.tri[0] >= 0; ++step) {
        int index = best.tri[best.which];
        int otherA = best.tri[(best.which + 1) % 3];
        int otherB = best.tri[(best.which + 2) % 3];
        float pa[4], pb[4];
        float a0[3], a1[3], a2[3], b0[3], b1[3], b2[3];
        LoadFacePoint(faces[otherA].p[0], a0);
        LoadFacePoint(faces[otherA].p[1], a1);
        LoadFacePoint(faces[otherA].p[2], a2);
        LoadFacePoint(faces[otherB].p[0], b0);
        LoadFacePoint(faces[otherB].p[1], b1);
        LoadFacePoint(faces[otherB].p[2], b2);
        if (!PlaneFromFacePoints(a0, a1, a2, pa) || !PlaneFromFacePoints(b0, b1, b2, pb))
            break;
        float dir[3];
        FloatCross(pa, pb, dir);
        float dirLen = sqrtf(FloatDot(dir, dir));
        if (dirLen < 1e-8f)
            break;
        dir[0] /= dirLen;
        dir[1] /= dirLen;
        dir[2] /= dirLen;

        bool improved = false;
        PlaneD base = live[index];
        const double epsilons[] = { 1e-6, 3e-6, 1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 3e-3 };
        for (double eps : epsilons) {
            if (eps > moveCap)
                break;
            for (double sign : { 1.0, -1.0 }) {
                PlaneD trial = base;
                trial.n[0] += dir[0] * eps * sign;
                trial.n[1] += dir[1] * eps * sign;
                trial.n[2] += dir[2] * eps * sign;
                double nlen = sqrt(trial.n[0] * trial.n[0] + trial.n[1] * trial.n[1] + trial.n[2] * trial.n[2]);
                if (nlen < 1e-12)
                    continue;
                trial.n[0] /= nlen;
                trial.n[1] /= nlen;
                trial.n[2] /= nlen;
                if (trial.n[0] * planes[index].n[0] + trial.n[1] * planes[index].n[1] + trial.n[2] * planes[index].n[2] < 0.999)
                    continue;
                trial.dist = trial.n[0] * onPlane[index].x + trial.n[1] * onPlane[index].y + trial.n[2] * onPlane[index].z;
                Vec3 tri[3];
                if (!PointsOnPlane(trial, onPlane[index], useSpan, useSpin, tri))
                    continue;
                faces[index].p[0] = tri[0];
                faces[index].p[1] = tri[1];
                faces[index].p[2] = tri[2];
                PlaneHit hit = AuditFacePlanes(faces);
                if (hit.slack > best.slack + 1e-5f) {
                    best = hit;
                    bestPoints = capture();
                    live[index] = trial;
                    improved = true;
                    if (best.slack >= 0.04f)
                        break;
                }
            }
            if (best.slack >= 0.04f)
                break;
        }
        restore(bestPoints);
        if (best.slack >= 0.04f)
            break;
        if (!improved) {
            bool shifted = false;
            const double shifts[] = { 0.002, 0.01, 0.05 };
            for (double shift : shifts) {
                for (double sign : { 1.0, -1.0 }) {
                    PlaneD trial = base;
                    trial.dist += shift * sign;
                    Vec3 tri[3];
                    if (!PointsOnPlane(trial, onPlane[index], useSpan, useSpin, tri))
                        continue;
                    faces[index].p[0] = tri[0];
                    faces[index].p[1] = tri[1];
                    faces[index].p[2] = tri[2];
                    PlaneHit hit = AuditFacePlanes(faces);
                    if (hit.slack > best.slack + 1e-5f) {
                        best = hit;
                        bestPoints = capture();
                        live[index] = trial;
                        shifted = true;
                        if (best.slack >= 0.04f)
                            break;
                    }
                }
                if (best.slack >= 0.04f)
                    break;
            }
            restore(bestPoints);
            if (!shifted)
                break;
        }
        restore(bestPoints);
    }
    restore(bestPoints);
}

void OrientWinding(vector<Vec3>& winding, const PlaneD& plane)
{
    if (winding.size() < 3)
        return;
    Vec3 n = { plane.n[0], plane.n[1], plane.n[2] };
    Vec3 area{ 0, 0, 0 };
    for (size_t i = 1; i + 1 < winding.size(); ++i)
        area = Add(area, Cross(Sub(winding[i], winding[0]), Sub(winding[i + 1], winding[0])));
    if (Dot(area, n) < 0.0)
        reverse(winding.begin(), winding.end());
}

double MinorSpan(const vector<Vec3>& winding, const PlaneD& plane)
{
    if (winding.size() < 3)
        return 0;
    Vec3 n = { plane.n[0], plane.n[1], plane.n[2] };
    Vec3 axis = fabs(n.z) < 0.9 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
    Vec3 tangent = Cross(n, axis);
    double tlen = Len(tangent);
    if (tlen < 1e-8)
        return 0;
    tangent = Mul(tangent, 1.0 / tlen);
    Vec3 bitangent = Cross(n, tangent);
    double minU = 1e30, maxU = -1e30, minV = 1e30, maxV = -1e30;
    for (const Vec3& p : winding) {
        double u = Dot(p, tangent);
        double v = Dot(p, bitangent);
        minU = min(minU, u);
        maxU = max(maxU, u);
        minV = min(minV, v);
        maxV = max(maxV, v);
    }
    return min(maxU - minU, maxV - minV);
}

bool Solve3(const double m[3][3], const double b[3], double x[3]);

bool IntersectPlanes(const PlaneD& a, const PlaneD& b, const PlaneD& c, Vec3& out)
{
    double m[3][3] = {
        { a.n[0], a.n[1], a.n[2] },
        { b.n[0], b.n[1], b.n[2] },
        { c.n[0], c.n[1], c.n[2] }
    };
    double rhs[3] = { a.dist, b.dist, c.dist };
    double x[3];
    if (!Solve3(m, rhs, x))
        return false;
    out = { x[0], x[1], x[2] };
    return true;
}

bool InsideConvex(const vector<Vec3>& w, const Vec3& p, const Vec3& n, double slop)
{
    if (w.size() < 3)
        return false;
    for (size_t i = 0; i < w.size(); ++i) {
        Vec3 edge = Sub(w[(i + 1) % w.size()], w[i]);
        double edgeLen = Len(edge);
        if (edgeLen < 1e-8)
            continue;
        double side = Dot(Cross(edge, Sub(p, w[i])), n);
        if (side < -slop * edgeLen)
            return false;
    }
    return true;
}

void BoundsOf(const vector<Vec3>& w, double mins[3], double maxs[3])
{
    mins[0] = maxs[0] = w[0].x;
    mins[1] = maxs[1] = w[0].y;
    mins[2] = maxs[2] = w[0].z;
    for (const Vec3& p : w) {
        mins[0] = min(mins[0], p.x); maxs[0] = max(maxs[0], p.x);
        mins[1] = min(mins[1], p.y); maxs[1] = max(maxs[1], p.y);
        mins[2] = min(mins[2], p.z); maxs[2] = max(maxs[2], p.z);
    }
}

bool BoundsOverlap(const double mins[3], const double maxs[3], const Vec3& a, const Vec3& b, const Vec3& c, double pad)
{
    double tmin[3] = { min(a.x, min(b.x, c.x)) - pad, min(a.y, min(b.y, c.y)) - pad, min(a.z, min(b.z, c.z)) - pad };
    double tmax[3] = { max(a.x, max(b.x, c.x)) + pad, max(a.y, max(b.y, c.y)) + pad, max(a.z, max(b.z, c.z)) + pad };
    for (int i = 0; i < 3; ++i) {
        if (tmax[i] < mins[i] || tmin[i] > maxs[i])
            return false;
    }
    return true;
}

void TextureAxes(const double n[3], double xv[3], double yv[3], int& s, int& t)
{
    static const double table[6][3][3] = {
        { { 0, 0, 1 }, { 1, 0, 0 }, { 0, -1, 0 } },
        { { 0, 0, -1 }, { 1, 0, 0 }, { 0, -1, 0 } },
        { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 } },
        { { -1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 } },
        { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, -1 } },
        { { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, -1 } }
    };
    int best = 0;
    double bestDot = -1e30;
    for (int i = 0; i < 6; ++i) {
        double d = n[0] * table[i][0][0] + n[1] * table[i][0][1] + n[2] * table[i][0][2];
        if (d > bestDot) {
            bestDot = d;
            best = i;
        }
    }
    memcpy(xv, table[best][1], sizeof(double) * 3);
    memcpy(yv, table[best][2], sizeof(double) * 3);
    auto axisOf = [](const double v[3]) {
        if (v[0] != 0.0) return 0;
        if (v[1] != 0.0) return 1;
        return 2;
    };
    s = axisOf(xv);
    t = axisOf(yv);
}

double Det3(const double m[3][3])
{
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

bool Solve3(const double m[3][3], const double b[3], double x[3])
{
    double det = Det3(m);
    if (fabs(det) < 1e-12)
        return false;
    for (int col = 0; col < 3; ++col) {
        double copy[3][3];
        memcpy(copy, m, sizeof(copy));
        for (int row = 0; row < 3; ++row)
            copy[row][col] = b[row];
        x[col] = Det3(copy) / det;
    }
    return true;
}

bool ExtractTex(const double planeN[3], double planeDist, double tex[2][4],
                const double xv[3], const double yv[3], int s, int t, FaceOut& face)
{
    int r = 3 - s - t;
    if (fabs(planeN[r]) > 1e-8) {
        for (int i = 0; i < 2; ++i) {
            if (tex[i][r] == 0.0)
                continue;
            double d = tex[i][r] / planeN[r];
            tex[i][0] -= d * planeN[0];
            tex[i][1] -= d * planeN[1];
            tex[i][2] -= d * planeN[2];
            tex[i][3] += planeDist * d;
        }
    }

    double lenT2 = tex[1][0] * tex[1][0] + tex[1][1] * tex[1][1] + tex[1][2] * tex[1][2];
    if (lenT2 < 1e-18)
        return false;
    double skew = (tex[0][0] * tex[1][0] + tex[0][1] * tex[1][1] + tex[0][2] * tex[1][2]) / lenT2;
    tex[0][0] -= skew * tex[1][0];
    tex[0][1] -= skew * tex[1][1];
    tex[0][2] -= skew * tex[1][2];

    double lenS = sqrt(tex[0][0] * tex[0][0] + tex[0][1] * tex[0][1] + tex[0][2] * tex[0][2]);
    double lenT = sqrt(lenT2);
    if (lenS < 1e-12 || lenT < 1e-12)
        return false;

    double scaleS = 1.0 / lenS;
    double scaleT = 1.0 / lenT;
    double p0s = xv[s];
    double p1t = yv[t];
    if (tex[0][s] * p0s * tex[1][t] * p1t - tex[0][t] * p0s * tex[1][s] * p1t < 0.0)
        scaleT = -scaleT;

    double rotate = atan2(tex[0][t] * p0s, tex[0][s] * p0s) * (180.0 / kPi);
    double shiftS = -tex[0][3] * scaleS;
    double shiftT = -tex[1][3] * scaleT;
    if (!isfinite(scaleS) || !isfinite(scaleT) || !isfinite(shiftS) || !isfinite(shiftT)
        || !isfinite(rotate) || !isfinite(skew) || scaleS == 0.0 || scaleT == 0.0)
        return false;

    face.scaleS = scaleS;
    face.scaleT = scaleT;
    face.shiftS = shiftS;
    face.shiftT = shiftT;
    face.rotate = rotate;
    face.skew = skew;
    return true;
}

struct Hit {
    double x, y, z, s, t;
};

bool FitFace(const vector<Hit>& hits, const PlaneD& plane, FaceOut& face)
{
    if (hits.size() < 3)
        return false;

    int axisDrop = 0;
    if (fabs(plane.n[0]) < fabs(plane.n[1]))
        axisDrop = 1;
    if (fabs(plane.n[axisDrop]) < fabs(plane.n[2]))
        axisDrop = 2;
    int axisU = (~axisDrop) & 1;
    int axisV = (~axisDrop) & 2;

    double ata[3][3] = {};
    double atbS[3] = {};
    double atbT[3] = {};
    for (const Hit& hit : hits) {
        double xyz[3] = { hit.x, hit.y, hit.z };
        double row[3] = { xyz[axisU], xyz[axisV], 1.0 };
        for (int r = 0; r < 3; ++r) {
            atbS[r] += row[r] * hit.s;
            atbT[r] += row[r] * hit.t;
            for (int c = 0; c < 3; ++c)
                ata[r][c] += row[r] * row[c];
        }
    }
    double solS[3], solT[3];
    if (!Solve3(ata, atbS, solS) || !Solve3(ata, atbT, solT))
        return false;

    double tex[2][4] = {};
    tex[0][axisU] = solS[0];
    tex[0][axisV] = solS[1];
    tex[0][3] = solS[2];
    tex[1][axisU] = solT[0];
    tex[1][axisV] = solT[1];
    tex[1][3] = solT[2];

    double xv[3], yv[3];
    int s = 0, t = 0;
    TextureAxes(plane.n, xv, yv, s, t);
    return ExtractTex(plane.n, plane.dist, tex, xv, yv, s, t, face);
}

void CopyLightmap(FaceOut& face, const FaceOut& fitted)
{
    face.lmScaleS = fitted.scaleS;
    face.lmScaleT = fitted.scaleT;
    face.lmShiftS = fitted.shiftS;
    face.lmShiftT = fitted.shiftT;
    face.lmRotate = fitted.rotate;
    face.lmSkew = fitted.skew;
}

bool MakePlane(const BspFile& bsp, const BspBrushSide& side, int local, const Vec3& origin, PlaneD& out)
{
    double n[3] = {};
    double dist = 0;
    if (local < 6) {
        int axis = local >> 1;
        float stored = 0;
        memcpy(&stored, &side.planeOrDist, sizeof(stored));
        if (local & 1) {
            n[axis] = 1.0;
            dist = stored;
        } else {
            n[axis] = -1.0;
            dist = -(double)stored;
        }
    } else {
        int planeNum = (int)side.planeOrDist;
        if (planeNum < 0 || planeNum >= (int)bsp.planes.size())
            return false;
        const BspPlane& src = bsp.planes[planeNum];
        double len = sqrt((double)src.normal[0] * src.normal[0]
                         + (double)src.normal[1] * src.normal[1]
                         + (double)src.normal[2] * src.normal[2]);
        if (len < 1e-8)
            return false;
        n[0] = src.normal[0] / len;
        n[1] = src.normal[1] / len;
        n[2] = src.normal[2] / len;
        dist = src.dist / len;
    }
    dist += n[0] * origin.x + n[1] * origin.y + n[2] * origin.z;
    out.n[0] = n[0];
    out.n[1] = n[1];
    out.n[2] = n[2];
    out.dist = dist;
    return true;
}

struct Names {
    vector<string> list;
    unordered_map<string, int> lookup;
    int Intern(const string& name)
    {
        auto found = lookup.find(name);
        if (found != lookup.end())
            return found->second;
        int id = (int)list.size();
        list.push_back(name);
        lookup.emplace(name, id);
        return id;
    }
};

struct BuiltSides {
    vector<SideGeom> sides;
};

static int gDropSides = 0;
static int gDropMat = 0;
static int gDropPlane = 0;
static int gDropFaces = 0;
static int gDropSolid = 0;

bool BuildBrush(const BspFile& bsp, int brushIndex, int sideStart, const Vec3& origin,
                const vector<int>& matName, const Names& names, int caulk,
                const vector<int>& edgeStart, BrushOut& brush, BuiltSides& sides)
{
    const BspBrush& src = bsp.brushes[brushIndex];
    if (src.numSides < 4) {
        ++gDropSides;
        return false;
    }
    if (src.materialNum >= bsp.materials.size()) {
        ++gDropMat;
        return false;
    }

    vector<PlaneD> planes(src.numSides);
    for (int local = 0; local < src.numSides; ++local) {
        if (!MakePlane(bsp, bsp.sides[sideStart + local], local, origin, planes[local])) {
            ++gDropPlane;
            return false;
        }
    }

    brush.contents = bsp.materials[src.materialNum].contentFlags;
    brush.faces.clear();

    for (int local = 0; local < src.numSides; ++local) {
        vector<Vec3> winding;
        bool fromEdges = false;
        int sideIndex = sideStart + local;
        int e0 = (sideIndex < (int)edgeStart.size()) ? edgeStart[sideIndex] : 0;
        int e1 = (sideIndex + 1 < (int)edgeStart.size()) ? edgeStart[sideIndex + 1] : e0;
        if (e1 - e0 >= 3 && e1 <= (int)bsp.edges.size()) {
            fromEdges = true;
            for (int e = 0; e < e1 - e0; ++e) {
                int s0 = bsp.edges[e0 + e];
                int s1 = bsp.edges[e0 + ((e + 1) % (e1 - e0))];
                Vec3 point;
                if (s0 >= src.numSides || s1 >= src.numSides || s0 == local || s1 == local
                    || !IntersectPlanes(planes[local], planes[s0], planes[s1], point)) {
                    fromEdges = false;
                    break;
                }
                winding.push_back(point);
            }
        }
        if (!fromEdges || winding.size() < 3) {
            winding = BaseWinding(planes[local]);
            for (int other = 0; other < src.numSides && !winding.empty(); ++other) {
                if (other == local)
                    continue;
                winding = ClipWinding(winding, planes[other]);
            }
        }
        OrientWinding(winding, planes[local]);
        if (winding.size() < 3)
            continue;

        Vec3 tri[3];
        if (!PickFaceFromWinding(winding, planes[local], tri) && !FacePoints(winding, planes[local], tri)) {
            Vec3 anchor{ 0, 0, 0 };
            for (const Vec3& p : winding)
                anchor = Add(anchor, p);
            anchor = Mul(anchor, 1.0 / (double)winding.size());
            double span = 16;
            for (const Vec3& p : winding)
                span = max(span, Len(Sub(p, anchor)));
            if (!PointsOnPlane(planes[local], anchor, (float)span, 0.f, tri))
                continue;
        }

        FaceOut face;
        face.p[0] = tri[0];
        face.p[1] = tri[1];
        face.p[2] = tri[2];
        unsigned matNum = bsp.sides[sideStart + local].materialNum;
        int nameId = caulk;
        unsigned surfaceFlags = 0;
        if (matNum < bsp.materials.size()) {
            nameId = matName[matNum];
            surfaceFlags = bsp.materials[matNum].surfaceFlags;
        }
        if (nameId < 0 || nameId >= (int)names.list.size())
            nameId = caulk;
        face.material = names.list[nameId];
        brush.faces.push_back(face);

        SideGeom side;
        side.nameId = nameId;
        side.drawNameId = nameId;
        side.surfaceFlags = surfaceFlags;
        side.plane = planes[local];
        side.faceIndex = (int)brush.faces.size() - 1;
        BoundsOf(winding, side.mins, side.maxs);
        side.winding = move(winding);
        sides.sides.push_back(move(side));
    }

    bool hasPortalBit = false;
    bool hasVisibleSolid = false;
    for (size_t i = 0; i < sides.sides.size(); ++i) {
        if (SideIsPortal(sides.sides[i].surfaceFlags))
            hasPortalBit = true;
        if (SideIsVisibleSolid(sides.sides[i].surfaceFlags))
            hasVisibleSolid = true;
    }
    const bool portalBrush = hasPortalBit;
    brush.portal = portalBrush;
    if (!portalBrush) {
        string caulkName = (caulk >= 0 && caulk < (int)names.list.size()) ? names.list[caulk] : string("caulk");
        for (size_t i = 0; i < brush.faces.size(); ++i) {
            const string& mat = brush.faces[i].material;
            if (!IsPortalMapMaterial(sides.sides[i].surfaceFlags, mat))
                continue;
            brush.faces[i].material = caulkName;
            sides.sides[i].nameId = caulk;
            sides.sides[i].surfaceFlags = kSurfNodraw;
        }
    } else if (!hasVisibleSolid) {
        int portalNodraw = caulk;
        auto pn = names.lookup.find("portal_nodraw");
        if (pn != names.lookup.end())
            portalNodraw = pn->second;
        const string& portalNodrawName = names.list[portalNodraw];
        for (size_t i = 0; i < brush.faces.size(); ++i) {
            if (IsPortalMapMaterial(sides.sides[i].surfaceFlags, brush.faces[i].material))
                continue;
            brush.faces[i].material = portalNodrawName;
            sides.sides[i].nameId = portalNodraw;
            sides.sides[i].surfaceFlags = kSurfNodraw;
        }
    }

    if (!portalBrush && brush.faces.size() >= 4 && brush.faces.size() == sides.sides.size()) {
        vector<PlaneD> fitPlanes;
        vector<Vec3> fitAnchors;
        fitPlanes.reserve(brush.faces.size());
        fitAnchors.reserve(brush.faces.size());
        for (int i = 0; i < (int)sides.sides.size(); ++i) {
            fitPlanes.push_back(sides.sides[i].plane);
            Vec3 anchor{ 0, 0, 0 };
            if (!sides.sides[i].winding.empty()) {
                for (const Vec3& point : sides.sides[i].winding)
                    anchor = Add(anchor, point);
                anchor = Mul(anchor, 1.0 / (double)sides.sides[i].winding.size());
            } else {
                anchor = Mul(Add(Add(brush.faces[i].p[0], brush.faces[i].p[1]), brush.faces[i].p[2]), 1.0 / 3.0);
            }
            fitAnchors.push_back(anchor);
        }
        float woundSlack = 0.f;
        if (BrushSolidPoints(brush.faces, &woundSlack) < 4 || woundSlack < 0.04f)
            FitBrushPlanes(brush.faces, fitPlanes, fitAnchors);
    }

    if (brush.faces.size() < 4) {
        ++gDropFaces;
        return false;
    }
    if (!portalBrush) {
        float solidSlack = 0.f;
        if (BrushSolidPoints(brush.faces, &solidSlack) < 4 || solidSlack < 0.04f) {
            ++gDropSolid;
            return false;
        }
    }
    return true;
}

struct VertRef {
    int type = 0;
    int index = 0;
    int nameId = 0;
};

template <typename Fn>
void ForSoups(const BspFile& bsp, int model, Fn&& fn)
{
    const BspModel& mdl = bsp.models[model];
    const int soupOrder[2] = { 1, 0 };
    for (int type : soupOrder) {
        int first = mdl.firstTriSoup[type];
        int count = mdl.triSoupCount[type];
        if (count == 0)
            continue;
        if (first < 0 || first + count > (int)bsp.soups[type].size())
            throw runtime_error("model draw-surface range is outside the bsp");
        for (int i = 0; i < count; ++i)
            fn(type, bsp.soups[type][first + i]);
    }
}

int ResolveVert(const BspTriSoup& soup, const vector<unsigned short>& indices, int i, int vertCount)
{
    if (i < 0 || i >= soup.indexCount)
        return -1;
    long long at = (long long)soup.firstIndex + i;
    if (at < 0 || at >= (long long)indices.size())
        return -1;
    unsigned idx = indices[(size_t)at];
    int resolved = (idx < soup.vertexCount) ? soup.firstVertex + (int)idx : (int)idx;
    if (resolved < 0 || resolved >= vertCount)
        return -1;
    return resolved;
}

string ResolveLayered(const string& name, const BspFile& bsp);

int MatchNameId(const BspFile& bsp, const Names& names, const vector<int>& matName, int matIndex)
{
    if (matIndex < 0 || matIndex >= (int)matName.size())
        return -1;
    string resolved = ResolveLayered(names.list[matName[matIndex]], bsp);
    auto found = names.lookup.find(resolved);
    if (found == names.lookup.end())
        return matName[matIndex];
    return found->second;
}

struct SideGrid {
    static constexpr double kPad = 8.0;
    static constexpr double kCell = 256.0;
    struct Key {
        int x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct Hash {
        size_t operator()(const Key& k) const
        {
            return ((size_t)k.x * 73856093u) ^ ((size_t)k.y * 19349663u) ^ ((size_t)k.z * 83492791u);
        }
    };
    unordered_map<Key, vector<int>, Hash> cells;
    vector<int> oversized;
    vector<int> stamp;
    int mark = 1;

    static int bucket(double v) { return (int)floor(v / kCell); }

    void build(const vector<SideGeom>& sides)
    {
        cells.clear();
        oversized.clear();
        stamp.assign(sides.size(), 0);
        mark = 1;
        for (int i = 0; i < (int)sides.size(); ++i) {
            const SideGeom& side = sides[i];
            int x0 = bucket(side.mins[0] - kPad);
            int x1 = bucket(side.maxs[0] + kPad);
            int y0 = bucket(side.mins[1] - kPad);
            int y1 = bucket(side.maxs[1] + kPad);
            int z0 = bucket(side.mins[2] - kPad);
            int z1 = bucket(side.maxs[2] + kPad);
            long long span = (long long)(x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1);
            if (span > 4096) {
                oversized.push_back(i);
                continue;
            }
            for (int x = x0; x <= x1; ++x)
                for (int y = y0; y <= y1; ++y)
                    for (int z = z0; z <= z1; ++z)
                        cells[{ x, y, z }].push_back(i);
        }
    }

    template <typename Fn>
    void query(const Vec3& a, const Vec3& b, const Vec3& c, Fn&& fn)
    {
        double minx = min(a.x, min(b.x, c.x));
        double miny = min(a.y, min(b.y, c.y));
        double minz = min(a.z, min(b.z, c.z));
        double maxx = max(a.x, max(b.x, c.x));
        double maxy = max(a.y, max(b.y, c.y));
        double maxz = max(a.z, max(b.z, c.z));
        int x0 = bucket(minx - kPad);
        int x1 = bucket(maxx + kPad);
        int y0 = bucket(miny - kPad);
        int y1 = bucket(maxy + kPad);
        int z0 = bucket(minz - kPad);
        int z1 = bucket(maxz + kPad);
        if (++mark == 0) {
            fill(stamp.begin(), stamp.end(), 0);
            mark = 1;
        }
        auto visit = [&](int idx) {
            if (stamp[idx] == mark)
                return;
            stamp[idx] = mark;
            fn(idx);
        };
        for (int idx : oversized)
            visit(idx);
        for (int x = x0; x <= x1; ++x) {
            for (int y = y0; y <= y1; ++y) {
                for (int z = z0; z <= z1; ++z) {
                    auto found = cells.find({ x, y, z });
                    if (found == cells.end())
                        continue;
                    for (int idx : found->second)
                        visit(idx);
                }
            }
        }
    }
};

struct VertUV {
    Hit albedo;
    Hit light;
};

bool ProjectFace(const FaceOut& face, const PlaneD& plane, double x, double y, double z, double& outS, double& outT)
{
    if (face.scaleS == 0.0 || face.scaleT == 0.0)
        return false;
    double xv[3], yv[3];
    int s = 0, t = 0;
    TextureAxes(plane.n, xv, yv, s, t);
    double rad = face.rotate * (kPi / 180.0);
    double cs = cos(rad);
    double sn = sin(rad);
    double row0[4] = {};
    double row1[4] = {};
    row0[s] = xv[s] / face.scaleS * cs;
    row0[t] = xv[s] / face.scaleS * sn;
    row1[s] = -sn * (yv[t] / face.scaleT);
    row1[t] = yv[t] / face.scaleT * cs;
    for (int i = 0; i < 3; ++i)
        row0[i] += face.skew * row1[i];
    row0[3] = -face.shiftS / face.scaleS;
    row1[3] = -face.shiftT / face.scaleT;
    double p[3] = { x, y, z };
    outS = row0[0] * p[0] + row0[1] * p[1] + row0[2] * p[2] + row0[3];
    outT = row1[0] * p[0] + row1[1] * p[1] + row1[2] * p[2] + row1[3];
    return true;
}

bool PickProjection(const vector<Hit>& hits, const PlaneD& plane, FaceOut& face)
{
    if (hits.size() < 3)
        return false;
    int bestCount = 0;
    FaceOut best;
    bool found = false;
    for (size_t base = 0; base + 2 < hits.size(); base += 3) {
        vector<Hit> tri(hits.begin() + base, hits.begin() + base + 3);
        FaceOut trial;
        if (!FitFace(tri, plane, trial))
            continue;
        int count = 0;
        for (const Hit& hit : hits) {
            double s = 0, t = 0;
            if (!ProjectFace(trial, plane, hit.x, hit.y, hit.z, s, t))
                continue;
            if (fabs(s - hit.s) <= 1e-3 && fabs(t - hit.t) <= 1e-3)
                count++;
        }
        if (!found || count > bestCount) {
            found = true;
            bestCount = count;
            best = trial;
        }
    }
    if (!found || bestCount < 3)
        return false;
    face = best;
    return true;
}

void FitAllBrushTextures(const BspFile& bsp, int model, const Vec3& origin, const vector<int>& matName,
                         const Names& names, vector<SideGeom>& sides, vector<BrushOut>& brushes,
                         const vector<int>& brushSideStart)
{
    if (sides.empty())
        return;
    auto inBrush = [&](int sideIndex, const Vec3& p) {
        int brush = sides[sideIndex].brushIndex;
        int i = sideIndex;
        while (i > 0 && sides[i - 1].brushIndex == brush)
            --i;
        for (; i < (int)sides.size() && sides[i].brushIndex == brush; ++i) {
            if (sides[i].plane.Dist(p) > 0.05)
                return false;
        }
        return true;
    };

    SideGrid grid;
    grid.build(sides);
    vector<vector<VertUV>> samples(sides.size());

    ForSoups(bsp, model, [&](int type, const BspTriSoup& soup) {
        if (soup.materialIndex >= bsp.materials.size() || soup.indexCount < 3)
            return;
        int nameId = MatchNameId(bsp, names, matName, soup.materialIndex);
        for (int i = 0; i + 2 < soup.indexCount; i += 3) {
            int ia = ResolveVert(soup, bsp.indices[type], i, (int)bsp.verts[type].size());
            int ib = ResolveVert(soup, bsp.indices[type], i + 1, (int)bsp.verts[type].size());
            int ic = ResolveVert(soup, bsp.indices[type], i + 2, (int)bsp.verts[type].size());
            if (ia < 0 || ib < 0 || ic < 0)
                continue;
            const BspDrawVert& va = bsp.verts[type][ia];
            const BspDrawVert& vb = bsp.verts[type][ib];
            const BspDrawVert& vc = bsp.verts[type][ic];
            Vec3 a = FromVert(va, origin);
            Vec3 b = FromVert(vb, origin);
            Vec3 c = FromVert(vc, origin);
            if (Len(Cross(Sub(b, a), Sub(c, a))) < 0.01)
                continue;
            grid.query(a, b, c, [&](int sideIndex) {
                const SideGeom& side = sides[sideIndex];
                if ((side.nameId != nameId && side.drawNameId != nameId) || side.faceIndex < 0)
                    return;
                if (fabs(side.plane.Dist(a)) > 0.05 || fabs(side.plane.Dist(b)) > 0.05 || fabs(side.plane.Dist(c)) > 0.05)
                    return;
                if (!inBrush(sideIndex, a) || !inBrush(sideIndex, b) || !inBrush(sideIndex, c))
                    return;
                vector<VertUV>& dest = samples[sideIndex];
                if (dest.size() >= 24)
                    return;
                dest.push_back({ { a.x, a.y, a.z, va.texCoord[0], va.texCoord[1] },
                                 { a.x, a.y, a.z, va.lmapCoord[0], va.lmapCoord[1] } });
                dest.push_back({ { b.x, b.y, b.z, vb.texCoord[0], vb.texCoord[1] },
                                 { b.x, b.y, b.z, vb.lmapCoord[0], vb.lmapCoord[1] } });
                dest.push_back({ { c.x, c.y, c.z, vc.texCoord[0], vc.texCoord[1] },
                                 { c.x, c.y, c.z, vc.lmapCoord[0], vc.lmapCoord[1] } });
            });
        }
    });

    for (int b = 0; b < (int)brushes.size(); ++b) {
        int begin = brushSideStart[b];
        int end = brushSideStart[b + 1];
        BrushOut& brush = brushes[b];
        for (int s = begin; s < end; ++s) {
            const SideGeom& side = sides[s];
            if (samples[s].size() < 3 || side.faceIndex < 0 || side.faceIndex >= (int)brush.faces.size())
                continue;
            vector<Hit> albedo;
            vector<Hit> light;
            albedo.reserve(samples[s].size());
            light.reserve(samples[s].size());
            for (const VertUV& vert : samples[s]) {
                albedo.push_back(vert.albedo);
                light.push_back(vert.light);
            }
            FaceOut& face = brush.faces[side.faceIndex];
            FaceOut albedoFit;
            if (PickProjection(albedo, side.plane, albedoFit)) {
                face.scaleS = albedoFit.scaleS;
                face.scaleT = albedoFit.scaleT;
                face.shiftS = albedoFit.shiftS;
                face.shiftT = albedoFit.shiftT;
                face.rotate = albedoFit.rotate;
                face.skew = albedoFit.skew;
            }
            FaceOut lightFit;
            if (PickProjection(light, side.plane, lightFit))
                CopyLightmap(face, lightFit);
        }
    }
}

bool SameCompiledPoint(const Vec3& a, const Vec3& b)
{
    return fabs(a.x - b.x) <= 0.001 && fabs(a.y - b.y) <= 0.001 && fabs(a.z - b.z) <= 0.001;
}

constexpr double kTrisOnEpsilon = 0.1;

bool VerticesOnBrushSides(SideGrid& grid, const vector<SideGeom>& sides,
                          const vector<vector<int>>& brushSides, int nameId,
                          const Vec3& a, const Vec3& b, const Vec3& c)
{
    auto onBrush = [&](const Vec3& p) {
        bool on = false;
        grid.query(p, p, p, [&](int sideIndex) {
            if (on)
                return;
            const SideGeom& side = sides[sideIndex];
            if (side.nameId != nameId && side.drawNameId != nameId)
                return;
            if (fabs(side.plane.Dist(p)) > 0.2)
                return;
            int brush = side.brushIndex;
            if (brush < 0 || brush >= (int)brushSides.size())
                return;
            for (int other : brushSides[brush]) {
                if (sides[other].plane.Dist(p) > 0.2)
                    return;
            }
            on = true;
        });
        return on;
    };
    return onBrush(a) && onBrush(b) && onBrush(c);
}

constexpr double kPatchWeld = 0.001;
constexpr double kPatchSt = 1e-5;

struct TriKey {
    int a, b, c;
    bool operator==(const TriKey& o) const { return a == o.a && b == o.b && c == o.c; }
};

struct TriKeyHash {
    size_t operator()(const TriKey& k) const
    {
        return ((size_t)k.a * 73856093u) ^ ((size_t)k.b * 19349663u) ^ ((size_t)k.c * 83492791u);
    }
};

TriKey CanonTri(int a, int b, int c)
{
    if (b < a && b <= c)
        return { b, c, a };
    if (c < a && c <= b)
        return { c, a, b };
    return { a, b, c };
}

MeshVertOut ToMeshVert(const WeldVert& v)
{
    MeshVertOut out;
    out.p = v.p;
    out.st[0] = v.st[0];
    out.st[1] = v.st[1];
    out.lm[0] = v.lm[0];
    out.lm[1] = v.lm[1];
    out.n[0] = v.n[0];
    out.n[1] = v.n[1];
    out.n[2] = v.n[2];
    memcpy(out.color, v.color, 4);
    return out;
}

Vec3 MeshFaceNormal(const MeshOut& mesh)
{
    Vec3 zero{ 0, 0, 0 };
    if (mesh.width < 2 || mesh.height < 2)
        return zero;
    for (int row = 0; row < mesh.height - 1; ++row) {
        for (int col = 0; col < mesh.width - 1; ++col) {
            const MeshVertOut& a = mesh.verts[(size_t)row * mesh.width + col];
            const MeshVertOut& b = mesh.verts[(size_t)row * mesh.width + col + 1];
            const MeshVertOut& d = mesh.verts[(size_t)(row + 1) * mesh.width + col];
            Vec3 d1 = Sub(b.p, d.p);
            Vec3 d2 = Sub(a.p, d.p);
            Vec3 n = Cross(d2, d1);
            if (Len(n) > 1e-4)
                return n;
        }
    }
    return zero;
}

void OrientMesh(MeshOut& mesh)
{
    Vec3 face = MeshFaceNormal(mesh);
    if (Len(face) < 1e-4)
        return;
    Vec3 sum{ 0, 0, 0 };
    int count = 0;
    for (const MeshVertOut& vert : mesh.verts) {
        if (vert.n[0] == 0.0f && vert.n[1] == 0.0f && vert.n[2] == 0.0f)
            continue;
        sum.x += vert.n[0];
        sum.y += vert.n[1];
        sum.z += vert.n[2];
        count++;
    }
    if (!count || Dot(face, sum) >= 0.0)
        return;
    for (int col = 0; col < mesh.width; ++col) {
        for (int row = 0; row < mesh.height / 2; ++row) {
            swap(mesh.verts[(size_t)row * mesh.width + col],
                 mesh.verts[(size_t)(mesh.height - 1 - row) * mesh.width + col]);
        }
    }
    for (MeshVertOut& vert : mesh.verts)
        vert.flag = 0;
}

void PushPieces(vector<MeshOut>& dest, MeshOut src, int& meshCount, int& vertCount)
{
    auto push = [&](MeshOut& piece) {
        meshCount++;
        vertCount += (int)piece.verts.size();
        dest.push_back(move(piece));
    };
    if (src.width < 2 || src.height < 2 || (int)src.verts.size() != src.width * src.height)
        return;
    if (src.width <= kMaxPatch && src.height <= kMaxPatch) {
        push(src);
        return;
    }
    for (int u0 = 0; u0 < src.width - 1; ) {
        int u1 = min(src.width, u0 + kMaxPatch);
        for (int v0 = 0; v0 < src.height - 1; ) {
            int v1 = min(src.height, v0 + kMaxPatch);
            int sw = u1 - u0;
            int sh = v1 - v0;
            MeshOut piece;
            piece.contents = src.contents;
            piece.material = src.material;
            piece.width = sw;
            piece.height = sh;
            piece.verts.resize((size_t)sw * sh);
            for (int sv = 0; sv < sh; ++sv) {
                for (int su = 0; su < sw; ++su) {
                    MeshVertOut vert = src.verts[(size_t)(v0 + sv) * src.width + (u0 + su)];
                    if (su == sw - 1 || sv == sh - 1)
                        vert.flag = 0;
                    piece.verts[(size_t)sv * sw + su] = vert;
                }
            }
            push(piece);
            if (v1 >= src.height)
                break;
            v0 = v1 - 1;
        }
        if (u1 >= src.width)
            break;
        u0 = u1 - 1;
    }
}

bool OrderCCW(int v[4], const vector<WeldVert>& verts, const Vec3& n)
{
    Vec3 center{ 0, 0, 0 };
    for (int i = 0; i < 4; ++i)
        center = Add(center, verts[v[i]].p);
    center = Mul(center, 0.25);
    Vec3 tangent = Sub(verts[v[1]].p, verts[v[0]].p);
    if (Len(tangent) < 1e-8)
        tangent = { 1, 0, 0 };
    else
        tangent = Mul(tangent, 1.0 / Len(tangent));
    Vec3 bitangent = Cross(n, tangent);

    struct Item { int id; double ang; };
    Item items[4];
    for (int i = 0; i < 4; ++i) {
        Vec3 d = Sub(verts[v[i]].p, center);
        items[i] = { v[i], atan2(Dot(d, bitangent), Dot(d, tangent)) };
    }
    sort(items, items + 4, [](const Item& a, const Item& b) { return a.ang < b.ang; });
    for (int i = 0; i < 4; ++i)
        v[i] = items[i].id;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
            if (v[i] == v[j])
                return false;
    Vec3 facing = Cross(Sub(verts[v[1]].p, verts[v[0]].p), Sub(verts[v[2]].p, verts[v[0]].p));
    if (Dot(facing, n) < 0)
        swap(v[1], v[3]);
    return true;
}

bool MakeQuad(const Tri& a, const Tri& b, int triA, int triB, const vector<WeldVert>& verts, Quad& quad)
{
    int idsA[3] = { a.a, a.b, a.c };
    int idsB[3] = { b.a, b.b, b.c };
    int shared[2], onlyA = -1, onlyB = -1, ns = 0;
    for (int i = 0; i < 3; ++i) {
        bool found = false;
        for (int j = 0; j < 3; ++j)
            found = found || idsA[i] == idsB[j];
        if (found) {
            if (ns < 2)
                shared[ns++] = idsA[i];
        } else {
            onlyA = idsA[i];
        }
    }
    for (int j = 0; j < 3; ++j) {
        bool found = false;
        for (int i = 0; i < 3; ++i)
            found = found || idsB[j] == idsA[i];
        if (!found)
            onlyB = idsB[j];
    }
    if (ns != 2 || onlyA < 0 || onlyB < 0)
        return false;
    int v[4] = { idsA[0], idsA[1], idsA[2], onlyB };
    Vec3 n = Cross(Sub(verts[a.b].p, verts[a.a].p), Sub(verts[a.c].p, verts[a.a].p));
    if (Len(n) < 1e-8)
        return false;
    n = Mul(n, 1.0 / Len(n));
    if (!OrderCCW(v, verts, n))
        return false;
    for (int i = 0; i < 4; ++i)
        quad.v[i] = v[i];
    quad.tri[0] = triA;
    quad.tri[1] = triB;
    return true;
}

struct GridCoord {
    int u, v;
};

bool TryGrid(const vector<Quad>& quads, const vector<WeldVert>& verts, const vector<Tri>& tris,
             unsigned contents, const string& material, vector<MeshOut>& dest, int& meshCount, int& vertCount)
{
    if (quads.empty() || quads.size() > 8192)
        return false;

    vector<int> degree(verts.size(), 0);
    for (const Quad& q : quads)
        for (int i = 0; i < 4; ++i)
            if (q.v[i] >= 0 && q.v[i] < (int)degree.size())
                degree[q.v[i]]++;

    int startQuad = -1;
    int startVert = -1;
    for (int qi = 0; qi < (int)quads.size() && startQuad < 0; ++qi) {
        for (int i = 0; i < 4; ++i) {
            if (degree[quads[qi].v[i]] == 1) {
                startQuad = qi;
                startVert = quads[qi].v[i];
                break;
            }
        }
    }
    if (startQuad < 0)
        return false;

    vector<Quad> local = quads;
    int rot = 0;
    for (; rot < 4; ++rot)
        if (local[startQuad].v[rot] == startVert)
            break;
    if (rot == 4)
        return false;
    int rotated[4];
    for (int k = 0; k < 4; ++k)
        rotated[k] = local[startQuad].v[(rot + k) % 4];
    for (int k = 0; k < 4; ++k)
        local[startQuad].v[k] = rotated[k];

    map<pair<int, int>, vector<int>> edgeQuads;
    for (int qi = 0; qi < (int)local.size(); ++qi) {
        for (int e = 0; e < 4; ++e) {
            int a = local[qi].v[e];
            int b = local[qi].v[(e + 1) % 4];
            if (a > b)
                swap(a, b);
            edgeQuads[{ a, b }].push_back(qi);
        }
    }

    unordered_map<int, GridCoord> coord;
    vector<char> queued(local.size(), 0);
    queue<int> pending;
    pending.push(startQuad);
    queued[startQuad] = 1;
    bool seeded = false;
    int visited = 0;

    auto apply = [&](const Quad& q) -> bool {
        int known = 0;
        GridCoord at[4];
        bool has[4] = {};
        for (int i = 0; i < 4; ++i) {
            auto found = coord.find(q.v[i]);
            if (found != coord.end()) {
                has[i] = true;
                at[i] = found->second;
                known++;
            }
        }
        if (known == 0) {
            if (seeded)
                return false;
            seeded = true;
            coord[q.v[0]] = { 0, 0 };
            coord[q.v[1]] = { 1, 0 };
            coord[q.v[2]] = { 1, 1 };
            coord[q.v[3]] = { 0, 1 };
            return true;
        }
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            if (!has[i] || !has[j])
                continue;
            int du = at[j].u - at[i].u;
            int dv = at[j].v - at[i].v;
            if (du * du + dv * dv != 1)
                continue;
            int ru = -dv;
            int rv = du;
            GridCoord expect[4];
            expect[i] = at[i];
            expect[j] = at[j];
            expect[(i + 2) % 4] = { at[j].u + ru, at[j].v + rv };
            expect[(i + 3) % 4] = { at[i].u + ru, at[i].v + rv };
            for (int k = 0; k < 4; ++k) {
                auto found = coord.find(q.v[k]);
                if (found == coord.end())
                    coord[q.v[k]] = expect[k];
                else if (found->second.u != expect[k].u || found->second.v != expect[k].v)
                    return false;
            }
            return true;
        }
        return false;
    };

    while (!pending.empty()) {
        int qi = pending.front();
        pending.pop();
        if (!apply(local[qi]))
            return false;
        visited++;
        for (int e = 0; e < 4; ++e) {
            int a = local[qi].v[e];
            int b = local[qi].v[(e + 1) % 4];
            if (a > b)
                swap(a, b);
            for (int other : edgeQuads[{ a, b }]) {
                if (!queued[other]) {
                    queued[other] = 1;
                    pending.push(other);
                }
            }
        }
    }
    if (visited != (int)local.size())
        return false;

    int minU = 0, minV = 0, maxU = 0, maxV = 0;
    bool init = false;
    for (const auto& item : coord) {
        if (!init) {
            minU = maxU = item.second.u;
            minV = maxV = item.second.v;
            init = true;
        } else {
            minU = min(minU, item.second.u);
            minV = min(minV, item.second.v);
            maxU = max(maxU, item.second.u);
            maxV = max(maxV, item.second.v);
        }
    }
    int width = maxU - minU + 1;
    int height = maxV - minV + 1;
    if (width < 2 || height < 2 || width > 128 || height > 128)
        return false;
    if ((long long)width * height != (long long)coord.size())
        return false;

    vector<int> at((size_t)width * height, -1);
    for (const auto& item : coord) {
        int u = item.second.u - minU;
        int v = item.second.v - minV;
        int slot = v * width + u;
        if (at[slot] != -1)
            return false;
        at[slot] = item.first;
    }
    for (int id : at)
        if (id < 0)
            return false;

    unordered_set<TriKey, TriKeyHash> triSet;
    for (const Tri& tri : tris)
        triSet.insert(CanonTri(tri.a, tri.b, tri.c));

    MeshOut mesh;
    mesh.contents = contents;
    mesh.material = material;
    mesh.width = width;
    mesh.height = height;
    mesh.verts.resize((size_t)width * height);
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            mesh.verts[(size_t)v * width + u] = ToMeshVert(verts[at[(size_t)v * width + u]]);
            if (u + 1 >= width || v + 1 >= height)
                continue;
            int A = at[(size_t)v * width + u];
            int B = at[(size_t)v * width + (u + 1)];
            int C = at[(size_t)(v + 1) * width + (u + 1)];
            int D = at[(size_t)(v + 1) * width + u];
            bool flag1 = triSet.count(CanonTri(A, C, B)) || triSet.count(CanonTri(C, A, D));
            bool flag0 = triSet.count(CanonTri(D, B, A)) || triSet.count(CanonTri(B, D, C));
            if (flag1 && !flag0)
                mesh.verts[(size_t)v * width + u].flag = 1;
        }
    }
    PushPieces(dest, move(mesh), meshCount, vertCount);
    return true;
}

void EmitQuad(const Quad& quad, const vector<WeldVert>& verts, const vector<Tri>& tris,
              unsigned contents, const string& material, vector<MeshOut>& dest,
              int& meshCount, int& vertCount)
{
    MeshOut mesh;
    mesh.contents = contents;
    mesh.material = material;
    mesh.width = 2;
    mesh.height = 2;
    mesh.verts.resize(4);
    mesh.verts[0] = ToMeshVert(verts[quad.v[0]]);
    mesh.verts[1] = ToMeshVert(verts[quad.v[1]]);
    mesh.verts[2] = ToMeshVert(verts[quad.v[3]]);
    mesh.verts[3] = ToMeshVert(verts[quad.v[2]]);

    unordered_set<TriKey, TriKeyHash> triSet;
    triSet.insert(CanonTri(tris[quad.tri[0]].a, tris[quad.tri[0]].b, tris[quad.tri[0]].c));
    triSet.insert(CanonTri(tris[quad.tri[1]].a, tris[quad.tri[1]].b, tris[quad.tri[1]].c));
    int A = quad.v[0], B = quad.v[1], C = quad.v[2], D = quad.v[3];
    bool flag1 = triSet.count(CanonTri(A, C, B)) || triSet.count(CanonTri(C, A, D));
    bool flag0 = triSet.count(CanonTri(D, B, A)) || triSet.count(CanonTri(B, D, C));
    if (flag1 && !flag0)
        mesh.verts[0].flag = 1;
    PushPieces(dest, move(mesh), meshCount, vertCount);
}

void EmitLooseTri(const Tri& tri, const vector<WeldVert>& verts, unsigned contents, const string& material,
                  vector<MeshOut>& dest, int& meshCount, int& vertCount, int& triMeshes)
{
    MeshOut mesh;
    mesh.contents = contents;
    mesh.material = material;
    mesh.width = 2;
    mesh.height = 2;
    mesh.verts.resize(4);
    mesh.verts[0] = ToMeshVert(verts[tri.c]);
    mesh.verts[1] = ToMeshVert(verts[tri.b]);
    mesh.verts[2] = ToMeshVert(verts[tri.a]);
    mesh.verts[3] = ToMeshVert(verts[tri.a]);
    triMeshes++;
    PushPieces(dest, move(mesh), meshCount, vertCount);
}

double EdgeLen2(const vector<WeldVert>& verts, int a, int b)
{
    Vec3 d = Sub(verts[a].p, verts[b].p);
    return Dot(d, d);
}

void LongestEdge(const Tri& tri, const vector<WeldVert>& verts, int& a, int& b)
{
    double ab = EdgeLen2(verts, tri.a, tri.b);
    double bc = EdgeLen2(verts, tri.b, tri.c);
    double ca = EdgeLen2(verts, tri.c, tri.a);
    if (ab >= bc && ab >= ca) { a = tri.a; b = tri.b; }
    else if (bc >= ca) { a = tri.b; b = tri.c; }
    else { a = tri.c; b = tri.a; }
}

bool SharedEdgeVerts(const Tri& a, const Tri& b, int& s0, int& s1)
{
    int va[3] = { a.a, a.b, a.c };
    int vb[3] = { b.a, b.b, b.c };
    int found[2];
    int count = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (va[i] != vb[j])
                continue;
            if (count < 2)
                found[count] = va[i];
            count++;
        }
    }
    if (count != 2)
        return false;
    s0 = found[0];
    s1 = found[1];
    return true;
}

bool DirectedShared(const Tri& tri, int s0, int s1, int& from, int& to, int& unique)
{
    int v[3] = { tri.a, tri.b, tri.c };
    for (int i = 0; i < 3; ++i) {
        int x = v[i];
        int y = v[(i + 1) % 3];
        int z = v[(i + 2) % 3];
        if ((x == s0 && y == s1) || (x == s1 && y == s0)) {
            from = x;
            to = y;
            unique = z;
            return true;
        }
    }
    return false;
}

struct PatchCell {
    int A, B, C, D;
};

bool LabelPatchCell(const Tri& t0, const Tri& t1, const vector<WeldVert>& verts, PatchCell& cell)
{
    int s0, s1;
    if (!SharedEdgeVerts(t0, t1, s0, s1))
        return false;
    int from0, to0, unique0, from1, to1, unique1;
    if (!DirectedShared(t0, s0, s1, from0, to0, unique0))
        return false;
    if (!DirectedShared(t1, s0, s1, from1, to1, unique1))
        return false;
    if (from1 != to0 || to1 != from0)
        return false;
    cell.D = from0;
    cell.B = to0;
    cell.A = unique0;
    cell.C = unique1;
    if (cell.A == cell.B || cell.A == cell.C || cell.A == cell.D
        || cell.B == cell.C || cell.B == cell.D || cell.C == cell.D)
        return false;

    const Vec3& a = verts[cell.A].p;
    const Vec3& b = verts[cell.B].p;
    const Vec3& d = verts[cell.D].p;
    const Vec3& c = verts[cell.C].p;
    Vec3 shared = Sub(b, d);
    double sharedLen = Len(shared);
    if (sharedLen < 1e-4)
        return false;
    return Dot(Cross(shared, Sub(a, d)), Cross(shared, Sub(c, d))) < 0.0;
}

void EdgeSt(const vector<WeldVert>& verts, int i, int j, double& ds, double& dt)
{
    ds = fabs((double)verts[i].st[0] - (double)verts[j].st[0]);
    dt = fabs((double)verts[i].st[1] - (double)verts[j].st[1]);
}

bool DiagonalInSt(const PatchCell& cell, const vector<WeldVert>& verts)
{
    double ds, dt;
    EdgeSt(verts, cell.D, cell.B, ds, dt);
    double minor = min(ds, dt);
    double major = max(ds, dt);
    if (major < 1e-7)
        return false;
    int edges[4][2] = {
        { cell.A, cell.B }, { cell.B, cell.C }, { cell.C, cell.D }, { cell.D, cell.A }
    };
    double sideMinor = 0;
    for (int e = 0; e < 4; ++e) {
        double a, b;
        EdgeSt(verts, edges[e][0], edges[e][1], a, b);
        sideMinor = max(sideMinor, min(a, b));
    }
    return minor > sideMinor * 2.0 + 1e-8;
}

bool EdgeIsLongest(const Tri& tri, const vector<WeldVert>& verts, int a, int b)
{
    double ab = EdgeLen2(verts, tri.a, tri.b);
    double bc = EdgeLen2(verts, tri.b, tri.c);
    double ca = EdgeLen2(verts, tri.c, tri.a);
    double shared = EdgeLen2(verts, a, b);
    return shared >= ab - 1e-8 && shared >= bc - 1e-8 && shared >= ca - 1e-8;
}

void EmitPatchCell(const PatchCell& cell, const vector<WeldVert>& verts, unsigned contents,
                   const string& material, vector<MeshOut>& dest, int& meshCount, int& vertCount)
{
    MeshOut mesh;
    mesh.contents = contents;
    mesh.material = material;
    mesh.width = 2;
    mesh.height = 2;
    mesh.verts.resize(4);
    mesh.verts[0] = ToMeshVert(verts[cell.A]);
    mesh.verts[1] = ToMeshVert(verts[cell.B]);
    mesh.verts[2] = ToMeshVert(verts[cell.D]);
    mesh.verts[3] = ToMeshVert(verts[cell.C]);
    PushPieces(dest, move(mesh), meshCount, vertCount);
}

int PatchVert(const PatchCell& cell, int index)
{
    int id[4] = { cell.A, cell.B, cell.C, cell.D };
    return id[index];
}

int LightmapCoordFloor(float v)
{
    return (int)floor((double)v * 512.0 - 0.5);
}

int LightmapCoordCeil(float v)
{
    return (int)ceil((double)v * 512.0 + 0.5);
}

int LightmapExtent(float mn, float mx)
{
    return LightmapCoordCeil(mx) + 1 - LightmapCoordFloor(mn);
}

// SubdivideTerrain rejects a terrain mesh that needs more than 32 lightmap splits.
bool LightmapSpanSafe(float s0, float s1, float t0, float t1)
{
    int w = LightmapExtent(s0, s1);
    int h = LightmapExtent(t0, t1);
    int m = w > h ? w : h;
    if (m <= 512)
        return true;
    int splits = (int)ceil((double)m * 0.0021186440717428923);
    return splits <= 32;
}

bool EmitStitchedPatch(const vector<PatchCell>& cells, const vector<WeldVert>& verts, unsigned contents,
                       const string& material, vector<MeshOut>& dest, int& meshCount, int& vertCount)
{
    if (cells.empty())
        return false;

    struct GridPos { int u, v; };
    struct PatchGrid {
        unordered_map<int, GridPos> vcoord;
        map<pair<int, int>, int> posVert;
        map<pair<int, int>, int> occ;

        bool Place(int cellIndex, const PatchCell& cell, const GridPos corner[4])
        {
            int minU = corner[0].u;
            int minV = corner[0].v;
            for (int k = 1; k < 4; ++k) {
                minU = min(minU, corner[k].u);
                minV = min(minV, corner[k].v);
            }
            if (occ.count({ minU, minV }))
                return false;
            for (int k = 0; k < 4; ++k) {
                int id = PatchVert(cell, k);
                auto known = vcoord.find(id);
                if (known != vcoord.end()) {
                    if (known->second.u != corner[k].u || known->second.v != corner[k].v)
                        return false;
                } else {
                    auto at = posVert.find({ corner[k].u, corner[k].v });
                    if (at != posVert.end() && at->second != id)
                        return false;
                }
            }
            for (int k = 0; k < 4; ++k) {
                int id = PatchVert(cell, k);
                vcoord[id] = corner[k];
                posVert[{ corner[k].u, corner[k].v }] = id;
            }
            occ[{ minU, minV }] = cellIndex;
            return true;
        }
    };

    map<pair<int, int>, vector<int>> edgeCells;
    for (int i = 0; i < (int)cells.size(); ++i) {
        for (int e = 0; e < 4; ++e) {
            int a = PatchVert(cells[i], e);
            int b = PatchVert(cells[i], (e + 1) % 4);
            if (a > b)
                swap(a, b);
            edgeCells[{ a, b }].push_back(i);
        }
    }

    vector<char> placed(cells.size(), 0);
    vector<char> emitted(cells.size(), 0);

    auto emitGrid = [&](PatchGrid& grid) {
        map<pair<int, int>, char> taken;

        auto cornerId = [&](int u, int v) -> int {
            auto found = grid.posVert.find({ u, v });
            return found == grid.posVert.end() ? -1 : found->second;
        };
        auto lightBounds = [&](int u0, int v0, int cw, int ch, float& s0, float& s1, float& t0, float& t1) -> bool {
            bool init = false;
            for (int v = 0; v <= ch; ++v) {
                for (int u = 0; u <= cw; ++u) {
                    int id = cornerId(u0 + u, v0 + v);
                    if (id < 0)
                        return false;
                    float s = verts[id].lm[0];
                    float t = verts[id].lm[1];
                    if (!init) {
                        s0 = s1 = s;
                        t0 = t1 = t;
                        init = true;
                    } else {
                        s0 = min(s0, s);
                        s1 = max(s1, s);
                        t0 = min(t0, t);
                        t1 = max(t1, t);
                    }
                }
            }
            return init;
        };
        auto emitRect = [&](int u0, int v0, int cw, int ch) -> bool {
            int width = cw + 1;
            int height = ch + 1;
            if (width > kMaxPatch || height > kMaxPatch)
                return false;
            MeshOut mesh;
            mesh.contents = contents;
            mesh.material = material;
            mesh.width = width;
            mesh.height = height;
            mesh.verts.assign((size_t)width * height, MeshVertOut{});
            for (int v = 0; v < height; ++v) {
                for (int u = 0; u < width; ++u) {
                    int id = cornerId(u0 + u, v0 + v);
                    if (id < 0)
                        return false;
                    mesh.verts[(size_t)v * width + u] = ToMeshVert(verts[id]);
                }
            }
            for (int cv = 0; cv < ch; ++cv) {
                for (int cu = 0; cu < cw; ++cu) {
                    auto found = grid.occ.find({ u0 + cu, v0 + cv });
                    if (found == grid.occ.end())
                        return false;
                    const PatchCell& cell = cells[found->second];
                    GridPos b = grid.vcoord[cell.B];
                    GridPos d = grid.vcoord[cell.D];
                    int bu = b.u - (u0 + cu);
                    int bv = b.v - (v0 + cv);
                    int du = d.u - (u0 + cu);
                    int dv = d.v - (v0 + cv);
                    if ((bu == 0 && bv == 0 && du == 1 && dv == 1)
                        || (bu == 1 && bv == 1 && du == 0 && dv == 0))
                        mesh.verts[(size_t)cv * width + cu].flag = 1;
                }
            }
            PushPieces(dest, move(mesh), meshCount, vertCount);
            for (int cv = 0; cv < ch; ++cv) {
                for (int cu = 0; cu < cw; ++cu) {
                    auto found = grid.occ.find({ u0 + cu, v0 + cv });
                    if (found != grid.occ.end())
                        emitted[found->second] = 1;
                }
            }
            return true;
        };

        for (const auto& item : grid.occ) {
            if (taken.count(item.first))
                continue;
            int u0 = item.first.first;
            int v0 = item.first.second;
            int maxW = 0;
            while (maxW < kMaxPatch - 1) {
                pair<int, int> key{ u0 + maxW, v0 };
                if (!grid.occ.count(key) || taken.count(key))
                    break;
                ++maxW;
            }
            int bestW = 1;
            int bestH = 1;
            for (int w = 1; w <= maxW; ++w) {
                int h = 1;
                while (h < kMaxPatch - 1) {
                    bool rowOk = true;
                    for (int u = 0; u < w && rowOk; ++u) {
                        pair<int, int> key{ u0 + u, v0 + h };
                        if (!grid.occ.count(key) || taken.count(key))
                            rowOk = false;
                    }
                    if (!rowOk)
                        break;
                    ++h;
                }
                int fw = w;
                int fh = h;
                float s0, s1, t0, t1;
                bool fit = false;
                while (fw >= 1 && fh >= 1) {
                    if (!lightBounds(u0, v0, fw, fh, s0, s1, t0, t1))
                        break;
                    if (fw == 1 && fh == 1) {
                        fit = true;
                        break;
                    }
                    if (LightmapSpanSafe(s0, s1, t0, t1)) {
                        fit = true;
                        break;
                    }
                    int se = LightmapExtent(s0, s1);
                    int te = LightmapExtent(t0, t1);
                    if (se >= te && fw > 1)
                        --fw;
                    else if (fh > 1)
                        --fh;
                    else
                        --fw;
                }
                if (!fit)
                    continue;
                int area = fw * fh;
                int bestArea = bestW * bestH;
                int span = min(fw, fh);
                int bestSpan = min(bestW, bestH);
                if (area > bestArea || (area == bestArea && (span > bestSpan || (span == bestSpan && fw > bestW)))) {
                    bestW = fw;
                    bestH = fh;
                }
            }
            if (!emitRect(u0, v0, bestW, bestH)) {
                for (int cv = 0; cv < bestH; ++cv) {
                    for (int cu = 0; cu < bestW; ++cu) {
                        auto found = grid.occ.find({ u0 + cu, v0 + cv });
                        if (found != grid.occ.end() && !emitted[found->second])
                            EmitPatchCell(cells[found->second], verts, contents, material, dest, meshCount, vertCount);
                        if (found != grid.occ.end())
                            emitted[found->second] = 1;
                    }
                }
            }
            for (int cv = 0; cv < bestH; ++cv)
                for (int cu = 0; cu < bestW; ++cu)
                    taken[{ u0 + cu, v0 + cv }] = 1;
        }
    };

    for (int seed = 0; seed < (int)cells.size(); ++seed) {
        if (placed[seed])
            continue;
        PatchGrid grid;
        GridPos start[4] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
        if (!grid.Place(seed, cells[seed], start))
            continue;
        placed[seed] = 1;
        queue<int> pending;
        pending.push(seed);
        while (!pending.empty()) {
            int ci = pending.front();
            pending.pop();
            const PatchCell& cur = cells[ci];
            for (int e = 0; e < 4; ++e) {
                int i = PatchVert(cur, e);
                int j = PatchVert(cur, (e + 1) % 4);
                GridPos pi = grid.vcoord[i];
                GridPos pj = grid.vcoord[j];
                int lo = i < j ? i : j;
                int hi = i < j ? j : i;
                auto found = edgeCells.find({ lo, hi });
                if (found == edgeCells.end())
                    continue;
                for (int ni : found->second) {
                    if (ni == ci || placed[ni])
                        continue;
                    const PatchCell& nb = cells[ni];
                    int k = -1;
                    for (int kk = 0; kk < 4; ++kk) {
                        if (PatchVert(nb, kk) == j && PatchVert(nb, (kk + 1) % 4) == i)
                            k = kk;
                    }
                    if (k < 0)
                        continue;
                    int du = pj.u - pi.u;
                    int dv = pj.v - pi.v;
                    int ru = dv;
                    int rv = -du;
                    GridPos nc[4];
                    nc[k] = pj;
                    nc[(k + 1) % 4] = pi;
                    nc[(k + 2) % 4] = { pi.u + ru, pi.v + rv };
                    nc[(k + 3) % 4] = { pj.u + ru, pj.v + rv };
                    if (grid.Place(ni, nb, nc)) {
                        placed[ni] = 1;
                        pending.push(ni);
                    }
                }
            }
        }
        emitGrid(grid);
    }

    for (int i = 0; i < (int)cells.size(); ++i) {
        if (!emitted[i])
            EmitPatchCell(cells[i], verts, contents, material, dest, meshCount, vertCount);
    }
    return true;
}

bool RecoverClosedBrushes(const vector<WeldVert>& verts, const vector<Tri>& tris, unsigned contents,
                          const string& material, vector<BrushOut>& brushes, vector<char>& used);

void BuildMeshes(const vector<WeldVert>& verts, const vector<Tri>& tris, unsigned contents,
                 const string& material, vector<MeshOut>& dest, int& meshCount, int& vertCount, int& triMeshes)
{
    if (tris.empty())
        return;

    map<pair<int, int>, vector<int>> edges;
    for (int i = 0; i < (int)tris.size(); ++i) {
        int e[3][2] = { { tris[i].a, tris[i].b }, { tris[i].b, tris[i].c }, { tris[i].c, tris[i].a } };
        for (int k = 0; k < 3; ++k) {
            int a = e[k][0], b = e[k][1];
            if (a > b)
                swap(a, b);
            edges[{ a, b }].push_back(i);
        }
    }

    struct Candidate {
        int t0, t1;
        PatchCell cell;
        double score;
        bool st;
    };
    vector<Candidate> stCells;
    vector<Candidate> longCells;
    for (const auto& entry : edges) {
        if (entry.second.size() != 2)
            continue;
        int t0 = entry.second[0];
        int t1 = entry.second[1];
        PatchCell cell;
        if (!LabelPatchCell(tris[t0], tris[t1], verts, cell))
            continue;
        Candidate candidate;
        candidate.t0 = t0;
        candidate.t1 = t1;
        candidate.cell = cell;
        candidate.st = DiagonalInSt(cell, verts);
        Vec3 predicted = Add(verts[cell.B].p, Sub(verts[cell.D].p, verts[cell.A].p));
        candidate.score = Len(Sub(verts[cell.C].p, predicted));
        if (candidate.st)
            stCells.push_back(candidate);
        else if (EdgeIsLongest(tris[t0], verts, entry.first.first, entry.first.second)
                 && EdgeIsLongest(tris[t1], verts, entry.first.first, entry.first.second))
            longCells.push_back(candidate);
    }
    sort(stCells.begin(), stCells.end(), [](const Candidate& a, const Candidate& b) { return a.score < b.score; });
    sort(longCells.begin(), longCells.end(), [](const Candidate& a, const Candidate& b) { return a.score < b.score; });

    vector<char> used(tris.size(), 0);
    vector<PatchCell> cells;
    auto take = [&](const vector<Candidate>& list) {
        for (const Candidate& candidate : list) {
            if (used[candidate.t0] || used[candidate.t1])
                continue;
            used[candidate.t0] = used[candidate.t1] = 1;
            cells.push_back(candidate.cell);
        }
    };
    take(stCells);
    take(longCells);
    EmitStitchedPatch(cells, verts, contents, material, dest, meshCount, vertCount);
    for (int i = 0; i < (int)tris.size(); ++i) {
        if (!used[i])
            EmitLooseTri(tris[i], verts, contents, material, dest, meshCount, vertCount, triMeshes);
    }
}

string ResolveLayered(const string& name, const BspFile& bsp)
{
    if (name.size() < 2 || name[0] != '*')
        return name;
    int index = 0;
    size_t i = 1;
    if (!isdigit((unsigned char)name[i]))
        return name;
    while (i < name.size() && isdigit((unsigned char)name[i])) {
        index = index * 10 + (name[i] - '0');
        ++i;
    }
    if (index < 0 || index >= (int)bsp.materials.size())
        return name;
    string inner = MaterialName(bsp.materials[index]);
    if (inner.empty() || inner[0] == '*')
        return name;
    return inner;
}

struct BucketKey {
    int nameId;
    unsigned contents;
    bool operator==(const BucketKey& o) const { return nameId == o.nameId && contents == o.contents; }
};

struct BucketHash {
    size_t operator()(const BucketKey& k) const
    {
        return ((size_t)k.nameId * 73856093u) ^ (k.contents * 19349663u);
    }
};

struct PosKey {
    int nameId;
    int p[9];
    bool operator==(const PosKey& o) const
    {
        if (nameId != o.nameId)
            return false;
        for (int i = 0; i < 9; ++i)
            if (p[i] != o.p[i])
                return false;
        return true;
    }
};

struct PosHash {
    size_t operator()(const PosKey& k) const
    {
        size_t h = (size_t)k.nameId * 73856093u;
        for (int i = 0; i < 9; ++i)
            h ^= (size_t)k.p[i] * (19349663u + (unsigned)i * 17u);
        return h;
    }
};

struct CollKey {
    int p[9];
    bool operator==(const CollKey& o) const
    {
        for (int i = 0; i < 9; ++i)
            if (p[i] != o.p[i])
                return false;
        return true;
    }
};

struct CollKeyHash {
    size_t operator()(const CollKey& k) const
    {
        size_t h = 0;
        for (int i = 0; i < 9; ++i)
            h ^= (size_t)k.p[i] * (73856093u + (unsigned)i * 17u);
        return h;
    }
};

int QuantThousand(float v)
{
    return (int)llround((double)v * 1000.0);
}

CollKey MakeCollKey(const float a[3], const float b[3], const float c[3])
{
    struct QPoint {
        int x, y, z;
        bool operator<(const QPoint& o) const
        {
            if (x != o.x) return x < o.x;
            if (y != o.y) return y < o.y;
            return z < o.z;
        }
    };
    QPoint pts[3] = {
        { QuantThousand(a[0]), QuantThousand(a[1]), QuantThousand(a[2]) },
        { QuantThousand(b[0]), QuantThousand(b[1]), QuantThousand(b[2]) },
        { QuantThousand(c[0]), QuantThousand(c[1]), QuantThousand(c[2]) }
    };
    sort(pts, pts + 3);
    CollKey key{};
    for (int i = 0; i < 3; ++i) {
        key.p[i * 3 + 0] = pts[i].x;
        key.p[i * 3 + 1] = pts[i].y;
        key.p[i * 3 + 2] = pts[i].z;
    }
    return key;
}

struct CollEdgeKey {
    int a[3];
    int b[3];
    bool operator==(const CollEdgeKey& o) const
    {
        return a[0] == o.a[0] && a[1] == o.a[1] && a[2] == o.a[2]
            && b[0] == o.b[0] && b[1] == o.b[1] && b[2] == o.b[2];
    }
};

struct CollEdgeHash {
    size_t operator()(const CollEdgeKey& k) const
    {
        size_t h = 0;
        for (int i = 0; i < 3; ++i) {
            h ^= (size_t)k.a[i] * (73856093u + (unsigned)i * 17u);
            h ^= (size_t)k.b[i] * (19349663u + (unsigned)i * 17u);
        }
        return h;
    }
};

struct CollThird {
    float p[3];
    unsigned contents;
    bool known;
};

struct CollisionSet {
    unordered_set<CollKey, CollKeyHash> present;
    unordered_map<CollKey, unsigned, CollKeyHash> contents;
    unordered_map<CollEdgeKey, vector<CollThird>, CollEdgeHash> edges;
};

constexpr double kSnapVertDist2 = 1.9083023e-06;

CollisionSet BuildCollisionContents(const BspFile& bsp)
{
    CollisionSet set;
    unordered_map<int, unsigned> partContents;
    unordered_set<int> partBad;
    unordered_set<int> seenAabb;
    for (const BspLeafColl& leaf : bsp.leafColl) {
        if (leaf.collCount <= 0)
            continue;
        vector<int> stack;
        stack.reserve(static_cast<size_t>(leaf.collCount));
        for (int i = 0; i < leaf.collCount; ++i)
            stack.push_back(leaf.firstCollAabb + i);
        for (size_t qi = 0; qi < stack.size(); ++qi) {
            int idx = stack[qi];
            if (idx < 0 || idx >= (int)bsp.collisionAabbs.size())
                continue;
            if (!seenAabb.insert(idx).second)
                continue;
            const BspCollisionAabb& aabb = bsp.collisionAabbs[idx];
            if (aabb.childCount > 0) {
                for (int c = 0; c < (int)aabb.childCount; ++c)
                    stack.push_back(aabb.u + c);
                continue;
            }
            if (aabb.u < 0 || aabb.u >= (int)bsp.collisionParts.size())
                continue;
            if (aabb.materialIndex >= bsp.materials.size())
                continue;
            if (partBad.count(aabb.u))
                continue;
            unsigned cont = bsp.materials[aabb.materialIndex].contentFlags;
            auto found = partContents.find(aabb.u);
            if (found == partContents.end())
                partContents.emplace(aabb.u, cont);
            else if (found->second != cont) {
                partContents.erase(found);
                partBad.insert(aabb.u);
            }
        }
    }

    for (const BspCollisionPart& partition : bsp.collisionParts) {
        for (int t = 0; t < (int)partition.triCount; ++t) {
            int triIndex = partition.firstTri + t;
            if (triIndex < 0 || triIndex >= (int)bsp.collisionTris.size())
                break;
            const BspCollisionTri& tri = bsp.collisionTris[triIndex];
            if (tri.v[0] >= bsp.collisionVerts.size() || tri.v[1] >= bsp.collisionVerts.size()
                || tri.v[2] >= bsp.collisionVerts.size())
                continue;
            set.present.insert(MakeCollKey(bsp.collisionVerts[tri.v[0]].xyz,
                                           bsp.collisionVerts[tri.v[1]].xyz,
                                           bsp.collisionVerts[tri.v[2]].xyz));
        }
    }

    unordered_set<CollKey, CollKeyHash> ambiguous;
    for (const auto& item : partContents) {
        const BspCollisionPart& partition = bsp.collisionParts[item.first];
        for (int t = 0; t < (int)partition.triCount; ++t) {
            int triIndex = partition.firstTri + t;
            if (triIndex < 0 || triIndex >= (int)bsp.collisionTris.size())
                break;
            const BspCollisionTri& tri = bsp.collisionTris[triIndex];
            if (tri.v[0] >= bsp.collisionVerts.size() || tri.v[1] >= bsp.collisionVerts.size()
                || tri.v[2] >= bsp.collisionVerts.size())
                continue;
            CollKey key = MakeCollKey(bsp.collisionVerts[tri.v[0]].xyz,
                                      bsp.collisionVerts[tri.v[1]].xyz,
                                      bsp.collisionVerts[tri.v[2]].xyz);
            if (ambiguous.count(key))
                continue;
            auto found = set.contents.find(key);
            if (found == set.contents.end())
                set.contents.emplace(key, item.second);
            else if (found->second != item.second) {
                set.contents.erase(found);
                ambiguous.insert(key);
            }
        }
    }

    auto addEdge = [&](const float p0[3], const float p1[3], const float third[3], unsigned cont, bool known) {
        int q0[3] = { QuantThousand(p0[0]), QuantThousand(p0[1]), QuantThousand(p0[2]) };
        int q1[3] = { QuantThousand(p1[0]), QuantThousand(p1[1]), QuantThousand(p1[2]) };
        bool swapEnds = false;
        for (int i = 0; i < 3; ++i) {
            if (q0[i] == q1[i])
                continue;
            swapEnds = q0[i] > q1[i];
            break;
        }
        CollEdgeKey edge{};
        memcpy(edge.a, swapEnds ? q1 : q0, sizeof(edge.a));
        memcpy(edge.b, swapEnds ? q0 : q1, sizeof(edge.b));
        CollThird ref{};
        memcpy(ref.p, third, sizeof(ref.p));
        ref.contents = cont;
        ref.known = known;
        set.edges[edge].push_back(ref);
    };
    for (const BspCollisionPart& partition : bsp.collisionParts) {
        for (int t = 0; t < (int)partition.triCount; ++t) {
            int triIndex = partition.firstTri + t;
            if (triIndex < 0 || triIndex >= (int)bsp.collisionTris.size())
                break;
            const BspCollisionTri& tri = bsp.collisionTris[triIndex];
            if (tri.v[0] >= bsp.collisionVerts.size() || tri.v[1] >= bsp.collisionVerts.size()
                || tri.v[2] >= bsp.collisionVerts.size())
                continue;
            const float* xyz[3] = {
                bsp.collisionVerts[tri.v[0]].xyz,
                bsp.collisionVerts[tri.v[1]].xyz,
                bsp.collisionVerts[tri.v[2]].xyz
            };
            CollKey key = MakeCollKey(xyz[0], xyz[1], xyz[2]);
            auto found = set.contents.find(key);
            bool known = found != set.contents.end();
            unsigned cont = known ? found->second : 0;
            addEdge(xyz[0], xyz[1], xyz[2], cont, known);
            addEdge(xyz[1], xyz[2], xyz[0], cont, known);
            addEdge(xyz[2], xyz[0], xyz[1], cont, known);
        }
    }
    return set;
}

bool CollisionContentsForTriangle(const CollisionSet& set, const float a[3], const float b[3], const float c[3],
                                  bool& known, unsigned& contents)
{
    CollKey key = MakeCollKey(a, b, c);
    auto exact = set.contents.find(key);
    if (exact != set.contents.end()) {
        known = true;
        contents = exact->second;
        return true;
    }
    if (set.present.count(key)) {
        known = false;
        contents = 0;
        return true;
    }

    const float* pts[3] = { a, b, c };
    int q[3][3];
    for (int i = 0; i < 3; ++i) {
        q[i][0] = QuantThousand(pts[i][0]);
        q[i][1] = QuantThousand(pts[i][1]);
        q[i][2] = QuantThousand(pts[i][2]);
    }
    bool found = false;
    double best = 0;
    bool bestKnown = false;
    unsigned bestContents = 0;
    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        bool swapEnds = false;
        for (int n = 0; n < 3; ++n) {
            if (q[i][n] == q[j][n])
                continue;
            swapEnds = q[i][n] > q[j][n];
            break;
        }
        CollEdgeKey edge{};
        memcpy(edge.a, swapEnds ? q[j] : q[i], sizeof(edge.a));
        memcpy(edge.b, swapEnds ? q[i] : q[j], sizeof(edge.b));
        auto refs = set.edges.find(edge);
        if (refs == set.edges.end())
            continue;
        for (const CollThird& ref : refs->second) {
            double dx = (double)ref.p[0] - (double)pts[k][0];
            double dy = (double)ref.p[1] - (double)pts[k][1];
            double dz = (double)ref.p[2] - (double)pts[k][2];
            double dist2 = dx * dx + dy * dy + dz * dz;
            if (dist2 > kSnapVertDist2)
                continue;
            if (found && dist2 > best)
                continue;
            if (found && dist2 == best && bestKnown)
                continue;
            found = true;
            best = dist2;
            bestKnown = ref.known;
            bestContents = ref.contents;
        }
    }
    if (!found)
        return false;
    known = bestKnown;
    contents = bestContents;
    return true;
}

bool PlaneFromFace(const Vec3& a, const Vec3& b, const Vec3& c, PlaneD& out)
{
    Vec3 n = Cross(Sub(b, a), Sub(c, a));
    double len = Len(n);
    if (len < 1e-8)
        return false;
    n = Mul(n, 1.0 / len);
    out.n[0] = n.x;
    out.n[1] = n.y;
    out.n[2] = n.z;
    out.dist = Dot(n, a);
    return true;
}

bool SamePortalPlane(const PlaneD& a, const PlaneD& b)
{
    double dot = a.n[0] * b.n[0] + a.n[1] * b.n[1] + a.n[2] * b.n[2];
    if (dot > 0.999 && fabs(a.dist - b.dist) <= 0.2)
        return true;
    if (dot < -0.999 && fabs(a.dist + b.dist) <= 0.2)
        return true;
    return false;
}

vector<Vec3> HullOnPlane(const vector<Vec3>& points, const Vec3& normal)
{
    Vec3 up = fabs(normal.z) < 0.9 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
    Vec3 tangent = Cross(normal, up);
    double tlen = Len(tangent);
    if (tlen < 1e-8)
        return {};
    tangent = Mul(tangent, 1.0 / tlen);
    Vec3 bitangent = Cross(normal, tangent);

    struct Q {
        double u, v;
        Vec3 p;
    };
    vector<Q> uniq;
    for (const Vec3& p : points) {
        bool have = false;
        for (const Q& q : uniq) {
            if (fabs(q.p.x - p.x) <= 0.05 && fabs(q.p.y - p.y) <= 0.05 && fabs(q.p.z - p.z) <= 0.05) {
                have = true;
                break;
            }
        }
        if (!have)
            uniq.push_back({ Dot(p, tangent), Dot(p, bitangent), p });
    }
    if (uniq.size() < 3)
        return {};
    sort(uniq.begin(), uniq.end(), [](const Q& a, const Q& b) {
        if (a.u != b.u)
            return a.u < b.u;
        return a.v < b.v;
    });
    auto turn = [](const Q& o, const Q& a, const Q& c) {
        return (a.u - o.u) * (c.v - o.v) - (a.v - o.v) * (c.u - o.u);
    };
    vector<Q> hull;
    for (const Q& p : uniq) {
        while (hull.size() >= 2 && turn(hull[hull.size() - 2], hull.back(), p) <= 1e-6)
            hull.pop_back();
        hull.push_back(p);
    }
    size_t lower = hull.size();
    for (int i = (int)uniq.size() - 2; i >= 0; --i) {
        const Q& p = uniq[i];
        while (hull.size() > lower && turn(hull[hull.size() - 2], hull.back(), p) <= 1e-6)
            hull.pop_back();
        hull.push_back(p);
    }
    if (!hull.empty())
        hull.pop_back();
    vector<Vec3> out;
    for (const Q& q : hull)
        out.push_back(q.p);
    return out.size() >= 3 ? out : vector<Vec3>{};
}

void ApplyPortalShapes(const BspFile& bsp, vector<BrushOut>& brushes, vector<SideGeom>& allSides,
                       vector<int>& brushSideStart)
{
    if (bsp.portals.empty() || brushes.empty())
        return;

    struct LumpFace {
        PlaneD plane;
        vector<Vec3> winding;
        Vec3 centroid;
        int owner = -1;
        int ownerSide = -1;
        double score = 1e30;
    };
    vector<LumpFace> lumps;
    for (const BspPortal& portal : bsp.portals) {
        if (portal.planeIndex < 0 || portal.planeIndex >= (int)bsp.planes.size())
            continue;
        int count = portal.vertCount;
        if (count < 3 || count > 64)
            continue;
        if (portal.firstVert < 0 || portal.firstVert > (int)bsp.portalVerts.size() - count)
            continue;
        const BspPlane& src = bsp.planes[portal.planeIndex];
        Vec3 n{ src.normal[0], src.normal[1], src.normal[2] };
        double len = Len(n);
        if (len < 1e-8)
            continue;
        n = Mul(n, 1.0 / len);
        LumpFace face;
        face.plane.n[0] = n.x;
        face.plane.n[1] = n.y;
        face.plane.n[2] = n.z;
        face.plane.dist = (src.normal[0] * src.dist + src.normal[1] * src.dist + src.normal[2] * src.dist) / (len * len);
        face.plane.dist = src.dist / len;
        face.centroid = { 0, 0, 0 };
        for (int i = 0; i < count; ++i) {
            const float* p = bsp.portalVerts[portal.firstVert + i].xyz;
            Vec3 v{ p[0], p[1], p[2] };
            face.winding.push_back(v);
            face.centroid = Add(face.centroid, v);
        }
        face.centroid = Mul(face.centroid, 1.0 / count);
        lumps.push_back(move(face));
    }
    if (lumps.empty())
        return;

    for (int b = 0; b < (int)brushes.size(); ++b) {
        int begin = brushSideStart[b];
        int end = brushSideStart[b + 1];
        for (int s = begin; s < end; ++s) {
            if (!SideIsPortal(allSides[s].surfaceFlags))
                continue;
            for (LumpFace& lump : lumps) {
                if (!SamePortalPlane(allSides[s].plane, lump.plane))
                    continue;
                Vec3 mid{ 0, 0, 0 };
                if (!allSides[s].winding.empty()) {
                    for (const Vec3& p : allSides[s].winding)
                        mid = Add(mid, p);
                    mid = Mul(mid, 1.0 / (double)allSides[s].winding.size());
                }
                double dist = Len(Sub(lump.centroid, mid));
                if (dist >= lump.score)
                    continue;
                lump.score = dist;
                lump.owner = b;
                lump.ownerSide = s;
            }
        }
    }

    auto sameSpot = [](const LumpFace& a, const LumpFace& b) {
        if (a.winding.size() != b.winding.size() || !SamePortalPlane(a.plane, b.plane))
            return false;
        for (const Vec3& p : a.winding) {
            bool hit = false;
            for (const Vec3& q : b.winding) {
                if (fabs(p.x - q.x) <= 0.5 && fabs(p.y - q.y) <= 0.5 && fabs(p.z - q.z) <= 0.5) {
                    hit = true;
                    break;
                }
            }
            if (!hit)
                return false;
        }
        return true;
    };

    vector<char> kill(brushes.size(), 0);
    vector<LumpFace> kept;
    int sideEnd = brushSideStart.back();
    for (const LumpFace& lump : lumps) {
        if (lump.owner < 0 || lump.ownerSide < 0 || lump.winding.size() < 3)
            continue;
        bool copy = false;
        for (const LumpFace& prev : kept) {
            if (sameSpot(prev, lump)) {
                copy = true;
                break;
            }
        }
        if (copy)
            continue;
        kept.push_back(lump);

        int b = lump.owner;
        int bestSide = lump.ownerSide;
        int begin = brushSideStart[b];
        int end = brushSideStart[b + 1];
        int portalFace = allSides[bestSide].faceIndex;
        if (portalFace < 0 || portalFace >= (int)brushes[b].faces.size())
            continue;
        string portalMat = brushes[b].faces[portalFace].material;
        string skirtMat = portalMat;
        for (int s = begin; s < end; ++s) {
            if (s == bestSide || allSides[s].faceIndex < 0
                || allSides[s].faceIndex >= (int)brushes[b].faces.size())
                continue;
            skirtMat = brushes[b].faces[allSides[s].faceIndex].material;
            break;
        }

        Vec3 axis{ lump.plane.n[0], lump.plane.n[1], lump.plane.n[2] };
        double minD = 0;
        double maxD = 0;
        for (int s = begin; s < end; ++s) {
            for (const Vec3& p : allSides[s].winding) {
                double d = lump.plane.Dist(p);
                minD = min(minD, d);
                maxD = max(maxD, d);
            }
        }
        double thickness = max(-minD, maxD);
        if (thickness < 0.5)
            thickness = 0.5;
        Vec3 extrude = Mul(axis, (-minD >= maxD) ? -thickness : thickness);

        vector<Vec3> w = lump.winding;
        for (Vec3& p : w) {
            double d = lump.plane.Dist(p);
            p = Sub(p, Mul(axis, d));
        }
        if (Dot(axis, extrude) < 0)
            reverse(w.begin(), w.end());

        BrushOut neu;
        neu.portal = true;
        neu.contents = brushes[b].contents;
        auto put = [&](const Vec3& a, const Vec3& mid, const Vec3& c, const string& material) {
            FaceOut face;
            face.material = material;
            face.p[0] = a;
            face.p[1] = mid;
            face.p[2] = c;
            neu.faces.push_back(face);
        };
        put(w[2], w[1], w[0], portalMat);
        for (int i = 0; i < (int)w.size(); ++i)
            put(w[i], w[(i + 1) % w.size()], Add(w[i], extrude), skirtMat);
        put(Add(w[0], extrude), Add(w[1], extrude), Add(w[2], extrude), skirtMat);
        brushes.push_back(move(neu));
        brushSideStart.push_back(sideEnd);
        kill[b] = 1;
    }

    for (int b = 0; b < (int)kill.size(); ++b) {
        if (kill[b])
            brushes[b].faces.clear();
    }
}

struct StOffset { int s = 0; int t = 0; };

vector<StOffset> TerrainSoupOffsets(const BspFile& bsp, int modelIndex, const Vec3& origin,
                                    const vector<int>& matName, const Names& names)
{
    struct Anchor {
        int soup;
        int nameId;
        Vec3 p;
        float st[2];
        float lm[2];
        unsigned char color[4];
    };
    struct Cell {
        int x, y, z;
        bool operator==(const Cell& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct CellHash {
        size_t operator()(const Cell& k) const
        {
            return ((size_t)k.x * 73856093u) ^ ((size_t)k.y * 19349663u) ^ ((size_t)k.z * 83492791u);
        }
    };
    vector<Anchor> anchors;
    unordered_map<Cell, vector<int>, CellHash> grid;
    int soupCount = 0;
    auto cellOf = [](const Vec3& p) {
        return Cell{ (int)floor(p.x / kPatchWeld), (int)floor(p.y / kPatchWeld), (int)floor(p.z / kPatchWeld) };
    };
    auto integerDelta = [](float a, float b, int& d) {
        double raw = (double)a - (double)b;
        d = (int)llround(raw);
        return fabs(raw - (double)d) <= kPatchSt;
    };

    ForSoups(bsp, modelIndex, [&](int type, const BspTriSoup& soup) {
        int serial = soupCount++;
        if (soup.materialIndex >= bsp.materials.size() || soup.indexCount < 3)
            return;
        int nameId = MatchNameId(bsp, names, matName, soup.materialIndex);
        if (nameId < 0)
            return;
        unordered_set<int> seen;
        for (int i = 0; i < soup.indexCount; ++i) {
            int iv = ResolveVert(soup, bsp.indices[type], i, (int)bsp.verts[type].size());
            if (iv < 0 || !seen.insert(iv).second)
                continue;
            const BspDrawVert& vert = bsp.verts[type][iv];
            Anchor a;
            a.soup = serial;
            a.nameId = nameId;
            a.p = FromVert(vert, origin);
            a.st[0] = vert.texCoord[0];
            a.st[1] = vert.texCoord[1];
            a.lm[0] = vert.lmapCoord[0];
            a.lm[1] = vert.lmapCoord[1];
            memcpy(a.color, vert.color, 4);
            int id = (int)anchors.size();
            anchors.push_back(a);
            grid[cellOf(a.p)].push_back(id);
        }
    });

    struct Acc { int ds = 0; int dt = 0; bool bad = false; bool set = false; };
    map<pair<int, int>, Acc> pairs;
    for (int ia = 0; ia < (int)anchors.size(); ++ia) {
        Cell c = cellOf(anchors[ia].p);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto found = grid.find(Cell{ c.x + dx, c.y + dy, c.z + dz });
                    if (found == grid.end())
                        continue;
                    for (int ib : found->second) {
                        if (ib <= ia)
                            continue;
                        const Anchor& a = anchors[ia];
                        const Anchor& b = anchors[ib];
                        if (a.soup == b.soup || a.nameId != b.nameId)
                            continue;
                        if (fabs(a.p.x - b.p.x) > kPatchWeld || fabs(a.p.y - b.p.y) > kPatchWeld
                            || fabs(a.p.z - b.p.z) > kPatchWeld)
                            continue;
                        if (fabs((double)a.lm[0] - (double)b.lm[0]) > kPatchSt
                            || fabs((double)a.lm[1] - (double)b.lm[1]) > kPatchSt)
                            continue;
                        if (a.color[0] != b.color[0] || a.color[1] != b.color[1]
                            || a.color[2] != b.color[2] || a.color[3] != b.color[3])
                            continue;
                        int lo = a.soup < b.soup ? ia : ib;
                        int hi = a.soup < b.soup ? ib : ia;
                        const Anchor& u = anchors[lo];
                        const Anchor& v = anchors[hi];
                        int ds = 0, dt = 0;
                        bool ok = integerDelta(u.st[0], v.st[0], ds) && integerDelta(u.st[1], v.st[1], dt);
                        Acc& acc = pairs[{ u.soup, v.soup }];
                        if (!ok) {
                            acc.bad = true;
                            continue;
                        }
                        if (!acc.set) {
                            acc.ds = ds;
                            acc.dt = dt;
                            acc.set = true;
                        } else if (acc.ds != ds || acc.dt != dt) {
                            acc.bad = true;
                        }
                    }
                }
            }
        }
    }

    struct Rel { int other; int ds; int dt; };
    vector<vector<Rel>> links((size_t)soupCount);
    for (const auto& item : pairs) {
        if (!item.second.set || item.second.bad)
            continue;
        int lo = item.first.first;
        int hi = item.first.second;
        links[lo].push_back({ hi, item.second.ds, item.second.dt });
        links[hi].push_back({ lo, -item.second.ds, -item.second.dt });
    }

    vector<StOffset> off((size_t)soupCount);
    vector<char> known((size_t)soupCount, 0);
    for (int s = 0; s < soupCount; ++s) {
        if (known[s])
            continue;
        known[s] = 1;
        vector<int> comp;
        comp.push_back(s);
        bool bad = false;
        queue<int> pending;
        pending.push(s);
        while (!pending.empty()) {
            int cur = pending.front();
            pending.pop();
            for (const Rel& rel : links[cur]) {
                int es = off[cur].s + rel.ds;
                int et = off[cur].t + rel.dt;
                if (known[rel.other]) {
                    if (off[rel.other].s != es || off[rel.other].t != et)
                        bad = true;
                    continue;
                }
                known[rel.other] = 1;
                off[rel.other].s = es;
                off[rel.other].t = et;
                comp.push_back(rel.other);
                pending.push(rel.other);
            }
        }
        if (bad) {
            for (int i : comp)
                off[i] = {};
        }
    }
    return off;
}

ModelGeo BuildModel(const BspFile& bsp, int modelIndex, const Vec3& origin, const vector<int>& matName,
                    const Names& names, int caulk, const CollisionSet* collision,
                    int& dropped, int& meshCount, int& vertCount, int& triMeshes)
{
    ModelGeo geo;
    const BspModel& model = bsp.models[modelIndex];
    if (model.firstBrush < 0 || model.numBrushes < 0
        || model.firstBrush + model.numBrushes > (int)bsp.brushes.size())
        throw runtime_error("model brush range is outside the bsp");

    vector<int> sideStart(bsp.brushes.size() + 1, 0);
    for (int i = 0; i < (int)bsp.brushes.size(); ++i)
        sideStart[i + 1] = sideStart[i] + bsp.brushes[i].numSides;

    vector<int> edgeStart(bsp.sides.size() + 1, 0);
    if (bsp.edgeCounts.size() == bsp.sides.size()) {
        for (int i = 0; i < (int)bsp.sides.size(); ++i)
            edgeStart[i + 1] = edgeStart[i] + bsp.edgeCounts[i];
    }

    vector<SideGeom> allSides;
    vector<int> brushSideStart;
    brushSideStart.push_back(0);
    for (int b = 0; b < model.numBrushes; ++b) {
        int brushIndex = model.firstBrush + b;
        BrushOut brush;
        BuiltSides built;
        if (!BuildBrush(bsp, brushIndex, sideStart[brushIndex], origin, matName, names, caulk, edgeStart, brush, built)) {
            dropped++;
            continue;
        }
        int ordinal = (int)geo.brushes.size();
        for (SideGeom& side : built.sides)
            side.brushIndex = ordinal;
        for (SideGeom& side : built.sides)
            allSides.push_back(move(side));
        brushSideStart.push_back((int)allSides.size());
        geo.brushes.push_back(move(brush));
    }

    for (int b = 0; b < (int)geo.brushes.size(); ++b) {
        int begin = brushSideStart[b];
        int end = brushSideStart[b + 1];
        for (int i = begin; i < end; ++i) {
            SideGeom& side = allSides[i];
            auto resolveId = [&](int id) {
                if (id < 0 || id >= (int)names.list.size())
                    return id;
                string resolved = ResolveLayered(names.list[id], bsp);
                auto found = names.lookup.find(resolved);
                return found == names.lookup.end() ? id : found->second;
            };
            side.drawNameId = resolveId(side.drawNameId);
            side.nameId = resolveId(side.nameId);
            if (side.faceIndex >= 0 && side.faceIndex < (int)geo.brushes[b].faces.size()
                && side.nameId >= 0 && side.nameId < (int)names.list.size())
                geo.brushes[b].faces[side.faceIndex].material = names.list[side.nameId];
        }
    }
    ApplyPortalShapes(bsp, geo.brushes, allSides, brushSideStart);
    FitAllBrushTextures(bsp, modelIndex, origin, matName, names, allSides, geo.brushes, brushSideStart);

    vector<SideGeom> live;
    for (SideGeom& side : allSides)
        if (!side.winding.empty())
            live.push_back(move(side));

    SideGrid sideGrid;
    sideGrid.build(live);
    vector<vector<int>> brushSides(geo.brushes.size());
    for (int i = 0; i < (int)live.size(); ++i) {
        int brush = live[i].brushIndex;
        if (brush >= 0 && brush < (int)brushSides.size())
            brushSides[brush].push_back(i);
    }

    struct WeldCell {
        int x, y, z;
        bool operator==(const WeldCell& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct WeldCellHash {
        size_t operator()(const WeldCell& k) const
        {
            return ((size_t)k.x * 73856093u) ^ ((size_t)k.y * 19349663u) ^ ((size_t)k.z * 83492791u);
        }
    };
    vector<WeldVert> weld;
    unordered_map<WeldCell, vector<int>, WeldCellHash> weldGrid;
    unordered_map<BucketKey, vector<Tri>, BucketHash> buckets;
    unordered_set<PosKey, PosHash> seenTris;
    const vector<StOffset> soupOff = TerrainSoupOffsets(bsp, modelIndex, origin, matName, names);
    int soupSerial = 0;

    auto weldCell = [](const Vec3& p) {
        return WeldCell{ (int)floor(p.x / kPatchWeld), (int)floor(p.y / kPatchWeld), (int)floor(p.z / kPatchWeld) };
    };
    auto weldPoint = [&](const BspDrawVert& vert, StOffset off) -> int {
        Vec3 p = FromVert(vert, origin);
        double st0 = (double)vert.texCoord[0] + off.s;
        double st1 = (double)vert.texCoord[1] + off.t;
        WeldCell cell = weldCell(p);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto found = weldGrid.find({ cell.x + dx, cell.y + dy, cell.z + dz });
                    if (found == weldGrid.end())
                        continue;
                    for (int id : found->second) {
                        const WeldVert& other = weld[id];
                        if (fabs(other.p.x - p.x) > kPatchWeld || fabs(other.p.y - p.y) > kPatchWeld
                            || fabs(other.p.z - p.z) > kPatchWeld)
                            continue;
                        if (fabs((double)other.st[0] - st0) > kPatchSt
                            || fabs((double)other.st[1] - st1) > kPatchSt)
                            continue;
                        if (fabs((double)other.lm[0] - (double)vert.lmapCoord[0]) > kPatchSt
                            || fabs((double)other.lm[1] - (double)vert.lmapCoord[1]) > kPatchSt)
                            continue;
                        if (other.color[0] != vert.color[0] || other.color[1] != vert.color[1]
                            || other.color[2] != vert.color[2] || other.color[3] != vert.color[3])
                            continue;
                        return id;
                    }
                }
            }
        }
        WeldVert out;
        out.p = p;
        out.st[0] = (float)st0;
        out.st[1] = (float)st1;
        out.lm[0] = vert.lmapCoord[0];
        out.lm[1] = vert.lmapCoord[1];
        out.n[0] = vert.normal[0];
        out.n[1] = vert.normal[1];
        out.n[2] = vert.normal[2];
        memcpy(out.color, vert.color, 4);
        int id = (int)weld.size();
        weld.push_back(out);
        weldGrid[cell].push_back(id);
        return id;
    };

    ForSoups(bsp, modelIndex, [&](int type, const BspTriSoup& soup) {
        int serial = soupSerial++;
        StOffset off = serial < (int)soupOff.size() ? soupOff[serial] : StOffset{};
        if (soup.materialIndex >= bsp.materials.size() || soup.indexCount < 3)
            return;
        int nameId = MatchNameId(bsp, names, matName, soup.materialIndex);
        unsigned soupContents = bsp.materials[soup.materialIndex].contentFlags;
        for (int i = 0; i + 2 < soup.indexCount; i += 3) {
            int ia = ResolveVert(soup, bsp.indices[type], i, (int)bsp.verts[type].size());
            int ib = ResolveVert(soup, bsp.indices[type], i + 1, (int)bsp.verts[type].size());
            int ic = ResolveVert(soup, bsp.indices[type], i + 2, (int)bsp.verts[type].size());
            if (ia < 0 || ib < 0 || ic < 0)
                continue;
            Vec3 a = FromVert(bsp.verts[type][ia], origin);
            Vec3 b = FromVert(bsp.verts[type][ib], origin);
            Vec3 c = FromVert(bsp.verts[type][ic], origin);
            Vec3 cr = Cross(Sub(b, a), Sub(c, a));
            if (Dot(cr, cr) < 1e-12)
                continue;
            int alpha = ((int)bsp.verts[type][ia].color[3] + (int)bsp.verts[type][ib].color[3]
                + (int)bsp.verts[type][ic].color[3] + 2) / 3;
            if (alpha == 0)
                continue;
            unsigned contents = soupContents;
            unsigned matched = 0;
            bool known = false;
            if (collision && CollisionContentsForTriangle(*collision,
                    bsp.verts[type][ia].xyz, bsp.verts[type][ib].xyz, bsp.verts[type][ic].xyz,
                    known, matched) && known && IsWeaponClipFlags(matched))
                contents = matched;
            if (IsWeaponClipFlags(contents))
                contents = (unsigned)kContentsClipShot | (unsigned)kContentsMissileClip;
            else if (contents & (unsigned)kContentsNonColliding)
                contents = (unsigned)kContentsNonColliding;
            else if (contents & (unsigned)kContentsDetail)
                contents = (unsigned)kContentsDetail;
            else
                contents = 0;
            if (VerticesOnBrushSides(sideGrid, live, brushSides, nameId, a, b, c))
                continue;

            struct PointKey { int x, y, z; };
            PointKey points[3] = {
                { (int)llround(a.x * 1000.0), (int)llround(a.y * 1000.0), (int)llround(a.z * 1000.0) },
                { (int)llround(b.x * 1000.0), (int)llround(b.y * 1000.0), (int)llround(b.z * 1000.0) },
                { (int)llround(c.x * 1000.0), (int)llround(c.y * 1000.0), (int)llround(c.z * 1000.0) }
            };
            sort(points, points + 3, [](const PointKey& lhs, const PointKey& rhs) {
                if (lhs.x != rhs.x) return lhs.x < rhs.x;
                if (lhs.y != rhs.y) return lhs.y < rhs.y;
                return lhs.z < rhs.z;
            });
            PosKey pos;
            pos.nameId = nameId;
            for (int p = 0; p < 3; ++p) {
                pos.p[p * 3 + 0] = points[p].x;
                pos.p[p * 3 + 1] = points[p].y;
                pos.p[p * 3 + 2] = points[p].z;
            }
            if (!seenTris.insert(pos).second)
                continue;

            Tri tri;
            tri.a = weldPoint(bsp.verts[type][ia], off);
            tri.b = weldPoint(bsp.verts[type][ib], off);
            tri.c = weldPoint(bsp.verts[type][ic], off);
            buckets[{ nameId, contents }].push_back(tri);
        }
    });

    for (auto& bucket : buckets) {
        int nameId = bucket.first.nameId;
        string raw = (nameId >= 0 && nameId < (int)names.list.size()) ? names.list[nameId] : "caulk";
        string material = ResolveLayered(raw, bsp);
        vector<char> consumed(bucket.second.size(), 0);
        RecoverClosedBrushes(weld, bucket.second, bucket.first.contents, material, geo.brushes, consumed);
        vector<Tri> surface;
        for (int i = 0; i < (int)bucket.second.size(); ++i)
            if (!consumed[i])
                surface.push_back(bucket.second[i]);
        BuildMeshes(weld, surface, bucket.first.contents, material, geo.meshes, meshCount, vertCount, triMeshes);
    }

    if (geo.brushes.empty() && geo.meshes.empty() && model.numSurfaces > 0) {
        if (model.firstSurface < 0
            || model.firstSurface + model.numSurfaces > (int)bsp.collisionAabbs.size())
            throw runtime_error("model collision aabb range is outside the bsp");

        vector<int> queue;
        for (int i = 0; i < model.numSurfaces; ++i)
            queue.push_back(model.firstSurface + i);
        for (size_t qi = 0; qi < queue.size(); ++qi) {
            int index = queue[qi];
            if (index < 0 || index >= (int)bsp.collisionAabbs.size())
                continue;
            const BspCollisionAabb& aabb = bsp.collisionAabbs[index];
            if (aabb.childCount > 0) {
                for (int c = 0; c < (int)aabb.childCount; ++c)
                    queue.push_back(aabb.u + c);
                continue;
            }

            string material = "caulk";
            unsigned contents = 0;
            if (aabb.materialIndex < bsp.materials.size()) {
                material = ResolveLayered(MaterialName(bsp.materials[aabb.materialIndex]), bsp);
                contents = bsp.materials[aabb.materialIndex].contentFlags;
            }

            double x0 = (double)aabb.midPoint[0] - (double)aabb.halfSize[0] + origin.x;
            double y0 = (double)aabb.midPoint[1] - (double)aabb.halfSize[1] + origin.y;
            double z0 = (double)aabb.midPoint[2] - (double)aabb.halfSize[2] + origin.z;
            double x1 = (double)aabb.midPoint[0] + (double)aabb.halfSize[0] + origin.x;
            double y1 = (double)aabb.midPoint[1] + (double)aabb.halfSize[1] + origin.y;
            double z1 = (double)aabb.midPoint[2] + (double)aabb.halfSize[2] + origin.z;

            const double eps = 1e-3;
            int flat = -1;
            if (fabs(x1 - x0) < eps)
                flat = 0;
            else if (fabs(y1 - y0) < eps)
                flat = 1;
            else if (fabs(z1 - z0) < eps)
                flat = 2;

            if (flat >= 0) {
                MeshOut mesh;
                mesh.contents = contents;
                mesh.material = material;
                mesh.width = 2;
                mesh.height = 2;
                mesh.verts.resize(4);
                auto setV = [&](int vi, double x, double y, double z) {
                    mesh.verts[vi].p = { x, y, z };
                    mesh.verts[vi].st[0] = 0;
                    mesh.verts[vi].st[1] = 0;
                    mesh.verts[vi].lm[0] = 0;
                    mesh.verts[vi].lm[1] = 0;
                    mesh.verts[vi].n[0] = flat == 0 ? 1.f : 0.f;
                    mesh.verts[vi].n[1] = flat == 1 ? 1.f : 0.f;
                    mesh.verts[vi].n[2] = flat == 2 ? 1.f : 0.f;
                    mesh.verts[vi].color[0] = mesh.verts[vi].color[1] = mesh.verts[vi].color[2] = 255;
                    mesh.verts[vi].color[3] = 255;
                };
                if (flat == 0) {
                    setV(0, x0, y0, z0);
                    setV(1, x0, y0, z1);
                    setV(2, x0, y1, z0);
                    setV(3, x0, y1, z1);
                } else if (flat == 1) {
                    setV(0, x0, y0, z0);
                    setV(1, x0, y0, z1);
                    setV(2, x1, y0, z0);
                    setV(3, x1, y0, z1);
                } else {
                    setV(0, x0, y0, z0);
                    setV(1, x0, y1, z0);
                    setV(2, x1, y0, z0);
                    setV(3, x1, y1, z0);
                }
                PushPieces(geo.meshes, move(mesh), meshCount, vertCount);
                ++triMeshes;
            } else {
                BrushOut brush;
                brush.contents = contents;
                auto add = [&](double ax, double ay, double az, double bx, double by, double bz,
                               double cx, double cy, double cz) {
                    FaceOut face;
                    face.material = material;
                    face.p[0] = { ax, ay, az };
                    face.p[1] = { bx, by, bz };
                    face.p[2] = { cx, cy, cz };
                    brush.faces.push_back(face);
                };
                add(x1, y1, z0, x0, y1, z0, x0, y0, z0);
                add(x0, y0, z1, x0, y1, z1, x1, y1, z1);
                add(x1, y0, z0, x0, y0, z0, x0, y0, z1);
                add(x0, y1, z0, x1, y1, z0, x1, y1, z1);
                add(x0, y0, z0, x0, y1, z0, x0, y1, z1);
                add(x1, y1, z0, x1, y0, z0, x1, y0, z1);
                geo.brushes.push_back(move(brush));
            }
        }
    }
    return geo;
}

bool RecoverClosedBrushes(const vector<WeldVert>& verts, const vector<Tri>& tris, unsigned contents,
                          const string& material, vector<BrushOut>& brushes, vector<char>& used)
{
    if (tris.size() < 4)
        return false;
    vector<int> parent(tris.size());
    for (int i = 0; i < (int)tris.size(); ++i)
        parent[i] = i;
    auto find = [&](auto&& self, int x) -> int {
        return parent[x] == x ? x : parent[x] = self(self, parent[x]);
    };
    struct CornerKey {
        int x, y, z;
        bool operator==(const CornerKey& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct EdgeKey {
        int ax, ay, az, bx, by, bz;
        bool operator==(const EdgeKey& o) const
        {
            return ax == o.ax && ay == o.ay && az == o.az && bx == o.bx && by == o.by && bz == o.bz;
        }
    };
    struct EdgeHash {
        size_t operator()(const EdgeKey& k) const
        {
            size_t h = ((size_t)k.ax * 73856093u) ^ ((size_t)k.ay * 19349663u) ^ ((size_t)k.az * 83492791u);
            h ^= ((size_t)k.bx * 73856093u) ^ ((size_t)k.by * 19349663u) ^ ((size_t)k.bz * 83492791u);
            return h;
        }
    };
    auto cornerOf = [&](int id) {
        const Vec3& point = verts[id].p;
        return CornerKey{ (int)llround(point.x * 100.0), (int)llround(point.y * 100.0), (int)llround(point.z * 100.0) };
    };
    unordered_map<EdgeKey, int, EdgeHash> edgeOwner;
    for (int i = 0; i < (int)tris.size(); ++i) {
        int ids[3] = { tris[i].a, tris[i].b, tris[i].c };
        for (int k = 0; k < 3; ++k) {
            CornerKey c0 = cornerOf(ids[k]);
            CornerKey c1 = cornerOf(ids[(k + 1) % 3]);
            EdgeKey edge{ c0.x, c0.y, c0.z, c1.x, c1.y, c1.z };
            if (edge.bx < edge.ax || (edge.bx == edge.ax && edge.by < edge.ay)
                || (edge.bx == edge.ax && edge.by == edge.ay && edge.bz < edge.az)) {
                swap(edge.ax, edge.bx);
                swap(edge.ay, edge.by);
                swap(edge.az, edge.bz);
            }
            auto found = edgeOwner.find(edge);
            if (found == edgeOwner.end())
                edgeOwner[edge] = i;
            else {
                int a = find(find, i);
                int b = find(find, found->second);
                if (a != b)
                    parent[a] = b;
            }
        }
    }

    unordered_map<int, vector<int>> groups;
    for (int i = 0; i < (int)tris.size(); ++i)
        groups[find(find, i)].push_back(i);

    bool any = false;
    for (const auto& group : groups) {
        const vector<int>& ids = group.second;
        if (ids.size() < 4)
            continue;

        struct PlaneKey {
            int x, y, z, d;
            bool operator==(const PlaneKey& o) const { return x == o.x && y == o.y && z == o.z && d == o.d; }
        };
        struct PlaneKeyHash {
            size_t operator()(const PlaneKey& k) const
            {
                return ((size_t)k.x * 73856093u) ^ ((size_t)k.y * 19349663u) ^ ((size_t)k.z * 83492791u) ^ ((size_t)k.d * 17u);
            }
        };
        struct PlaneGroup {
            PlaneD plane;
            vector<int> tris;
        };
        unordered_map<PlaneKey, int, PlaneKeyHash> planeIndex;
        vector<PlaneGroup> planes;
        bool bad = false;
        for (int ti : ids) {
            const Vec3& a = verts[tris[ti].a].p;
            const Vec3& b = verts[tris[ti].b].p;
            const Vec3& c = verts[tris[ti].c].p;
            Vec3 n = Cross(Sub(c, a), Sub(b, a));
            double len = Len(n);
            if (len < 1e-4) {
                bad = true;
                break;
            }
            n = Mul(n, 1.0 / len);
            double dist = Dot(n, a);
            PlaneKey key{ (int)llround(n.x * 200.0), (int)llround(n.y * 200.0), (int)llround(n.z * 200.0), (int)llround(dist * 4.0) };
            auto found = planeIndex.find(key);
            if (found == planeIndex.end()) {
                planeIndex[key] = (int)planes.size();
                planes.push_back({ { { n.x, n.y, n.z }, dist }, { ti } });
            } else {
                planes[found->second].tris.push_back(ti);
            }
        }
        if (bad || planes.size() < 4 || planes.size() > 128)
            continue;

        BrushOut brush;
        brush.contents = contents;
        bool closed = true;
        vector<PlaneD> fitPlanes;
        vector<Vec3> fitAnchors;
        double bmin[3] = { 1e30, 1e30, 1e30 };
        double bmax[3] = { -1e30, -1e30, -1e30 };
        for (int p = 0; p < (int)planes.size(); ++p) {
            vector<Vec3> winding = BaseWinding(planes[p].plane);
            for (int other = 0; other < (int)planes.size() && !winding.empty(); ++other) {
                if (other == p)
                    continue;
                winding = ClipWinding(winding, planes[other].plane);
            }
            OrientWinding(winding, planes[p].plane);
            Vec3 tri[3];
            if (!FacePoints(winding, planes[p].plane, tri)) {
                closed = false;
                break;
            }
            FaceOut face;
            face.p[0] = tri[0];
            face.p[1] = tri[1];
            face.p[2] = tri[2];
            face.material = material;
            vector<Hit> hits(3);
            int sample = planes[p].tris[0];
            int corner[3] = { tris[sample].a, tris[sample].b, tris[sample].c };
            for (int k = 0; k < 3; ++k) {
                hits[k] = { verts[corner[k]].p.x, verts[corner[k]].p.y, verts[corner[k]].p.z,
                            verts[corner[k]].st[0], verts[corner[k]].st[1] };
            }
            FitFace(hits, planes[p].plane, face);
            brush.faces.push_back(face);
            Vec3 anchor{ 0, 0, 0 };
            if (!winding.empty()) {
                for (const Vec3& point : winding)
                    anchor = Add(anchor, point);
                anchor = Mul(anchor, 1.0 / (double)winding.size());
            }
            fitPlanes.push_back(planes[p].plane);
            fitAnchors.push_back(anchor);
            for (const Vec3& point : winding) {
                bmin[0] = min(bmin[0], point.x); bmax[0] = max(bmax[0], point.x);
                bmin[1] = min(bmin[1], point.y); bmax[1] = max(bmax[1], point.y);
                bmin[2] = min(bmin[2], point.z); bmax[2] = max(bmax[2], point.z);
            }
        }
        if (!closed || brush.faces.size() < 4)
            continue;

        double pmin[3] = { 1e30, 1e30, 1e30 };
        double pmax[3] = { -1e30, -1e30, -1e30 };
        bool inside = true;
        for (int ti : ids) {
            int corner[3] = { tris[ti].a, tris[ti].b, tris[ti].c };
            for (int id : corner) {
                const Vec3& point = verts[id].p;
                pmin[0] = min(pmin[0], point.x); pmax[0] = max(pmax[0], point.x);
                pmin[1] = min(pmin[1], point.y); pmax[1] = max(pmax[1], point.y);
                pmin[2] = min(pmin[2], point.z); pmax[2] = max(pmax[2], point.z);
                for (const PlaneGroup& plane : planes) {
                    if (plane.plane.Dist(point) > 1.5) {
                        inside = false;
                        break;
                    }
                }
            }
        }
        if (!inside)
            continue;
        for (int axis = 0; axis < 3; ++axis) {
            double span = max(1.0, pmax[axis] - pmin[axis]);
            double pad = max(8.0, span * 0.5);
            if (bmin[axis] < pmin[axis] - pad || bmax[axis] > pmax[axis] + pad)
                inside = false;
        }
        if (!inside)
            continue;

        FitBrushPlanes(brush.faces, fitPlanes, fitAnchors);
        float solidSlack = 0.f;
        if (BrushSolidPoints(brush.faces, &solidSlack) < 4 || solidSlack < 0.04f)
            continue;
        brushes.push_back(move(brush));
        for (int ti : ids)
            used[ti] = 1;
        for (int ti = 0; ti < (int)tris.size(); ++ti) {
            if (used[ti])
                continue;
            int corner[3] = { tris[ti].a, tris[ti].b, tris[ti].c };
            bool onBrush = true;
            for (int id : corner) {
                const Vec3& point = verts[id].p;
                bool onPlane = false;
                for (const PlaneGroup& plane : planes) {
                    double d = plane.plane.Dist(point);
                    if (d > 0.15) {
                        onBrush = false;
                        break;
                    }
                    if (fabs(d) <= 0.15)
                        onPlane = true;
                }
                if (!onBrush || !onPlane) {
                    onBrush = false;
                    break;
                }
            }
            if (onBrush)
                used[ti] = 1;
        }
        any = true;
    }
    return any;
}

void SkipSpace(const char*& p)
{
    while (*p && isspace((unsigned char)*p))
        ++p;
}

bool ParseQuoted(const char*& p, string& out)
{
    SkipSpace(p);
    out.clear();
    if (*p == '"') {
        ++p;
        while (*p && *p != '"') {
            out.push_back(*p);
            ++p;
        }
        if (*p != '"')
            return false;
        ++p;
        return true;
    }
    if (!*p || *p == '{' || *p == '}')
        return false;
    while (*p && !isspace((unsigned char)*p)) {
        out.push_back(*p);
        ++p;
    }
    return !out.empty();
}

vector<Ent> ParseEntities(const string& text)
{
    const char* p = text.c_str();
    vector<Ent> ents;
    while (true) {
        SkipSpace(p);
        if (!*p)
            break;
        if (*p != '{')
            throw runtime_error("entity lump is not map text");
        ++p;
        Ent ent;
        while (true) {
            SkipSpace(p);
            if (*p == '}') {
                ++p;
                break;
            }
            if (!*p)
                throw runtime_error("entity lump ended inside an entity");
            Pair pair;
            if (!ParseQuoted(p, pair.key) || !ParseQuoted(p, pair.value))
                throw runtime_error("entity lump has a broken key");
            ent.pairs.push_back(move(pair));
        }
        ents.push_back(move(ent));
    }
    return ents;
}

string ClassName(const Ent& ent)
{
    for (const Pair& pair : ent.pairs)
        if (pair.key == "classname")
            return pair.value;
    return {};
}

string KeyOf(const Ent& ent, const string& key)
{
    for (const Pair& pair : ent.pairs)
        if (pair.key == key)
            return pair.value;
    return {};
}

void SetKey(Ent& ent, const string& key, const string& value)
{
    for (Pair& pair : ent.pairs) {
        if (pair.key == key) {
            pair.value = value;
            return;
        }
    }
    ent.pairs.push_back({ key, value });
}

int FindEntWithTargetname(const vector<Ent>& ents, const string& name)
{
    if (name.empty())
        return -1;
    for (int i = 0; i < (int)ents.size(); ++i) {
        if (KeyOf(ents[i], "targetname") == name)
            return i;
    }
    return -1;
}

Vec3 OriginOf(const Ent& ent)
{
    Vec3 origin{ 0, 0, 0 };
    string value = KeyOf(ent, "origin");
    if (value.empty())
        return origin;
    sscanf(value.c_str(), "%lf %lf %lf", &origin.x, &origin.y, &origin.z);
    return origin;
}

const BspPrimaryLight* MatchPrimaryLight(const BspFile& bsp, const Ent& ent)
{
    Vec3 origin = OriginOf(ent);
    string indexText = KeyOf(ent, "pl#");
    if (!indexText.empty()) {
        int index = atoi(indexText.c_str());
        if (index >= 0 && index < (int)bsp.primaryLights.size()) {
            const BspPrimaryLight& candidate = bsp.primaryLights[index];
            Vec3 delta{ origin.x - candidate.origin[0], origin.y - candidate.origin[1],
                        origin.z - candidate.origin[2] };
            if (Len(delta) < 1.0)
                return &candidate;
        }
    }
    for (const BspPrimaryLight& candidate : bsp.primaryLights) {
        Vec3 delta{ origin.x - candidate.origin[0], origin.y - candidate.origin[1],
                    origin.z - candidate.origin[2] };
        if (Len(delta) < 1.0)
            return &candidate;
    }
    return nullptr;
}

int StarModel(const string& value)
{
    if (value.size() < 2 || value[0] != '*')
        return -1;
    char* end = nullptr;
    long n = strtol(value.c_str() + 1, &end, 10);
    if (!end || *end || n < 0 || n > 1000000)
        return -1;
    return (int)n;
}

bool IsLightClass(const string& name)
{
    return name == "light" || name.rfind("light_", 0) == 0;
}

bool IsLightTargetClass(const string& name)
{
    return name == "info_null" || name == "info_notnull" || name == "info_notnull_big";
}

bool DroppedClass(const string& name)
{
    return name == "misc_prefab" || name == "func_group" || name == "group_info" || name == "reflection_probe";
}

struct FileOut {
    FILE* f = nullptr;
    explicit FileOut(const fs::path& path)
    {
        if (_wfopen_s(&f, path.c_str(), L"wb") != 0 || !f)
            throw runtime_error("could not write " + path.string());
    }
    ~FileOut()
    {
        if (f)
            fclose(f);
    }
};

void WriteHeader(FILE* f)
{
    fputs("iwmap 4\n", f);
    fputs("\"000_Global\" flags  active\n", f);
    fputs("\"The Map\" flags \n", f);
}

void WritePairs(FILE* f, const Ent& ent)
{
    for (const Pair& pair : ent.pairs) {
        if (pair.key == "gndLt" || pair.key == "pl#")
            continue;
        if ((pair.key == "model" || pair.key == "physicsmodel") && StarModel(pair.value) >= 0)
            continue;
        fprintf(f, "\"%s\" \"%s\"\n", pair.key.c_str(), pair.value.c_str());
    }
}

void WriteBrush(FILE* f, int index, const BrushOut& brush)
{
    fprintf(f, "// brush %d\n{\n", index);
    if (const char* contents = ContentsToken(brush.contents))
        fprintf(f, " contents %s;\n", contents);
    for (const FaceOut& face : brush.faces) {
        for (int i = 0; i < 3; ++i) {
            fputs(i ? " " : "", f);
            fputs("( ", f);
            WriteNum(f, face.p[i].x); fputc(' ', f);
            WriteNum(f, face.p[i].y); fputc(' ', f);
            WriteNum(f, face.p[i].z);
            fputs(" )", f);
        }
        fprintf(f, " %s ", face.material.c_str());
        WriteNum(f, face.scaleS); fputc(' ', f);
        WriteNum(f, face.scaleT); fputc(' ', f);
        WriteNum(f, face.shiftS); fputc(' ', f);
        WriteNum(f, face.shiftT); fputc(' ', f);
        WriteNum(f, face.rotate); fputc(' ', f);
        WriteNum(f, face.skew);
        fputs(" lightmap_gray ", f);
        WriteNum(f, face.lmScaleS); fputc(' ', f);
        WriteNum(f, face.lmScaleT); fputc(' ', f);
        WriteNum(f, face.lmShiftS); fputc(' ', f);
        WriteNum(f, face.lmShiftT); fputc(' ', f);
        WriteNum(f, face.lmRotate); fputc(' ', f);
        WriteNum(f, face.lmSkew);
        fputc('\n', f);
    }
    fputs("}\n", f);
}

void WriteMesh(FILE* f, int index, const MeshOut& mesh)
{
    fprintf(f, "// brush %d\n{\n mesh\n {\n", index);
    if (const char* contents = ContentsToken(mesh.contents))
        fprintf(f, "  contents %s;\n", contents);
    fprintf(f, "  %s\n  lightmap_gray\n  %d %d 0 1\n", mesh.material.c_str(), mesh.width, mesh.height);
    for (int col = 0; col < mesh.width; ++col) {
        fputs("  (\n", f);
        for (int row = 0; row < mesh.height; ++row) {
            const MeshVertOut& v = mesh.verts[(size_t)row * mesh.width + col];
            fputs("   v ", f);
            WriteNum(f, v.p.x); fputc(' ', f);
            WriteNum(f, v.p.y); fputc(' ', f);
            WriteNum(f, v.p.z);
            fprintf(f, " c %u %u %u %u", v.color[2], v.color[1], v.color[0], v.color[3]);
            fputs(" t ", f);
            WriteNum(f, (double)v.st[0] * kPatchTexScale); fputc(' ', f);
            WriteNum(f, (double)v.st[1] * kPatchTexScale); fputc(' ', f);
            WriteNum(f, (double)v.lm[0] * kPatchTexScale); fputc(' ', f);
            WriteNum(f, (double)v.lm[1] * kPatchTexScale);
            if (v.flag)
                fprintf(f, " f %d", v.flag);
            fputc('\n', f);
        }
        fputs("  )\n", f);
    }
    fputs(" }\n}\n", f);
}

double GridToWorld(int index, int axis)
{
    if (axis == 2)
        return (index - 0x0800) * 64.0;
    return (index - 0x1000) * 32.0;
}

BrushOut AxisBrush(const string& material, double x0, double y0, double z0, double x1, double y1, double z1)
{
    BrushOut brush;
    brush.contents = 0;
    auto add = [&](double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz) {
        FaceOut face;
        face.material = material;
        face.p[0] = { ax, ay, az };
        face.p[1] = { bx, by, bz };
        face.p[2] = { cx, cy, cz };
        brush.faces.push_back(face);
    };
    add(x1, y1, z0, x0, y1, z0, x0, y0, z0);
    add(x0, y0, z1, x0, y1, z1, x1, y1, z1);
    add(x1, y0, z0, x0, y0, z0, x0, y0, z1);
    add(x0, y1, z0, x1, y1, z0, x1, y1, z1);
    add(x0, y0, z0, x0, y1, z0, x0, y1, z1);
    add(x1, y1, z0, x1, y0, z0, x1, y0, z1);
    return brush;
}

struct GridKey {
    int x, y, z;
    bool operator==(const GridKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct GridHash {
    size_t operator()(const GridKey& k) const
    {
        return ((size_t)k.x * 73856093u) ^ ((size_t)k.y * 19349663u) ^ ((size_t)k.z * 83492791u);
    }
};

vector<GridKey> ReadGridPoints(const fs::path& path)
{
    vector<GridKey> points;
    FILE* file = nullptr;
    if (_wfopen_s(&file, path.c_str(), L"rb") != 0 || !file)
        return points;
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    if (size <= 0 || size % 6 != 0) {
        fclose(file);
        return points;
    }
    int count = (int)(size / 6);
    points.reserve((size_t)count);
    for (int i = 0; i < count; ++i) {
        unsigned short pos[3];
        if (fread(pos, 2, 3, file) != 3)
            break;
        points.push_back({ (int)pos[0], (int)pos[1], (int)pos[2] });
    }
    fclose(file);
    return points;
}

void AddGridVolumes(vector<BrushOut>& brushes, const vector<GridKey>& points, const string& material)
{
    if (points.empty())
        return;
    unordered_map<GridKey, int, GridHash> lookup;
    lookup.reserve(points.size() * 2);
    for (int i = 0; i < (int)points.size(); ++i)
        lookup.emplace(points[i], i);

    vector<int> parent(points.size());
    for (int i = 0; i < (int)points.size(); ++i)
        parent[i] = i;
    auto find = [&](auto&& self, int x) -> int {
        return parent[x] == x ? x : parent[x] = self(self, parent[x]);
    };
    const int step[6][3] = {
        { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
    };
    for (int i = 0; i < (int)points.size(); ++i) {
        for (int n = 0; n < 6; ++n) {
            GridKey next{ points[i].x + step[n][0], points[i].y + step[n][1], points[i].z + step[n][2] };
            auto found = lookup.find(next);
            if (found == lookup.end())
                continue;
            int a = find(find, i);
            int b = find(find, found->second);
            if (a != b)
                parent[a] = b;
        }
    }

    struct Bounds {
        int min[3];
        int max[3];
        bool init = false;
    };
    unordered_map<int, Bounds> groups;
    for (int i = 0; i < (int)points.size(); ++i) {
        int root = find(find, i);
        Bounds& box = groups[root];
        int pos[3] = { points[i].x, points[i].y, points[i].z };
        if (!box.init) {
            for (int axis = 0; axis < 3; ++axis)
                box.min[axis] = box.max[axis] = pos[axis];
            box.init = true;
        } else {
            for (int axis = 0; axis < 3; ++axis) {
                box.min[axis] = min(box.min[axis], pos[axis]);
                box.max[axis] = max(box.max[axis], pos[axis]);
            }
        }
    }

    for (const auto& item : groups) {
        const Bounds& box = item.second;
        double wmin[3];
        double wmax[3];
        for (int axis = 0; axis < 3; ++axis) {
            wmin[axis] = GridToWorld(box.min[axis], axis);
            wmax[axis] = GridToWorld(box.max[axis], axis);
            if (box.min[axis] == box.max[axis])
                wmax[axis] += axis == 2 ? 64.0 : 32.0;
        }
        brushes.push_back(AxisBrush(material, wmin[0], wmin[1], wmin[2], wmax[0], wmax[1], wmax[2]));
    }
}

void WriteGeometry(FILE* f, int& brushIndex, const ModelGeo& geo)
{
    for (const BrushOut& brush : geo.brushes) {
        if (brush.faces.size() < 4)
            continue;
        WriteBrush(f, brushIndex++, brush);
    }
    for (const MeshOut& mesh : geo.meshes)
        WriteMesh(f, brushIndex++, mesh);
}

void WriteProbe(FILE* f, int index, const BspProbe& probe)
{
    char name[65] = {};
    memcpy(name, probe.colorCorrection, 64);
    fprintf(f, "// entity %d\n{\n\"origin\" \"", index);
    WriteNum(f, probe.origin[0]); fputc(' ', f);
    WriteNum(f, probe.origin[1]); fputc(' ', f);
    WriteNum(f, probe.origin[2]);
    fputs("\"\n", f);
    if (name[0]) {
        for (char* p = name; *p; ++p)
            if (*p == '"')
                *p = '\'';
        fprintf(f, "\"color_correction\" \"%s\"\n", name);
    }
    fputs("\"classname\" \"reflection_probe\"\n}\n", f);
}

float CodRint(float value)
{
    return (float)floor((double)(value + 0.5f));
}

void CodSnapPlane(float n[3], float* dist)
{
    for (int i = 0; i < 3; ++i) {
        if (fabs(n[i] - 1.0f) < 1e-7) {
            n[0] = n[1] = n[2] = 0.f;
            n[i] = 1.f;
            break;
        }
        if (fabs(n[i] - -1.0f) < 1e-7) {
            n[0] = n[1] = n[2] = 0.f;
            n[i] = -1.f;
            break;
        }
    }
    float snapped = CodRint(*dist);
    if (fabs(*dist - snapped) < 0.001f)
        *dist = snapped;
}

bool CodPlaneFromPoints(float plane[4], const float p0[3], const float p1[3], const float p2[3])
{
    float d1[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
    float d2[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
    plane[0] = d2[1] * d1[2] - d2[2] * d1[1];
    plane[1] = d2[2] * d1[0] - d2[0] * d1[2];
    plane[2] = d2[0] * d1[1] - d2[1] * d1[0];
    float lenSq = plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2];
    if (lenSq < 2.f) {
        if (lenSq == 0.f)
            return false;
        float d1sq = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2];
        float d2sq = d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2];
        if (lenSq < d1sq * d2sq * 0.000001f) {
            float e1[3] = { p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2] };
            float e2[3] = { p0[0] - p1[0], p0[1] - p1[1], p0[2] - p1[2] };
            plane[0] = e2[1] * e1[2] - e2[2] * e1[1];
            plane[1] = e2[2] * e1[0] - e2[0] * e1[2];
            plane[2] = e2[0] * e1[1] - e2[1] * e1[0];
            lenSq = plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2];
            float e1sq = e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2];
            float e2sq = e2[0] * e2[0] + e2[1] * e2[1] + e2[2] * e2[2];
            if (lenSq == 0.f || lenSq < e1sq * e2sq * 0.000001f)
                return false;
        }
    }
    float len = (float)sqrt((double)lenSq);
    plane[0] /= len;
    plane[1] /= len;
    plane[2] /= len;
    plane[3] = plane[0] * p0[0] + plane[1] * p0[1] + plane[2] * p0[2];
    return true;
}

bool CodFacePlane(const FaceOut& face, float plane[4])
{
    float p0[3], p1[3], p2[3];
    LoadFacePoint(face.p[0], p0);
    LoadFacePoint(face.p[1], p1);
    LoadFacePoint(face.p[2], p2);
    return CodPlaneFromPoints(plane, p0, p1, p2);
}

float CodDot(const float* a, const float* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

bool CodIntersect(const float* p0, const float* p1, const float* p2, float out[3])
{
    double det = (double)((p1[1] * p2[2] - p2[1] * p1[2]) * p0[0])
               + (double)((p2[1] * p0[2] - p0[1] * p2[2]) * p1[0])
               + (double)((p0[1] * p1[2] - p1[1] * p0[2]) * p2[0]);
    if (fabs(det) < (double)0.001f)
        return false;
    double inv = 1.0 / det;
    out[0] = (float)(((double)((p1[1] * p2[2] - p2[1] * p1[2]) * p0[3])
                    + (double)((p2[1] * p0[2] - p0[1] * p2[2]) * p1[3])
                    + (double)((p0[1] * p1[2] - p1[1] * p0[2]) * p2[3])) * inv);
    out[1] = (float)(((double)((p1[2] * p2[0] - p2[2] * p1[0]) * p0[3])
                    + (double)((p2[2] * p0[0] - p0[2] * p2[0]) * p1[3])
                    + (double)((p0[2] * p1[0] - p1[2] * p0[0]) * p2[3])) * inv);
    out[2] = (float)(((double)((p1[0] * p2[1] - p2[0] * p1[1]) * p0[3])
                    + (double)((p2[0] * p0[1] - p0[0] * p2[1]) * p1[3])
                    + (double)((p0[0] * p1[1] - p1[0] * p0[1]) * p2[3])) * inv);
    return true;
}

void CodSnapPoint(const float* p0, const float* p1, const float* p2, float point[3])
{
    float snapped[3];
    for (int i = 0; i < 3; ++i) {
        float grid = (float)RoundToInt(point[i] / 0.25f) * 0.25f;
        snapped[i] = fabsf(grid - point[i]) < 0.01f ? grid : point[i];
    }
    if (snapped[0] == point[0] && snapped[1] == point[1] && snapped[2] == point[2])
        return;
    float maxError = 0.f;
    float maxAllowed = 0.01f;
    const float* planes[3] = { p0, p1, p2 };
    for (int i = 0; i < 3; ++i) {
        float onSnap = fabsf(CodDot(planes[i], snapped) - planes[i][3]);
        if (maxError < onSnap)
            maxError = onSnap;
        float onPoint = fabsf(CodDot(planes[i], point) - planes[i][3]);
        if (maxAllowed < onPoint)
            maxAllowed = onPoint;
    }
    if (maxError < maxAllowed) {
        point[0] = snapped[0];
        point[1] = snapped[1];
        point[2] = snapped[2];
    }
}

struct CodMapPlane {
    float n[3];
    float d;
    int hashNext;
    int born;
};

struct CodPlanePool {
    vector<CodMapPlane> planes;
    int hash[1024];

    CodPlanePool()
    {
        for (int i = 0; i < 1024; ++i)
            hash[i] = -1;
    }

    void Save(int out[1024]) const { memcpy(out, hash, sizeof(hash)); }

    void Restore(int count, const int saved[1024])
    {
        planes.resize(count);
        memcpy(hash, saved, sizeof(hash));
    }

    void HashPlane(int idx)
    {
        int bucket = (int)fabsf(planes[idx].d) / 8;
        bucket &= 1023;
        planes[idx].hashNext = hash[bucket];
        hash[bucket] = idx;
    }

    bool PlaneEqual(int idx, const float n[3], float dist) const
    {
        const CodMapPlane& p = planes[idx];
        return fabs(p.n[0] - n[0]) < 1e-7 && fabs(p.n[1] - n[1]) < 1e-7
            && fabs(p.n[2] - n[2]) < 1e-7 && fabs(p.d - dist) < 0.001f;
    }

    int FindExisting(const float n[3], float dist, int limit) const
    {
        int hashBase = (int)(fabs((double)dist) / 8.0) & 1023;
        for (int off = -1; off <= 1; ++off) {
            int bucket = (hashBase + off) & 1023;
            for (int p = hash[bucket]; p >= 0; p = planes[p].hashNext) {
                if (p >= limit)
                    continue;
                if (PlaneEqual(p, n, dist))
                    return p;
            }
        }
        return -1;
    }

    int Create(const float n[3], float dist, int born)
    {
        int idx = (int)planes.size();
        CodMapPlane a{};
        CodMapPlane b{};
        a.n[0] = n[0];
        a.n[1] = n[1];
        a.n[2] = n[2];
        a.d = dist;
        a.born = born;
        a.hashNext = -1;
        b.n[0] = -n[0];
        b.n[1] = -n[1];
        b.n[2] = -n[2];
        b.d = -dist;
        b.born = born;
        b.hashNext = -1;
        int type = 3;
        if (a.n[0] == 1.f || a.n[0] == -1.f)
            type = 0;
        else if (a.n[1] == 1.f || a.n[1] == -1.f)
            type = 1;
        else if (a.n[2] == 1.f || a.n[2] == -1.f)
            type = 2;
        planes.push_back(a);
        planes.push_back(b);
        if (type < 3 && (planes[idx].n[0] < 0.f || planes[idx].n[1] < 0.f || planes[idx].n[2] < 0.f)) {
            swap(planes[idx], planes[idx + 1]);
            HashPlane(idx);
            HashPlane(idx + 1);
            return idx + 1;
        }
        HashPlane(idx);
        HashPlane(idx + 1);
        return idx;
    }

    int Find(float n[3], float dist, int born)
    {
        CodSnapPlane(n, &dist);
        int hashBase = (int)(fabs((double)dist) / 8.0) & 1023;
        for (int off = -1; off <= 1; ++off) {
            int bucket = (hashBase + off) & 1023;
            for (int p = hash[bucket]; p >= 0; p = planes[p].hashNext) {
                if (PlaneEqual(p, n, dist))
                    return p;
            }
        }
        return Create(n, dist, born);
    }
};

struct CodHit {
    float err = 0.f;
    int face[3] = { -1, -1, -1 };
    int born[3] = { -1, -1, -1 };
    int which = 0;
};

float CodWorst(const CodPlanePool& pool, const vector<int>& faces, const vector<int>& nums, CodHit* hit, int* pointCount)
{
    int n = (int)nums.size();
    float worst = 0.f;
    int points = 0;
    if (pointCount)
        *pointCount = 0;
    vector<float> pl((size_t)n * 4);
    for (int i = 0; i < n; ++i) {
        const CodMapPlane& src = pool.planes[nums[i]];
        pl[(size_t)i * 4 + 0] = src.n[0];
        pl[(size_t)i * 4 + 1] = src.n[1];
        pl[(size_t)i * 4 + 2] = src.n[2];
        pl[(size_t)i * 4 + 3] = src.d;
    }
    auto at = [&](int i) -> const float* { return &pl[(size_t)i * 4]; };
    for (int i = 0; i < n - 2; ++i) {
        for (int j = i + 1; j < n - 1; ++j) {
            if (nums[j] == nums[i])
                continue;
            for (int k = j + 1; k < n; ++k) {
                if (nums[k] == nums[j] || nums[k] == nums[i])
                    continue;
                float point[3];
                if (!CodIntersect(at(i), at(j), at(k), point))
                    continue;
                CodSnapPoint(at(i), at(j), at(k), point);
                bool outside = false;
                for (int t = 0; t < n; ++t) {
                    if (nums[t] == nums[i] || nums[t] == nums[j] || nums[t] == nums[k])
                        continue;
                    if (CodDot(at(t), point) - at(t)[3] > 0.1f) {
                        outside = true;
                        break;
                    }
                }
                if (outside)
                    continue;
                points++;
                int ids[3] = { i, j, k };
                for (int s = 0; s < 3; ++s) {
                    float err = CodDot(at(ids[s]), point) - at(ids[s])[3];
                    float tol = 0.1f;
                    float scaled = fabsf(at(ids[s])[3]) * 0.00001f;
                    if (scaled > tol)
                        tol = scaled;
                    if (!(fabsf(err) < tol) && fabsf(err) > fabsf(worst)) {
                        worst = err;
                        if (hit) {
                            hit->err = err;
                            hit->which = s;
                            hit->face[0] = faces[i];
                            hit->face[1] = faces[j];
                            hit->face[2] = faces[k];
                            hit->born[0] = pool.planes[nums[i]].born;
                            hit->born[1] = pool.planes[nums[j]].born;
                            hit->born[2] = pool.planes[nums[k]].born;
                        }
                    }
                }
            }
        }
    }
    if (pointCount)
        *pointCount = points;
    return worst;
}

bool CodAssign(CodPlanePool& pool, BrushOut& brush, int born, vector<int>& sideFaces, vector<int>& nums, CodHit* hit, float* err)
{
    sideFaces.clear();
    nums.clear();
    for (int i = 0; i < (int)brush.faces.size(); ++i) {
        float plane[4];
        if (!CodFacePlane(brush.faces[i], plane))
            continue;
        sideFaces.push_back(i);
        nums.push_back(pool.Find(plane, plane[3], born));
    }
    int points = 0;
    *err = sideFaces.size() >= 3 ? CodWorst(pool, sideFaces, nums, hit, &points) : 1.f;
    if (points < 4) {
        *err = 1.f;
        return false;
    }
    return fabsf(*err) < 0.1f;
}

bool CodSeparateFace(FaceOut& face, const FaceOut& original, int step, const CodPlanePool& pool, int limit)
{
    face = original;
    float plane[4];
    if (!CodFacePlane(original, plane))
        return false;
    CodSnapPlane(plane, &plane[3]);
    if (step < 16) {
        const float shifts[] = {
            0.002f, -0.002f, 0.004f, -0.004f, 0.008f, -0.008f, 0.016f, -0.016f,
            0.032f, -0.032f, 0.064f, -0.064f, 0.125f, -0.125f, 0.25f, -0.25f
        };
        float delta = shifts[step];
        for (int i = 0; i < 3; ++i) {
            face.p[i].x = RoundTripCoord(original.p[i].x + (double)plane[0] * delta);
            face.p[i].y = RoundTripCoord(original.p[i].y + (double)plane[1] * delta);
            face.p[i].z = RoundTripCoord(original.p[i].z + (double)plane[2] * delta);
        }
    } else {
        int jog = step - 16;
        if (jog >= 6)
            return false;
        float amount = (jog & 1 ? -1.f : 1.f) * (0.05f * (float)(1 + jog / 2));
        for (int i = 0; i < 3; ++i) {
            face.p[i].x = RoundTripCoord(original.p[i].x + (double)plane[0] * amount);
            face.p[i].y = RoundTripCoord(original.p[i].y + (double)plane[1] * amount);
            face.p[i].z = RoundTripCoord(original.p[i].z + (double)plane[2] * amount);
        }
    }
    float recovered[4];
    if (!CodFacePlane(face, recovered)) {
        face = original;
        return false;
    }
    CodSnapPlane(recovered, &recovered[3]);
    if (CodDot(recovered, plane) < 0.f || pool.FindExisting(recovered, recovered[3], limit) >= 0) {
        face = original;
        return false;
    }
    return true;
}

bool CodShareFace(BrushOut& brush, const vector<FaceOut>& original, int victim, int partner)
{
    float victimPlane[4], partnerPlane[4];
    if (!CodFacePlane(original[victim], victimPlane) || !CodFacePlane(original[partner], partnerPlane))
        return false;
    CodSnapPlane(victimPlane, &victimPlane[3]);
    CodSnapPlane(partnerPlane, &partnerPlane[3]);
    if (CodDot(victimPlane, partnerPlane) < 0.75f)
        return false;
    float p0[3];
    LoadFacePoint(original[victim].p[0], p0);
    float gap = fabsf(CodDot(partnerPlane, p0) - partnerPlane[3]);
    if (gap > 1.f)
        return false;
    brush.faces[victim].p[0] = original[partner].p[0];
    brush.faces[victim].p[1] = original[partner].p[1];
    brush.faces[victim].p[2] = original[partner].p[2];
    return true;
}

void SealParsedPlanes(const vector<BrushOut*>& brushes)
{
    CodPlanePool pool;
    int moved = 0;
    int left = 0;
    for (int brushIndex = 0; brushIndex < (int)brushes.size(); ++brushIndex) {
        BrushOut& brush = *brushes[brushIndex];
        if (brush.faces.size() < 3) {
            vector<int> sideFaces, nums;
            CodHit unused;
            float err = 0.f;
            CodAssign(pool, brush, brushIndex, sideFaces, nums, &unused, &err);
            continue;
        }
        int mark = (int)pool.planes.size();
        int markHash[1024];
        pool.Save(markHash);
        vector<int> sideFaces, nums;
        CodHit hit;
        float err = 0.f;
        if (CodAssign(pool, brush, brushIndex, sideFaces, nums, &hit, &err))
            continue;

        vector<FaceOut> original = brush.faces;
        vector<int> step(brush.faces.size(), 0);
        vector<char> shared(brush.faces.size(), 0);
        bool fixed = false;
        for (int attempt = 0; attempt < 64; ++attempt) {
            pool.Restore(mark, markHash);
            if (CodAssign(pool, brush, brushIndex, sideFaces, nums, &hit, &err)) {
                fixed = true;
                break;
            }
            int victim = -1;
            if (hit.which >= 0 && hit.which < 3 && hit.face[hit.which] >= 0
                && hit.born[hit.which] != brushIndex && !shared[hit.face[hit.which]]
                && step[hit.face[hit.which]] < 22)
                victim = hit.face[hit.which];
            if (victim < 0) {
                for (int s = 0; s < 3; ++s) {
                    int face = hit.face[s];
                    if (face >= 0 && hit.born[s] != brushIndex && !shared[face] && step[face] < 22) {
                        victim = face;
                        break;
                    }
                }
            }
            if (victim < 0 && hit.which >= 0 && hit.which < 3) {
                int face = hit.face[hit.which];
                if (face >= 0 && !shared[face] && step[face] < 22)
                    victim = face;
            }
            if (victim < 0)
                break;
            pool.Restore(mark, markHash);
            bool changed = false;
            if (step[victim] < 22) {
                int use = step[victim]++;
                changed = CodSeparateFace(brush.faces[victim], original[victim], use, pool, mark);
                if (!changed)
                    continue;
            }
            if (!changed && !shared[victim]) {
                int partner = -1;
                float bestDot = 0.75f;
                for (int s = 0; s < 3; ++s) {
                    int other = hit.face[s];
                    if (other < 0 || other == victim)
                        continue;
                    float a[4], b[4];
                    if (!CodFacePlane(original[victim], a) || !CodFacePlane(original[other], b))
                        continue;
                    float dot = CodDot(a, b);
                    if (dot > bestDot) {
                        bestDot = dot;
                        partner = other;
                    }
                }
                if (partner >= 0 && CodShareFace(brush, original, victim, partner)) {
                    shared[victim] = 1;
                    changed = true;
                }
            }
            if (!changed)
                shared[victim] = 1;
        }
        if (!fixed) {
            brush.faces = original;
            pool.Restore(mark, markHash);
            if (!CodAssign(pool, brush, brushIndex, sideFaces, nums, &hit, &err)) {
                if (brush.portal)
                    continue;
                brush.faces.clear();
                left++;
                cout << "plane seal dropped brush " << brushIndex << " error " << err << endl;
            }
        } else {
            moved++;
        }
    }
    if (moved || left)
        cout << "plane seal moved " << moved << " brushes, dropped " << left << endl;
}

string FormatFloat(double v)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%.9g", v);
    return buf;
}

string FormatVec3(double x, double y, double z)
{
    return FormatFloat(x) + " " + FormatFloat(y) + " " + FormatFloat(z);
}

string CStringN(const char* text, int cap)
{
    int n = 0;
    while (n < cap && text[n])
        ++n;
    return string(text, text + n);
}

bool NearPoint(const Vec3& a, const Vec3& b)
{
    return fabs(a.x - b.x) <= kPatchWeld && fabs(a.y - b.y) <= kPatchWeld && fabs(a.z - b.z) <= kPatchWeld;
}

double FovFromCosHalf(float cosHalf)
{
    double c = cosHalf;
    if (c > 1.0)
        c = 1.0;
    if (c < -1.0)
        c = -1.0;
    return acos(c) * (360.0 / kPi);
}

void AddMissingPrimaryLights(const BspFile& bsp, vector<Ent>& ents)
{
    vector<Vec3> have;
    unordered_set<string> names;
    for (const Ent& ent : ents) {
        string targetname = KeyOf(ent, "targetname");
        if (!targetname.empty())
            names.insert(targetname);
        string target = KeyOf(ent, "target");
        if (!target.empty())
            names.insert(target);
        if (IsLightClass(ClassName(ent)))
            have.push_back(OriginOf(ent));
    }

    int added = 0;
    for (int i = 0; i < (int)bsp.primaryLights.size(); ++i) {
        const BspPrimaryLight& light = bsp.primaryLights[i];
        if (light.type != 2 && light.type != 3)
            continue;
        if (!(light.radius > 0.f))
            continue;
        Vec3 origin{ light.origin[0], light.origin[1], light.origin[2] };
        bool exists = false;
        for (const Vec3& at : have)
            exists = exists || NearPoint(at, origin);
        if (exists)
            continue;

        Vec3 dir{ light.dir[0], light.dir[1], light.dir[2] };
        double dirLen = Len(dir);
        if (dirLen < 1e-8)
            continue;
        dir = Mul(dir, 1.0 / dirLen);

        double cosOuter = light.cosHalfFovOuter;
        if (cosOuter > 1.0)
            cosOuter = 1.0;
        if (cosOuter < -1.0)
            cosOuter = -1.0;
        double half = acos(cosOuter);
        double dist = 64.0;
        double sine = sin(half);
        if (sine > 1e-6) {
            double along = 64.0 * cos(half) / sine;
            if (along > 1.0)
                dist = along;
        }
        Vec3 aim = Sub(origin, Mul(dir, dist));

        string target = "light_target_" + to_string(i);
        while (names.count(target))
            target += "_";
        names.insert(target);

        float maxChannel = light.color[0];
        if (light.color[1] > maxChannel)
            maxChannel = light.color[1];
        if (light.color[2] > maxChannel)
            maxChannel = light.color[2];
        float color[3];
        float intensity;
        if (maxChannel == 0.f) {
            color[0] = color[1] = color[2] = 1.f;
            intensity = 0.f;
        } else {
            color[0] = light.color[0] / maxChannel;
            color[1] = light.color[1] / maxChannel;
            color[2] = light.color[2] / maxChannel;
            intensity = maxChannel;
        }

        int spawn = light.type == 2 ? 2 : 1;
        bool moving = light.translationLimit != 0.f || fabsf(light.rotationLimit - 1.f) > 1e-6f;
        if (moving)
            spawn |= 4;
        if (light.canUseShadowMap == 0)
            spawn |= 8;

        Ent lamp;
        lamp.pairs.push_back({ "classname", "light" });
        lamp.pairs.push_back({ "origin", FormatVec3(origin.x, origin.y, origin.z) });
        string def = CStringN(light.defName, 64);
        if (!def.empty())
            lamp.pairs.push_back({ "def", def });
        lamp.pairs.push_back({ "radius", FormatFloat(light.radius) });
        lamp.pairs.push_back({ "_color", FormatVec3(color[0], color[1], color[2]) });
        lamp.pairs.push_back({ "intensity", FormatFloat(intensity) });
        lamp.pairs.push_back({ "fov_outer", FormatFloat(FovFromCosHalf(light.cosHalfFovOuter)) });
        lamp.pairs.push_back({ "fov_inner", FormatFloat(FovFromCosHalf(light.cosHalfFovInner)) });
        if (light.type == 2)
            lamp.pairs.push_back({ "exponent", to_string(light.coneExponent) });
        lamp.pairs.push_back({ "target", target });
        lamp.pairs.push_back({ "spawnflags", to_string(spawn) });
        if (moving) {
            double turnCos = light.rotationLimit;
            if (turnCos > 1.0)
                turnCos = 1.0;
            if (turnCos < -1.0)
                turnCos = -1.0;
            lamp.pairs.push_back({ "maxmove", FormatFloat(light.translationLimit) });
            lamp.pairs.push_back({ "maxturn", FormatFloat(acos(turnCos) * (180.0 / kPi)) });
        }

        Ent marker;
        marker.pairs.push_back({ "classname", "info_null" });
        marker.pairs.push_back({ "origin", FormatVec3(aim.x, aim.y, aim.z) });
        marker.pairs.push_back({ "targetname", target });
        ents.push_back(move(lamp));
        ents.push_back(move(marker));
        added++;
    }
    if (added)
        cout << "restored " << added << " primary lights" << endl;
}

}

ExportStats ExportBsp(const fs::path& bspPath, const fs::path& exeDir)
{
    cout << "reading " << bspPath.string() << endl;
    BspFile bsp = LoadBsp(bspPath.wstring());
    cout << "version " << bsp.version
         << "  brushes " << bsp.brushes.size()
         << "  models " << bsp.models.size()
         << "  probes " << bsp.probes.size() << endl;

    Names names;
    int caulk = names.Intern("caulk");
    vector<int> matName(bsp.materials.size(), caulk);
    for (int i = 0; i < (int)bsp.materials.size(); ++i) {
        string name = MaterialName(bsp.materials[i]);
        if (name.empty())
            name = "caulk";
        matName[i] = names.Intern(name);
    }

    vector<Ent> ents = ParseEntities(bsp.entities);
    int worldIndex = -1;
    for (int i = 0; i < (int)ents.size(); ++i) {
        if (ClassName(ents[i]) == "worldspawn") {
            worldIndex = i;
            break;
        }
    }
    if (worldIndex < 0) {
        Ent world;
        world.pairs.push_back({ "classname", "worldspawn" });
        ents.insert(ents.begin(), move(world));
        worldIndex = 0;
    }

    AddMissingPrimaryLights(bsp, ents);

    unordered_set<string> lightTargets;
    vector<char> geoEnt(ents.size(), 0);
    for (int i = 0; i < (int)ents.size(); ++i) {
        if (i == worldIndex)
            continue;
        string cls = ClassName(ents[i]);
        if (cls == "misc_model" || cls == "dyn_model" || cls == "info_null")
            geoEnt[i] = 1;
        if (!IsLightClass(cls))
            continue;
        geoEnt[i] = 1;
        string target = KeyOf(ents[i], "target");
        if (!target.empty())
            lightTargets.insert(target);
    }
    for (int i = 0; i < (int)ents.size(); ++i) {
        if (i == worldIndex || !IsLightTargetClass(ClassName(ents[i])))
            continue;
        string targetname = KeyOf(ents[i], "targetname");
        if (!targetname.empty() && lightTargets.count(targetname))
            geoEnt[i] = 1;
    }

    {
        unordered_set<string> targetNames;
        for (const Ent& ent : ents) {
            string targetname = KeyOf(ent, "targetname");
            if (!targetname.empty())
                targetNames.insert(targetname);
        }
        int restored = 0;
        const int lightCount = (int)ents.size();
        for (int i = 0; i < lightCount; ++i) {
            if (!IsLightClass(ClassName(ents[i])))
                continue;

            string target = KeyOf(ents[i], "target");
            if (target.empty())
                continue;

            int targetIdx = FindEntWithTargetname(ents, target);
            if (targetIdx >= 0 && IsLightTargetClass(ClassName(ents[targetIdx])))
                continue;

            const BspPrimaryLight* light = MatchPrimaryLight(bsp, ents[i]);
            Vec3 at{ 0, 0, 0 };
            if (targetIdx >= 0 && !IsLightTargetClass(ClassName(ents[targetIdx]))) {
                if (!light)
                    continue;
                at = OriginOf(ents[targetIdx]);
                string unique = target + "_aim";
                while (targetNames.count(unique))
                    unique += "_";
                SetKey(ents[i], "target", unique);
                target = unique;
            } else if (targetIdx < 0) {
                if (!light)
                    continue;
                Vec3 origin = OriginOf(ents[i]);
                Vec3 dir{ light->dir[0], light->dir[1], light->dir[2] };
                double dirLen = Len(dir);
                if (dirLen < 1e-8)
                    continue;
                dir = Mul(dir, 1.0 / dirLen);
                at = Sub(origin, Mul(dir, 128.0));
            } else {
                continue;
            }

            if (target.empty() || targetNames.count(target))
                continue;

            char originText[128];
            snprintf(originText, sizeof(originText), "%.4f %.4f %.4f", at.x, at.y, at.z);
            Ent marker;
            marker.pairs.push_back({ "origin", originText });
            marker.pairs.push_back({ "targetname", target });
            marker.pairs.push_back({ "classname", "info_null" });
            ents.push_back(move(marker));
            geoEnt.push_back(1);
            targetNames.insert(target);
            lightTargets.insert(target);
            restored++;
        }
        if (restored)
            cout << "restored " << restored << " light targets" << endl;
    }

    vector<Vec3> origins(bsp.models.size(), Vec3{ 0, 0, 0 });
    vector<vector<int>> entModels(ents.size());
    vector<char> claimed(bsp.models.size(), 0);
    if (!claimed.empty())
        claimed[0] = 1;

    auto claim = [&](int entIndex, int model) {
        if (model <= 0 || model >= (int)bsp.models.size() || claimed[model])
            return;
        claimed[model] = 1;
        origins[model] = OriginOf(ents[entIndex]);
        entModels[entIndex].push_back(model);
    };

    for (int i = 0; i < (int)ents.size(); ++i) {
        if (i == worldIndex || DroppedClass(ClassName(ents[i])))
            continue;
        claim(i, StarModel(KeyOf(ents[i], "model")));
        claim(i, StarModel(KeyOf(ents[i], "physicsmodel")));
    }

    vector<int> orphans;
    for (int m = 1; m < (int)bsp.models.size(); ++m)
        if (!claimed[m])
            orphans.push_back(m);

    ExportStats stats;
    stats.version = bsp.version;
    stats.probes = (int)bsp.probes.size();
    if (!orphans.empty())
        stats.warning = "unreferenced brush models were merged into the geo map";

    gDropSides = gDropMat = gDropPlane = gDropFaces = gDropSolid = 0;
    const CollisionSet collisionContents = BuildCollisionContents(bsp);
    vector<ModelGeo> models(bsp.models.size());
    for (int m = 0; m < (int)bsp.models.size(); ++m) {
        cout << "model " << m << "/" << bsp.models.size() << endl;
        models[m] = BuildModel(bsp, m, origins[m], matName, names, caulk, &collisionContents,
                               stats.droppedBrushes, stats.meshes, stats.meshVerts, stats.triangleMeshes);
    }
    cout << "drop reasons sides=" << gDropSides << " mat=" << gDropMat << " plane=" << gDropPlane
         << " faces=" << gDropFaces << " solid=" << gDropSolid << endl;

    ModelGeo world = move(models[0]);
    for (int m : orphans) {
        for (BrushOut& brush : models[m].brushes)
            world.brushes.push_back(move(brush));
        for (MeshOut& mesh : models[m].meshes)
            world.meshes.push_back(move(mesh));
    }
    stats.probes = (int)bsp.probes.size();
    stats.worldBrushes = (int)world.brushes.size();
    for (int i = 0; i < (int)ents.size(); ++i)
        for (int m : entModels[i])
            stats.entityBrushes += (int)models[m].brushes.size();

    vector<BrushOut*> planeOrder;
    for (BrushOut& brush : world.brushes)
        planeOrder.push_back(&brush);
    for (int i = 0; i < (int)ents.size(); ++i) {
        if (!geoEnt[i])
            continue;
        for (int m : entModels[i])
            for (BrushOut& brush : models[m].brushes)
                planeOrder.push_back(&brush);
    }
    for (int i = 0; i < (int)ents.size(); ++i) {
        if (i == worldIndex || geoEnt[i] || DroppedClass(ClassName(ents[i])))
            continue;
        for (int m : entModels[i])
            for (BrushOut& brush : models[m].brushes)
                planeOrder.push_back(&brush);
    }
    SealParsedPlanes(planeOrder);

    fs::path dir = exeDir / "export" / bspPath.stem();
    fs::create_directories(dir);
    fs::path geoPath = dir / (bspPath.stem().wstring() + L"_geo.map");
    fs::path entPath = dir / (bspPath.stem().wstring() + L".map");
    string prefabName = geoPath.filename().string();

    {
        FileOut geo(geoPath);
        WriteHeader(geo.f);
        int entIndex = 0;
        int brushIndex = 0;
        fprintf(geo.f, "// entity %d\n{\n", entIndex++);
        WritePairs(geo.f, ents[worldIndex]);
        WriteGeometry(geo.f, brushIndex, world);
        fputs("}\n", geo.f);

        for (const BspProbe& probe : bsp.probes)
            WriteProbe(geo.f, entIndex++, probe);
        stats.geoEntities = (int)bsp.probes.size();

        for (int i = 0; i < (int)ents.size(); ++i) {
            if (!geoEnt[i])
                continue;
            fprintf(geo.f, "// entity %d\n{\n", entIndex++);
            WritePairs(geo.f, ents[i]);
            brushIndex = 0;
            for (int m : entModels[i])
                WriteGeometry(geo.f, brushIndex, models[m]);
            fputs("}\n", geo.f);
            stats.geoEntities++;
        }
    }

    {
        FileOut ent(entPath);
        WriteHeader(ent.f);
        int entIndex = 0;
        fprintf(ent.f, "// entity %d\n{\n", entIndex++);
        WritePairs(ent.f, ents[worldIndex]);
        fputs("}\n", ent.f);
        fprintf(ent.f, "// entity %d\n{\n", entIndex++);
        fprintf(ent.f, "\"model\" \"%s\"\n", prefabName.c_str());
        fputs("\"origin\" \"0 0 0\"\n", ent.f);
        fputs("\"angles\" \"0 0 0\"\n", ent.f);
        fputs("\"classname\" \"misc_prefab\"\n}\n", ent.f);

        for (int i = 0; i < (int)ents.size(); ++i) {
            if (i == worldIndex || geoEnt[i] || DroppedClass(ClassName(ents[i])))
                continue;
            fprintf(ent.f, "// entity %d\n{\n", entIndex++);
            WritePairs(ent.f, ents[i]);
            int brushIndex = 0;
            for (int m : entModels[i])
                WriteGeometry(ent.f, brushIndex, models[m]);
            fputs("}\n", ent.f);
            stats.entEntities++;
        }
    }

    cout << "wrote " << geoPath.string() << endl;
    cout << "wrote " << entPath.string() << endl;
    return stats;
}
