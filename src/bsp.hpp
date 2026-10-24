#pragma once

#include "common.hpp"

// CoD d3dbsp. Versions through 18 store a fixed lump directory. Versions 19
// through 37 store a chunk directory. Record sizes follow the version checks
// in the engine loaders; this exporter reads the same lumps either way.

static constexpr int kBspIdent = ('P' << 24) | ('S' << 16) | ('B' << 8) | 'I';

enum BspLump : int {
    kLumpMaterials = 0,
    kLumpPlanes = 4,
    kLumpBrushSides = 5,
    kLumpBrushSideEdgeCounts = 6,
    kLumpBrushEdges = 7,
    kLumpBrushes = 8,
    kLumpTriangles = 9,
    kLumpDrawVerts = 10,
    kLumpDrawIndices = 11,
    kLumpCollisionAabbs = 36,
    kLumpModels = 37,
    kLumpEntities = 39,
    kLumpReflectionProbes = 41,
    kLumpPrimaryLights = 43,
    kLumpLightGridHeader = 44,
    kLumpSimpleTriangles = 47,
    kLumpSimpleDrawVerts = 48,
    kLumpSimpleDrawIndices = 49
};

static constexpr int kContentsDetail = 0x08000000;
static constexpr int kContentsNonColliding = 0x00000004;
static constexpr int kContentsMissileClip = 0x00000080;
static constexpr int kContentsClipShot = 0x00002000;
static constexpr unsigned kSurfNodraw = 0x00000080;
static constexpr unsigned kSurfPortal = 0x80000000;
static constexpr double kPatchTexScale = 1024.0;

struct BspMaterial {
    char name[64];
    unsigned surfaceFlags;
    unsigned contentFlags;
};

struct BspPlane {
    float normal[3];
    float dist;
};

struct BspBrushSide {
    unsigned planeOrDist;
    unsigned materialNum;
};

struct BspBrush {
    unsigned short numSides;
    unsigned short materialNum;
};

struct BspTriSoup {
    unsigned short materialIndex;
    unsigned char lightmapIndex;
    unsigned char reflectionProbeIndex;
    unsigned char primaryLightIndex;
    unsigned char vertexLayerType;
    unsigned short pad;
    int vertexLayerData;
    int firstVertex;
    unsigned short vertexCount;
    unsigned short indexCount;
    int firstIndex;
};

struct BspDrawVert {
    float xyz[3];
    float normal[3];
    unsigned char color[4];
    float texCoord[2];
    float lmapCoord[2];
    float tangent[3];
    float binormal[3];
};

struct BspModel {
    float mins[3];
    float maxs[3];
    unsigned short firstTriSoup[2];
    unsigned short triSoupCount[2];
    int firstSurface;
    int numSurfaces;
    int firstBrush;
    int numBrushes;
};

struct BspCollisionAabb {
    float midPoint[3];
    float halfSize[3];
    unsigned short materialIndex;
    unsigned short childCount;
    int u;
};
static_assert(sizeof(BspCollisionAabb) == 32, "collision aabb");

struct BspProbe {
    float origin[3];
    char colorCorrection[64];
};

struct BspPrimaryLight {
    unsigned char type;
    unsigned char canUseShadowMap;
    unsigned char exponent;
    unsigned char unused;
    float color[3];
    float dir[3];
    float origin[3];
    float radius;
    float cosHalfFovOuter;
    float cosHalfFovInner;
    float cosHalfFovExpanded;
    float rotationLimit;
    float translationLimit;
    char defName[64];
};
static_assert(sizeof(BspPrimaryLight) == 128, "primary light");

struct BspFile {
    int version = 0;
    vector<BspMaterial> materials;
    vector<BspPlane> planes;
    vector<BspBrushSide> sides;
    vector<unsigned char> edgeCounts;
    vector<unsigned char> edges;
    vector<BspBrush> brushes;
    vector<BspTriSoup> soups[2];
    vector<BspDrawVert> verts[2];
    vector<unsigned short> indices[2];
    vector<BspModel> models;
    vector<BspCollisionAabb> collisionAabbs;
    string entities;
    vector<BspProbe> probes;
    vector<BspPrimaryLight> primaryLights;
    bool hasLightGrid = false;
    unsigned short lightGridMins[3] = {};
    unsigned short lightGridMaxs[3] = {};
};

BspFile LoadBsp(const wstring& path);
string MaterialName(const BspMaterial& material);
