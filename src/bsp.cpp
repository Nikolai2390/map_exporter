#include "bsp.hpp"

namespace {

[[noreturn]] void Fail(const string& message)
{
    throw runtime_error(message);
}

struct Chunk {
    int type = 0;
    int size = 0;
    uint64_t offset = 0;
};

class Reader {
public:
    explicit Reader(const wstring& path)
    {
        file_.open(path, ios::binary);
        if (!file_)
            Fail("could not open bsp");

        file_.seekg(0, ios::end);
        size_ = static_cast<uint64_t>(file_.tellg());
        file_.seekg(0, ios::beg);
        if (size_ < 12)
            Fail("bsp is truncated");
    }

    void ReadExact(void* dest, uint64_t offset, size_t bytes) const
    {
        if (offset + bytes > size_)
            Fail("bsp lump runs past end of file");
        file_.seekg(static_cast<streamoff>(offset), ios::beg);
        file_.read(reinterpret_cast<char*>(dest), static_cast<streamsize>(bytes));
        if (!file_)
            Fail("failed reading bsp");
    }

    template <typename T>
    T ReadAt(uint64_t offset) const
    {
        T value{};
        ReadExact(&value, offset, sizeof(T));
        return value;
    }

    uint64_t size() const { return size_; }

private:
    mutable ifstream file_;
    uint64_t size_ = 0;
};

// com_bsp_load_obj.cpp Com_GetBspLump: chunk directory once version > 18.
static constexpr int kOldestBspVersion = 6;
static constexpr int kLastFixedDirectoryVersion = 18;
static constexpr int kNewestBspVersion = 37;

// lumpsForVersion, indexed by version - kOldestBspVersion. One entry per fixed-directory version.
static constexpr unsigned kLumpCountByVersion[] = {
    41u, 41u, 42u, 43u, 43u, 43u, 43u, 44u, 44u, 44u, 46u, 46u, 47u
};

// BspHeader::chunks[100]
static constexpr int kChunkDirectoryLimit = 100;

// DiskGfxReflectionProbe is 0x20044 once version > 11. DiskGfxReflectionProbe11
// is 0x20004 for version > 7. Both sizes are origin, then an optional 64-byte
// color-correction name, then the cubemap pixels.
static constexpr unsigned kProbePixelBytes = 131064;
static constexpr unsigned kProbeOriginBytes = 12;
static constexpr unsigned kProbeNameBytes = 64;
static constexpr unsigned kProbeBytes = kProbeOriginBytes + kProbeNameBytes + kProbePixelBytes;
static constexpr unsigned kLegacyProbeBytes = kProbeOriginBytes + kProbePixelBytes;

struct TriSoupV8 {
    unsigned short materialIndex;
    unsigned char lightmapIndex;
    unsigned char reflectionProbeIndex;
    int firstVertex;
    unsigned short vertexCount;
    unsigned short indexCount;
    int firstIndex;
};

struct TriSoupV12 {
    unsigned short materialIndex;
    unsigned char lightmapIndex;
    unsigned char reflectionProbeIndex;
    int vertexLayerData;
    int firstVertex;
    unsigned short vertexCount;
    unsigned short indexCount;
    int firstIndex;
};

static_assert(sizeof(kLumpCountByVersion) / sizeof(kLumpCountByVersion[0])
              == static_cast<size_t>(kLastFixedDirectoryVersion - kOldestBspVersion + 1),
              "lump counts");
static_assert(kProbeBytes == 0x20044, "reflection probe");
static_assert(kLegacyProbeBytes == 0x20004, "reflection probe v11");
static_assert(sizeof(TriSoupV8) == 0x10, "trisoup v8");
static_assert(sizeof(TriSoupV12) == 0x14, "trisoup v12");

vector<Chunk> ReadChunkDirectory(const Reader& reader)
{
    const int chunkCount = reader.ReadAt<int>(8);
    if (chunkCount <= 0 || chunkCount > kChunkDirectoryLimit)
        Fail("bsp chunk count is invalid");

    const uint64_t tableBytes = static_cast<uint64_t>(chunkCount) * 8;
    if (12 + tableBytes > reader.size())
        Fail("bsp chunk table is truncated");

    vector<Chunk> chunks(static_cast<size_t>(chunkCount));
    uint64_t data = 12 + tableBytes;
    for (int i = 0; i < chunkCount; ++i) {
        chunks[i].type = reader.ReadAt<int>(12 + static_cast<uint64_t>(i) * 8);
        chunks[i].size = reader.ReadAt<int>(12 + static_cast<uint64_t>(i) * 8 + 4);
        chunks[i].offset = data;
        if (chunks[i].size < 0)
            Fail("bsp chunk size is negative");
        const uint64_t bytes = static_cast<uint64_t>(chunks[i].size);
        if (data + bytes > reader.size())
            Fail("bsp chunk data runs past end of file");
        data += (bytes + 3u) & ~3u;
    }
    return chunks;
}

vector<Chunk> ReadLumpDirectory(const Reader& reader, int lumpCount)
{
    const uint64_t tableBytes = static_cast<uint64_t>(lumpCount) * 8;
    if (8 + tableBytes > reader.size())
        Fail("bsp lump table is truncated");

    vector<Chunk> chunks(static_cast<size_t>(lumpCount));
    for (int i = 0; i < lumpCount; ++i) {
        const uint64_t at = 8 + static_cast<uint64_t>(i) * 8;
        const uint32_t length = reader.ReadAt<uint32_t>(at);
        const uint32_t offset = reader.ReadAt<uint32_t>(at + 4);
        if (length > 0x7fffffffu)
            Fail("bsp lump size is invalid");
        chunks[i].type = i;
        chunks[i].size = static_cast<int>(length);
        chunks[i].offset = offset;
        if (length != 0) {
            const uint64_t end = static_cast<uint64_t>(offset) + length;
            if (end < offset || end > reader.size())
                Fail("bsp lump data runs past end of file");
        }
    }
    return chunks;
}

vector<Chunk> ReadChunks(const Reader& reader, int version)
{
    // Com_GetBspLump uses the chunk directory once version > kLastFixedDirectoryVersion.
    if (version > kLastFixedDirectoryVersion)
        return ReadChunkDirectory(reader);
    return ReadLumpDirectory(reader, static_cast<int>(kLumpCountByVersion[version - kOldestBspVersion]));
}

const Chunk* FindChunk(const vector<Chunk>& chunks, int type)
{
    for (const Chunk& chunk : chunks) {
        if (chunk.type == type)
            return &chunk;
    }
    return nullptr;
}

template <typename T>
vector<T> ReadArray(const Reader& reader, const vector<Chunk>& chunks, int type)
{
    const Chunk* chunk = FindChunk(chunks, type);
    if (!chunk || chunk->size == 0)
        return {};
    if (chunk->size % static_cast<int>(sizeof(T)) != 0) {
        Fail("lump " + to_string(type) + " size " + to_string(chunk->size)
             + " is not a multiple of " + to_string(sizeof(T)));
    }
    vector<T> out(static_cast<size_t>(chunk->size / sizeof(T)));
    reader.ReadExact(out.data(), chunk->offset, out.size() * sizeof(T));
    return out;
}

BspTriSoup MakeTriSoup(
    unsigned short materialIndex,
    unsigned char lightmapIndex,
    unsigned char reflectionProbeIndex,
    int vertexLayerData,
    int firstVertex,
    unsigned short vertexCount,
    unsigned short indexCount,
    int firstIndex)
{
    BspTriSoup soup{};
    soup.materialIndex = materialIndex;
    soup.lightmapIndex = lightmapIndex;
    soup.reflectionProbeIndex = reflectionProbeIndex;
    soup.primaryLightIndex = 1;
    soup.vertexLayerData = vertexLayerData;
    soup.firstVertex = firstVertex;
    soup.vertexCount = vertexCount;
    soup.indexCount = indexCount;
    soup.firstIndex = firstIndex;
    return soup;
}

vector<BspTriSoup> ReadTriangles(const Reader& reader, const vector<Chunk>& chunks, int type, int version)
{
    // R_LoadTriangleSurfaces: 24 bytes above version 12, 20 above version 8, else 16.
    if (version > 12)
        return ReadArray<BspTriSoup>(reader, chunks, type);

    if (version > 8) {
        const vector<TriSoupV12> old = ReadArray<TriSoupV12>(reader, chunks, type);
        vector<BspTriSoup> out;
        out.reserve(old.size());
        for (const TriSoupV12& tri : old) {
            out.push_back(MakeTriSoup(
                tri.materialIndex, tri.lightmapIndex, tri.reflectionProbeIndex,
                tri.vertexLayerData, tri.firstVertex, tri.vertexCount, tri.indexCount, tri.firstIndex));
        }
        return out;
    }

    const vector<TriSoupV8> old = ReadArray<TriSoupV8>(reader, chunks, type);
    vector<BspTriSoup> out;
    out.reserve(old.size());
    for (const TriSoupV8& tri : old) {
        out.push_back(MakeTriSoup(
            tri.materialIndex, tri.lightmapIndex, tri.reflectionProbeIndex,
            0, tri.firstVertex, tri.vertexCount, tri.indexCount, tri.firstIndex));
    }
    return out;
}

unsigned ProbeBytes(int version)
{
    // R_LoadReflectionProbes.
    if (version > 11)
        return kProbeBytes;
    if (version > 7)
        return kLegacyProbeBytes;
    return 0;
}

} // namespace

string MaterialName(const BspMaterial& material)
{
    char name[65] = {};
    memcpy(name, material.name, 64);
    return string(name);
}

BspFile LoadBsp(const wstring& path)
{
    static_assert(sizeof(BspMaterial) == 72, "material");
    static_assert(sizeof(BspPlane) == 16, "plane");
    static_assert(sizeof(BspBrushSide) == 8, "brush side");
    static_assert(sizeof(BspBrush) == 4, "brush");
    static_assert(sizeof(BspTriSoup) == 24, "trisoup");
    static_assert(sizeof(BspDrawVert) == 68, "draw vert");
    static_assert(sizeof(BspModel) == 48, "model");
    static_assert(sizeof(BspCollisionVert) == 12, "collision vert");
    static_assert(sizeof(BspCollisionTri) == 6, "collision tri");
    static_assert(sizeof(BspCollisionPart) == 12, "collision partition");
    static_assert(sizeof(BspPortal) == 16, "portal");
    static_assert(sizeof(BspPortalVert) == 12, "portal vert");

    Reader reader(path);
    const int ident = reader.ReadAt<int>(0);
    const int version = reader.ReadAt<int>(4);
    if (ident != kBspIdent)
        Fail("not an IBSP file");
    if (version < kOldestBspVersion || version > kNewestBspVersion)
        Fail("bsp version " + to_string(version) + " is outside "
             + to_string(kOldestBspVersion) + ".." + to_string(kNewestBspVersion));

    const vector<Chunk> chunks = ReadChunks(reader, version);

    BspFile bsp;
    bsp.version = version;
    bsp.materials = ReadArray<BspMaterial>(reader, chunks, kLumpMaterials);
    bsp.planes = ReadArray<BspPlane>(reader, chunks, kLumpPlanes);
    bsp.portalVerts = ReadArray<BspPortalVert>(reader, chunks, kLumpPortalVerts);
    bsp.portals = ReadArray<BspPortal>(reader, chunks, kLumpPortals);
    bsp.sides = ReadArray<BspBrushSide>(reader, chunks, kLumpBrushSides);
    bsp.edgeCounts = ReadArray<unsigned char>(reader, chunks, kLumpBrushSideEdgeCounts);
    bsp.edges = ReadArray<unsigned char>(reader, chunks, kLumpBrushEdges);
    bsp.brushes = ReadArray<BspBrush>(reader, chunks, kLumpBrushes);
    bsp.soups[0] = ReadTriangles(reader, chunks, kLumpTriangles, version);
    bsp.soups[1] = ReadTriangles(reader, chunks, kLumpSimpleTriangles, version);
    bsp.verts[0] = ReadArray<BspDrawVert>(reader, chunks, kLumpDrawVerts);
    bsp.verts[1] = ReadArray<BspDrawVert>(reader, chunks, kLumpSimpleDrawVerts);
    bsp.indices[0] = ReadArray<unsigned short>(reader, chunks, kLumpDrawIndices);
    bsp.indices[1] = ReadArray<unsigned short>(reader, chunks, kLumpSimpleDrawIndices);
    bsp.models = ReadArray<BspModel>(reader, chunks, kLumpModels);
    bsp.collisionAabbs = ReadArray<BspCollisionAabb>(reader, chunks, kLumpCollisionAabbs);
    bsp.collisionVerts = ReadArray<BspCollisionVert>(reader, chunks, kLumpCollisionVerts);
    bsp.collisionTris = ReadArray<BspCollisionTri>(reader, chunks, kLumpCollisionTris);
    bsp.collisionParts = ReadArray<BspCollisionPart>(reader, chunks, kLumpCollisionPartitions);

    if (const Chunk* leaf = FindChunk(chunks, kLumpLeafs)) {
        int record = 0;
        int firstOffset = 0;
        if (version >= 15 && leaf->size % 24 == 0) {
            record = 24;
            firstOffset = 4;
        } else if (version < 15 && leaf->size % 36 == 0) {
            record = 36;
            firstOffset = 8;
        }
        if (record && leaf->size > 0) {
            vector<unsigned char> raw(static_cast<size_t>(leaf->size));
            reader.ReadExact(raw.data(), leaf->offset, raw.size());
            const int count = leaf->size / record;
            bsp.leafColl.resize(static_cast<size_t>(count));
            for (int i = 0; i < count; ++i) {
                const unsigned char* at = raw.data() + static_cast<size_t>(i) * record + firstOffset;
                memcpy(&bsp.leafColl[i].firstCollAabb, at, 4);
                memcpy(&bsp.leafColl[i].collCount, at + 4, 4);
            }
        }
    }

    if (const Chunk* ents = FindChunk(chunks, kLumpEntities)) {
        if (ents->size > 0) {
            vector<char> text(static_cast<size_t>(ents->size));
            reader.ReadExact(text.data(), ents->offset, text.size());
            bsp.entities.assign(text.begin(), text.end());
            const auto end = bsp.entities.find('\0');
            if (end != string::npos)
                bsp.entities.resize(end);
        }
    }

    if (const unsigned probeBytes = ProbeBytes(version)) {
        if (const Chunk* probes = FindChunk(chunks, kLumpReflectionProbes)) {
            if (probes->size > 0) {
                if (probes->size % static_cast<int>(probeBytes) != 0) {
                    Fail("reflection probe lump size " + to_string(probes->size)
                         + " is not a multiple of " + to_string(probeBytes));
                }
                const int count = probes->size / static_cast<int>(probeBytes);
                const unsigned headerBytes = version > 11 ? kProbeOriginBytes + kProbeNameBytes : kProbeOriginBytes;
                bsp.probes.resize(static_cast<size_t>(count));
                for (int i = 0; i < count; ++i) {
                    unsigned char head[kProbeOriginBytes + kProbeNameBytes] = {};
                    reader.ReadExact(head, probes->offset + static_cast<uint64_t>(i) * probeBytes, headerBytes);
                    memcpy(bsp.probes[i].origin, head, kProbeOriginBytes);
                    if (version > 11)
                        memcpy(bsp.probes[i].colorCorrection, head + kProbeOriginBytes, kProbeNameBytes);
                }
            }
        }
    }

    if (version >= 17) {
        if (const Chunk* lights = FindChunk(chunks, kLumpPrimaryLights)) {
            if (lights->size > 0) {
                if (lights->size % static_cast<int>(sizeof(BspPrimaryLight)) != 0)
                    Fail("primary light lump size is not a multiple of 128");
                const int count = lights->size / static_cast<int>(sizeof(BspPrimaryLight));
                bsp.primaryLights.resize(static_cast<size_t>(count));
                reader.ReadExact(bsp.primaryLights.data(), lights->offset,
                                 static_cast<size_t>(lights->size));
            }
        }
    }

    if (bsp.models.empty())
        Fail("bsp has no models");

    int sideCursor = 0;
    for (const BspBrush& brush : bsp.brushes) {
        if (sideCursor > (int)bsp.sides.size()
            || brush.numSides > (int)bsp.sides.size() - sideCursor)
            Fail("brush sides overflow the side lump");
        sideCursor += brush.numSides;
    }
    if (sideCursor != (int)bsp.sides.size())
        Fail("brush side count does not match the side lump");

    return bsp;
}
