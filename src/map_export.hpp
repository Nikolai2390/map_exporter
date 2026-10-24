#pragma once

#include "common.hpp"

struct ExportStats {
    int version = 0;
    int worldBrushes = 0;
    int entityBrushes = 0;
    int meshes = 0;
    int meshVerts = 0;
    int triangleMeshes = 0;
    int probes = 0;
    int geoEntities = 0;
    int entEntities = 0;
    int droppedBrushes = 0;
    string warning;
};

ExportStats ExportBsp(const fs::path& bspPath, const fs::path& exeDir);
