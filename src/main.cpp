#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>

#include "map_export.hpp"

namespace {

string Narrow(const wstring& text)
{
    if (text.empty())
        return {};
    const int bytes = WideCharToMultiByte(CP_ACP, 0, text.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (bytes <= 1)
        return {};
    string out(static_cast<size_t>(bytes - 1), '\0');
    WideCharToMultiByte(CP_ACP, 0, text.c_str(), -1, out.data(), bytes, nullptr, nullptr);
    return out;
}

void Show(const wstring& text, bool error)
{
    cout << Narrow(text) << endl;
    MessageBoxW(nullptr, text.c_str(), L"map_exporter", error ? MB_ICONERROR | MB_OK : MB_OK);
}

fs::path ExeDir()
{
    wchar_t buffer[MAX_PATH];
    const DWORD length = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (length == 0 || length >= MAX_PATH)
        throw runtime_error("could not resolve the exporter path");
    return fs::path(buffer).parent_path();
}

} // namespace

int main()
{
    int argc = 0;
    LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    if (!argv) {
        Show(L"Could not read the command line.", true);
        return 1;
    }

    if (argc < 2) {
        LocalFree(argv);
        Show(L"Drop one or more .d3dbsp files onto map_exporter.exe.\n\n"
             L"Each map is written to export\\<name>\\ next to the exe:\n"
             L"  <name>_geo.map  brushes, terrain, lights, reflection probes\n"
             L"  <name>.map      misc_prefab of the geo plus everything else",
             false);
        return 1;
    }

    const fs::path exeDir = ExeDir();
    wstring report;
    int failed = 0;

    for (int i = 1; i < argc; ++i) {
        const fs::path bspPath(argv[i]);
        try {
            const ExportStats stats = ExportBsp(bspPath, exeDir);
            report += bspPath.filename().wstring();
            report += L"  (version " + to_wstring(stats.version) + L")\n";
            report += L"  world brushes " + to_wstring(stats.worldBrushes);
            report += L", entity brushes " + to_wstring(stats.entityBrushes);
            report += L", meshes " + to_wstring(stats.meshes);
            report += L" (" + to_wstring(stats.meshVerts) + L" verts, ";
            report += to_wstring(stats.triangleMeshes) + L" triangle pieces)\n";
            report += L"  probes " + to_wstring(stats.probes);
            report += L", geo ents " + to_wstring(stats.geoEntities);
            report += L", map ents " + to_wstring(stats.entEntities) + L"\n";
            if (stats.droppedBrushes)
                report += L"  dropped degenerate brushes " + to_wstring(stats.droppedBrushes) + L"\n";
            if (!stats.warning.empty())
                report += L"  " + wstring(stats.warning.begin(), stats.warning.end()) + L"\n";
            report += L"  export\\" + bspPath.stem().wstring() + L"\\\n\n";
        } catch (const exception& ex) {
            ++failed;
            report += bspPath.filename().wstring();
            report += L" failed: ";
            report += wstring(ex.what(), ex.what() + strlen(ex.what()));
            report += L"\n\n";
        }
    }

    LocalFree(argv);
    Show(report, failed != 0);
    return failed ? 1 : 0;
}
