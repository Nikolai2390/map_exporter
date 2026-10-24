# map_exporter

Exports Call of Duty `.d3dbsp` maps (versions 6-22) to Radiant `.map` files.

Drop one or more `.d3dbsp` files onto `map_exporter.exe`, or pass them on the command line:

```
map_exporter.exe path\to\map.d3dbsp
```

Each map is written next to the exe:

```
export\<name>\<name>_geo.map   brushes, terrain, lights, reflection probes
export\<name>\<name>.map       misc_prefab of the geo map, plus the other entities
```

## Build

Requires CMake and Visual Studio 2022.

```
generate_project.bat
```

Open `build\map_exporter.sln` and build Release. The exe is `bin\map_exporter.exe`.
