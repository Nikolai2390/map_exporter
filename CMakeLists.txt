cmake_minimum_required(VERSION 3.16)
project(map_exporter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(map_exporter
    src/main.cpp
    src/bsp.cpp
    src/map_export.cpp
)

target_include_directories(map_exporter PRIVATE src)

if(MSVC)
    target_compile_definitions(map_exporter PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    target_compile_options(map_exporter PRIVATE /W3 /EHsc)
else()
    target_compile_options(map_exporter PRIVATE -O2 -Wall)
endif()

target_link_libraries(map_exporter PRIVATE shell32)

set_target_properties(map_exporter PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/bin"
)
