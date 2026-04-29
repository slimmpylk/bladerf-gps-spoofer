cmake_minimum_required(VERSION 3.20)
project(bladerf_gps_spoofer VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
else()
    add_compile_options(-Wall -Wextra -O3)
endif()

find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)
find_package(PkgConfig REQUIRED)

# libbladeRF is OPTIONAL so the core library and tests can build
# on machines without bladeRF hardware/drivers installed.
# The main gps-spoofer executable requires it.
pkg_check_modules(BLADERF libbladerf)

# Core signal generation library — no hardware deps, always builds
add_subdirectory(core)
add_subdirectory(tests)

if(BLADERF_FOUND)
    message(STATUS "libbladeRF found ${BLADERF_VERSION} — building gps-spoofer")
    add_subdirectory(bladerf)

    add_executable(gps-spoofer main.cpp)
    target_include_directories(gps-spoofer PRIVATE
            ${CMAKE_SOURCE_DIR}/core/include
            ${CMAKE_SOURCE_DIR}/bladerf/include
    )
    target_link_libraries(gps-spoofer PRIVATE
            gps_core
            gps_bladerf
            nlohmann_json::nlohmann_json
    )
else()
    message(STATUS "libbladeRF NOT found — skipping gps-spoofer executable")
    message(STATUS "Install with: sudo apt install libbladerf-dev")
endif()