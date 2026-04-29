add_library(gps_core STATIC
    src/ca_code.cpp
    src/ephemeris.cpp
    src/nav_message.cpp
    src/modulator.cpp
    src/iq_stream.cpp
)
target_include_directories(gps_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(gps_core PUBLIC cxx_std_20)
