add_library(gps_bladerf STATIC src/bladerf_tx.cpp)
target_include_directories(gps_bladerf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gps_bladerf PUBLIC gps_core)
