enable_testing()

add_executable(test_ca_code test_ca_code.cpp)
target_link_libraries(test_ca_code PRIVATE gps_core GTest::gtest_main)
target_include_directories(test_ca_code PRIVATE ${CMAKE_SOURCE_DIR}/core/include)

add_executable(test_ephemeris test_ephemeris.cpp)
target_link_libraries(test_ephemeris PRIVATE gps_core GTest::gtest_main)
target_include_directories(test_ephemeris PRIVATE ${CMAKE_SOURCE_DIR}/core/include)

add_executable(test_nav_message test_nav_message.cpp)
target_link_libraries(test_nav_message PRIVATE gps_core GTest::gtest_main)
target_include_directories(test_nav_message PRIVATE ${CMAKE_SOURCE_DIR}/core/include)

add_executable(test_modulator test_modulator.cpp)
target_link_libraries(test_modulator PRIVATE gps_core GTest::gtest_main)
target_include_directories(test_modulator PRIVATE ${CMAKE_SOURCE_DIR}/core/include)

include(GoogleTest)
gtest_discover_tests(test_ca_code)
gtest_discover_tests(test_ephemeris)
gtest_discover_tests(test_nav_message)
gtest_discover_tests(test_modulator)

add_executable(test_iq_stream test_iq_stream.cpp)
target_link_libraries(test_iq_stream PRIVATE gps_core GTest::gtest_main)
target_include_directories(test_iq_stream PRIVATE ${CMAKE_SOURCE_DIR}/core/include)
gtest_discover_tests(test_iq_stream)