add_executable(sbw_tests
  test_alive_counter.cpp
  test_can_protocol.cpp
  test_crc8.cpp)
target_link_libraries(sbw_tests PRIVATE sbw_core GTest::gtest_main)
sbw_set_test_warnings(sbw_tests)
include(GoogleTest)
gtest_discover_tests(sbw_tests)
