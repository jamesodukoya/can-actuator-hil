add_executable(sbw_tests
  test_smoke.cpp)
target_link_libraries(sbw_tests PRIVATE GTest::gtest_main)
sbw_set_test_warnings(sbw_tests)
include(GoogleTest)
gtest_discover_tests(sbw_tests)
