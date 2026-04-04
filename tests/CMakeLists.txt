cmake_minimum_required(VERSION 3.10)

find_package(GTest REQUIRED)
if(NOT TARGET GTest::gtest_main)
  message(FATAL_ERROR "GTest::gtest_main target not found. Please install libgtest-dev.")
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(IGNITION_MATH ignition-math6 REQUIRED)

set(ROOT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# ── FindName tests (pure string logic; no Gazebo or ignition dependency) ──────
add_executable(test_findname
  test_findname.cpp
  ../src/vectorview/NameUtils.cpp
)
target_include_directories(test_findname PRIVATE ${ROOT_INCLUDE})
target_link_libraries(test_findname PRIVATE GTest::gtest_main)
add_test(NAME FindName COMMAND test_findname)

# ── ForceFilter tests (needs ignition-math and DspFilters) ───────────────────
add_executable(test_forcefilter
  test_forcefilter.cpp
)
target_include_directories(test_forcefilter
  PRIVATE ${ROOT_INCLUDE}
          ${IGNITION_MATH_INCLUDE_DIRS})
target_link_libraries(test_forcefilter
  PRIVATE DspFilters
          GTest::gtest_main
          ${IGNITION_MATH_LIBRARIES})
add_test(NAME ForceFilter COMMAND test_forcefilter)
