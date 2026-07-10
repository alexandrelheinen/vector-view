include(CTest)
enable_testing()

set(CATCH_INCLUDE ${PROJECT_SOURCE_DIR}/third_party/catch2)

include_directories(
  ${INCLUDE_DIR}
  ${DSPFILTERS_INCLUDE}
  ${CATCH_INCLUDE})

add_executable(test_contact_utils
  test_contact_utils.cpp)

target_link_libraries(test_contact_utils vectorview_common)

add_test(NAME ContactUtils COMMAND test_contact_utils)

add_executable(test_force_filter
  test_force_filter.cpp)

target_link_libraries(test_force_filter vectorview_filters dspfilters)

add_test(NAME ForceFilter COMMAND test_force_filter)
