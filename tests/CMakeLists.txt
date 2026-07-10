set(CATCH_INCLUDE ${PROJECT_SOURCE_DIR}/external/catch2)

include_directories(
  ${INCLUDE_DIR}
  ${DSPFILTERS_INCLUDE}
  ${CATCH_INCLUDE})

set(catch_main catch_main.cpp)

add_executable(test_contact_utils
  ${catch_main}
  test_contact_utils.cpp)

target_link_libraries(test_contact_utils vectorview_common)

add_test(NAME ContactUtils COMMAND $<TARGET_FILE:test_contact_utils>)

add_executable(test_force_filter
  ${catch_main}
  test_force_filter.cpp)

target_link_libraries(test_force_filter vectorview_filters dspfilters)

add_test(NAME ForceFilter COMMAND $<TARGET_FILE:test_force_filter>)
