cmake_minimum_required(VERSION 3.16)
project(vector-view VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(VECTOR_VIEW vector_view)
set(VECTOR_GUI vector_gui)

option(BUILD_VECTOR_GUI "Build the Qt Vector GUI application" ON)
option(BUILD_VECTOR_VIEW "Build the Gazebo Sim Vector View system plugin" ON)
option(BUILD_TESTS "Build unit tests" ON)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(DSPFILTERS_INCLUDE ${PROJECT_SOURCE_DIR}/external/dspfilters/include)
set(QCUSTOMPLOT_INCLUDE ${PROJECT_SOURCE_DIR}/external/qcustomplot)

set(dspfilters_sources
  external/dspfilters/src/Bessel.cpp
  external/dspfilters/src/Biquad.cpp
  external/dspfilters/src/Butterworth.cpp
  external/dspfilters/src/Cascade.cpp
  external/dspfilters/src/ChebyshevI.cpp
  external/dspfilters/src/ChebyshevII.cpp
  external/dspfilters/src/Custom.cpp
  external/dspfilters/src/Design.cpp
  external/dspfilters/src/Documentation.cpp
  external/dspfilters/src/Elliptic.cpp
  external/dspfilters/src/Filter.cpp
  external/dspfilters/src/Legendre.cpp
  external/dspfilters/src/Param.cpp
  external/dspfilters/src/PoleFilter.cpp
  external/dspfilters/src/RBJ.cpp
  external/dspfilters/src/RootFinder.cpp
  external/dspfilters/src/State.cpp
)

set(forcefilter_sources src/filters/ForceFilter.cpp)
set(common_sources
  src/common/ContactUtils.cpp
  src/common/TopicPath.cpp)
set(gz_common_sources src/common/ContactMessage.cpp)
set(plugin_sources
  src/plugin/VectorView.cpp
  ${gz_common_sources})
set(gui_sources
  include/vectorview/Interface.h
  src/gui/VectorGUI.cpp
  src/gui/Interface.cpp
  ${gz_common_sources}
  external/qcustomplot/qcustomplot.cpp
)

if(BUILD_VECTOR_GUI)
  find_package(Qt6 QUIET COMPONENTS Widgets PrintSupport Xml Core)
  if(NOT Qt6_FOUND)
    message(WARNING
      "Qt6 not found; disabling BUILD_VECTOR_GUI. "
      "Install qt6-base-dev or pass -DBUILD_VECTOR_GUI=OFF for a tests-only build.")
    set(BUILD_VECTOR_GUI OFF CACHE BOOL "Build the Qt Vector GUI application" FORCE)
  endif()
endif()

if(BUILD_VECTOR_VIEW OR BUILD_VECTOR_GUI)
  find_package(gz-sim8 QUIET)
  find_package(gz-plugin2 QUIET COMPONENTS register)
  find_package(gz-transport13 QUIET)
  find_package(gz-msgs10 QUIET)
  find_package(gz-math7 QUIET)

  if(NOT gz-sim8_FOUND OR NOT gz-transport13_FOUND OR NOT gz-msgs10_FOUND)
    message(WARNING
      "Gazebo Harmonic (gz-sim8) not found; disabling BUILD_VECTOR_VIEW and BUILD_VECTOR_GUI. "
      "Install gz-harmonic or pass -DBUILD_VECTOR_VIEW=OFF -DBUILD_VECTOR_GUI=OFF "
      "for a tests-only build.")
    set(BUILD_VECTOR_VIEW OFF CACHE BOOL "Build the Gazebo Sim Vector View system plugin" FORCE)
    set(BUILD_VECTOR_GUI OFF CACHE BOOL "Build the Qt Vector GUI application" FORCE)
  endif()
endif()

if(BUILD_VECTOR_VIEW OR BUILD_VECTOR_GUI OR BUILD_TESTS)
  add_library(vector_view_common STATIC ${common_sources})
  target_include_directories(vector_view_common
    PUBLIC
      ${INCLUDE_DIR}
      ${DSPFILTERS_INCLUDE})

  add_library(dspfilters SHARED ${dspfilters_sources})
  target_include_directories(dspfilters PUBLIC ${DSPFILTERS_INCLUDE})

  add_library(vector_view_filters SHARED ${forcefilter_sources})
  target_link_libraries(vector_view_filters PUBLIC dspfilters)
  target_include_directories(vector_view_filters
    PUBLIC
      ${INCLUDE_DIR}
      ${DSPFILTERS_INCLUDE})
endif()

if(BUILD_VECTOR_VIEW)
  add_library(${VECTOR_VIEW} SHARED ${plugin_sources})
  set_target_properties(${VECTOR_VIEW} PROPERTIES OUTPUT_NAME "vector-view")
  target_include_directories(${VECTOR_VIEW}
    PRIVATE
      ${INCLUDE_DIR}
      ${DSPFILTERS_INCLUDE})
  target_link_libraries(${VECTOR_VIEW}
    PRIVATE
      vector_view_common
      vector_view_filters
      dspfilters
      gz-sim8::gz-sim8
      gz-plugin2::register
      gz-transport13::gz-transport13
      gz-msgs10::gz-msgs10
      gz-math7::gz-math7)
endif()

if(BUILD_VECTOR_GUI)
  set(CMAKE_AUTOMOC ON)
  add_executable(${VECTOR_GUI} ${gui_sources})
  set_target_properties(${VECTOR_GUI} PROPERTIES OUTPUT_NAME "vector-gui")
  target_include_directories(${VECTOR_GUI}
    PRIVATE
      ${INCLUDE_DIR}
      ${QCUSTOMPLOT_INCLUDE}
      ${DSPFILTERS_INCLUDE})
  target_link_libraries(${VECTOR_GUI}
    PRIVATE
      vector_view_common
      vector_view_filters
      dspfilters
      Qt6::Widgets
      Qt6::PrintSupport
      Qt6::Xml
      Qt6::Core
      gz-transport13::gz-transport13
      gz-msgs10::gz-msgs10
      gz-math7::gz-math7)
endif()

if(BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()

if(NOT BUILD_VECTOR_VIEW AND NOT BUILD_VECTOR_GUI AND NOT BUILD_TESTS)
  message(FATAL_ERROR
    "Nothing to build. Enable at least one of: "
    "BUILD_VECTOR_VIEW, BUILD_VECTOR_GUI, BUILD_TESTS.")
endif()

set(INSTALL_LIBDIR lib)
set(INSTALL_BINDIR bin)
set(INSTALL_DATADIR share/vector-view)

if(BUILD_VECTOR_VIEW OR BUILD_VECTOR_GUI OR BUILD_TESTS)
  install(TARGETS dspfilters vector_view_filters vector_view_common
    LIBRARY DESTINATION ${INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${INSTALL_LIBDIR}
    RUNTIME DESTINATION ${INSTALL_BINDIR})
endif()

if(BUILD_VECTOR_VIEW)
  install(TARGETS ${VECTOR_VIEW}
    LIBRARY DESTINATION ${INSTALL_LIBDIR}
    RUNTIME DESTINATION ${INSTALL_BINDIR})
endif()

if(BUILD_VECTOR_GUI)
  install(TARGETS ${VECTOR_GUI}
    RUNTIME DESTINATION ${INSTALL_BINDIR})
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/models/
  DESTINATION ${INSTALL_DATADIR}/models)
install(FILES
  ${PROJECT_SOURCE_DIR}/worlds/robot.world
  ${PROJECT_SOURCE_DIR}/worlds/release_demo.world
  DESTINATION ${INSTALL_DATADIR}/worlds)
