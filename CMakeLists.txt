cmake_minimum_required(VERSION 3.16)
project(vectorview VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECTNAME vectorview)
set(GUINAME vectorGUI)

option(BUILD_VECTORGUI "Build the Qt VectorGUI application" ON)
option(BUILD_VECTORVIEW "Build the Gazebo Sim VectorView system plugin" ON)
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

if(BUILD_VECTORGUI)
  find_package(Qt6 QUIET COMPONENTS Widgets PrintSupport Xml Core)
  if(NOT Qt6_FOUND)
    message(WARNING
      "Qt6 not found — disabling BUILD_VECTORGUI. "
      "Install qt6-base-dev or pass -DBUILD_VECTORGUI=OFF for a tests-only build.")
    set(BUILD_VECTORGUI OFF CACHE BOOL "Build the Qt VectorGUI application" FORCE)
  endif()
endif()

if(BUILD_VECTORVIEW OR BUILD_VECTORGUI)
  find_package(gz-sim8 QUIET)
  find_package(gz-plugin2 QUIET COMPONENTS register)
  find_package(gz-transport13 QUIET)
  find_package(gz-msgs10 QUIET)
  find_package(gz-math7 QUIET)

  if(NOT gz-sim8_FOUND OR NOT gz-transport13_FOUND OR NOT gz-msgs10_FOUND)
    message(WARNING
      "Gazebo Harmonic (gz-sim8) not found — disabling BUILD_VECTORVIEW and BUILD_VECTORGUI. "
      "Install gz-harmonic or pass -DBUILD_VECTORVIEW=OFF -DBUILD_VECTORGUI=OFF "
      "for a tests-only build.")
    set(BUILD_VECTORVIEW OFF CACHE BOOL "Build the Gazebo Sim VectorView system plugin" FORCE)
    set(BUILD_VECTORGUI OFF CACHE BOOL "Build the Qt VectorGUI application" FORCE)
  endif()
endif()

if(BUILD_VECTORVIEW OR BUILD_VECTORGUI OR BUILD_TESTS)
  add_library(vectorview_common STATIC ${common_sources})
  target_include_directories(vectorview_common
    PUBLIC
      ${INCLUDE_DIR}
      ${DSPFILTERS_INCLUDE})

  add_library(dspfilters SHARED ${dspfilters_sources})
  target_include_directories(dspfilters PUBLIC ${DSPFILTERS_INCLUDE})

  add_library(vectorview_filters SHARED ${forcefilter_sources})
  target_link_libraries(vectorview_filters PUBLIC dspfilters)
  target_include_directories(vectorview_filters
    PUBLIC
      ${INCLUDE_DIR}
      ${DSPFILTERS_INCLUDE})
endif()

if(BUILD_VECTORVIEW)
  add_library(${PROJECTNAME} SHARED ${plugin_sources})
  target_include_directories(${PROJECTNAME}
    PRIVATE
      ${INCLUDE_DIR}
      ${DSPFILTERS_INCLUDE})
  target_link_libraries(${PROJECTNAME}
    PRIVATE
      vectorview_common
      vectorview_filters
      dspfilters
      gz-sim8::gz-sim8
      gz-plugin2::register
      gz-transport13::gz-transport13
      gz-msgs10::gz-msgs10
      gz-math7::gz-math7)
endif()

if(BUILD_VECTORGUI)
  set(CMAKE_AUTOMOC ON)
  add_executable(${GUINAME} ${gui_sources})
  target_include_directories(${GUINAME}
    PRIVATE
      ${INCLUDE_DIR}
      ${QCUSTOMPLOT_INCLUDE}
      ${DSPFILTERS_INCLUDE})
  target_link_libraries(${GUINAME}
    PRIVATE
      vectorview_common
      vectorview_filters
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

if(NOT BUILD_VECTORVIEW AND NOT BUILD_VECTORGUI AND NOT BUILD_TESTS)
  message(FATAL_ERROR
    "Nothing to build. Enable at least one of: "
    "BUILD_VECTORVIEW, BUILD_VECTORGUI, BUILD_TESTS.")
endif()

set(INSTALL_LIBDIR lib)
set(INSTALL_BINDIR bin)
set(INSTALL_DATADIR share/vectorview)

if(BUILD_VECTORVIEW OR BUILD_VECTORGUI OR BUILD_TESTS)
  install(TARGETS dspfilters vectorview_filters vectorview_common
    LIBRARY DESTINATION ${INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${INSTALL_LIBDIR}
    RUNTIME DESTINATION ${INSTALL_BINDIR})
endif()

if(BUILD_VECTORVIEW)
  install(TARGETS ${PROJECTNAME}
    LIBRARY DESTINATION ${INSTALL_LIBDIR}
    RUNTIME DESTINATION ${INSTALL_BINDIR})
endif()

if(BUILD_VECTORGUI)
  install(TARGETS ${GUINAME}
    RUNTIME DESTINATION ${INSTALL_BINDIR})
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/models/
  DESTINATION ${INSTALL_DATADIR}/models)
install(FILES
  ${PROJECT_SOURCE_DIR}/worlds/robot.world
  ${PROJECT_SOURCE_DIR}/worlds/release_demo.world
  DESTINATION ${INSTALL_DATADIR}/worlds)
