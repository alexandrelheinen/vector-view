cmake_minimum_required(VERSION 3.10)
project(vectorview LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECTNAME vectorview)
set(GUINAME vectorGUI)

option(BUILD_VECTORGUI "Build the Qt VectorGUI application" ON)
option(BUILD_VECTORVIEW "Build the Gazebo VectorView visual plugin" ON)
option(BUILD_TESTS "Build unit tests" ON)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(DSPFILTERS_INCLUDE ${PROJECT_SOURCE_DIR}/third_party/dspfilters/include)
set(QCUSTOMPLOT_INCLUDE ${PROJECT_SOURCE_DIR}/third_party/qcustomplot)

set(dspfilters_sources
  third_party/dspfilters/src/Bessel.cpp
  third_party/dspfilters/src/Biquad.cpp
  third_party/dspfilters/src/Butterworth.cpp
  third_party/dspfilters/src/Cascade.cpp
  third_party/dspfilters/src/ChebyshevI.cpp
  third_party/dspfilters/src/ChebyshevII.cpp
  third_party/dspfilters/src/Custom.cpp
  third_party/dspfilters/src/Design.cpp
  third_party/dspfilters/src/Documentation.cpp
  third_party/dspfilters/src/Elliptic.cpp
  third_party/dspfilters/src/Filter.cpp
  third_party/dspfilters/src/Legendre.cpp
  third_party/dspfilters/src/Param.cpp
  third_party/dspfilters/src/PoleFilter.cpp
  third_party/dspfilters/src/RBJ.cpp
  third_party/dspfilters/src/RootFinder.cpp
  third_party/dspfilters/src/State.cpp
)

set(forcefilter_sources src/filters/ForceFilter.cpp)
set(common_sources src/common/ContactUtils.cpp)
set(plugin_sources src/plugin/VectorView.cpp)
set(gui_sources
  src/gui/VectorGUI.cpp
  src/gui/Interface.cpp
  third_party/qcustomplot/qcustomplot.cpp
)

# Optional dependencies for plugin/GUI targets only.
if(BUILD_VECTORGUI)
  set(QT_USE_QTGUI TRUE)
  set(QT_USE_QTXML TRUE)
  set(QT_USE_QTCORE TRUE)
  find_package(Qt4 QUIET)
  if(NOT QT4_FOUND)
    message(WARNING
      "Qt4 not found — disabling BUILD_VECTORGUI. "
      "Install Qt4 or pass -DBUILD_VECTORGUI=OFF for a tests-only build.")
    set(BUILD_VECTORGUI OFF CACHE BOOL "Build the Qt VectorGUI application" FORCE)
  endif()
endif()

if(BUILD_VECTORVIEW OR BUILD_VECTORGUI)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GAZEBO QUIET gazebo)
  pkg_check_modules(OGRE QUIET OGRE)
  pkg_check_modules(OGRE-Terrain QUIET OGRE-Terrain)

  if(NOT GAZEBO_FOUND)
    message(WARNING
      "Gazebo not found — disabling BUILD_VECTORVIEW and BUILD_VECTORGUI. "
      "Install Gazebo or pass -DBUILD_VECTORVIEW=OFF -DBUILD_VECTORGUI=OFF "
      "for a tests-only build.")
    set(BUILD_VECTORVIEW OFF CACHE BOOL "Build the Gazebo VectorView visual plugin" FORCE)
    set(BUILD_VECTORGUI OFF CACHE BOOL "Build the Qt VectorGUI application" FORCE)
  endif()
endif()

if(BUILD_VECTORVIEW OR BUILD_VECTORGUI)
  include(FindProtobuf)
  find_package(Protobuf REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system filesystem)

  foreach(fileName ${GAZEBO_INCLUDE_DIRS})
    list(APPEND GAZEBO_ROOT_DIRS ${fileName}/gazebo)
  endforeach()

  include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${GAZEBO_ROOT_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE-Terrain_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS})

  link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${OGRE_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS})

  list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
endif()

if(BUILD_VECTORGUI)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  set(CMAKE_AUTOMOC ON)
  include(${QT_USE_FILE})
  add_definitions(${QT_DEFINITIONS})
  include_directories(${QT_INCLUDE_DIRS})
  link_directories(${QT_LIBRARY_DIRS})
endif()

if(BUILD_VECTORVIEW OR BUILD_VECTORGUI OR BUILD_TESTS)
  include_directories(
    ${INCLUDE_DIR}
    ${DSPFILTERS_INCLUDE}
    ${QCUSTOMPLOT_INCLUDE})

  add_library(vectorview_common STATIC ${common_sources})
  add_library(dspfilters SHARED ${dspfilters_sources})
  add_library(vectorview_filters SHARED ${forcefilter_sources})
  target_link_libraries(vectorview_filters dspfilters)
endif()

if(BUILD_VECTORVIEW)
  add_library(${PROJECTNAME} SHARED ${plugin_sources})
  target_link_libraries(${PROJECTNAME}
    vectorview_common
    vectorview_filters
    dspfilters
    ${GAZEBO_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARY})
endif()

if(BUILD_VECTORGUI)
  add_executable(${GUINAME} ${gui_sources})
  target_link_libraries(${GUINAME}
    vectorview_common
    vectorview_filters
    dspfilters
    ${GAZEBO_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARY})
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
install(FILES ${PROJECT_SOURCE_DIR}/worlds/robot.world
  DESTINATION ${INSTALL_DATADIR}/worlds)
