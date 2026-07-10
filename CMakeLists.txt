cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PROJECTNAME vectorview)
set(GUINAME vectorGUI)
project(${PROJECTNAME})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include(FindPkgConfig)
find_package(PkgConfig REQUIRED)

option(BUILD_VECTORGUI "Enables VectorGUI interface installation" TRUE)
option(BUILD_VECTORVIEW "Either VectorView Visual Plugin is built or not" TRUE)

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

set(forcefilter_sources
  src/filters/ForceFilter.cpp
)

set(common_sources
  src/common/ContactUtils.cpp
)

set(plugin_sources
  src/plugin/VectorView.cpp
)

set(gui_sources
  src/gui/VectorGUI.cpp
  src/gui/Interface.cpp
  third_party/qcustomplot/qcustomplot.cpp
)

if(PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

foreach(fileName ${GAZEBO_INCLUDE_DIRS})
  list(APPEND GAZEBO_ROOT_DIRS ${fileName}/gazebo)
endforeach(fileName)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_ROOT_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  ${DSPFILTERS_INCLUDE}
  ${QCUSTOMPLOT_INCLUDE})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

if(BUILD_VECTORGUI)
  find_package(Qt4 REQUIRED QtGui QtXml QtCore)
  include(${QT_USE_FILE})
  add_definitions(${QT_DEFINITIONS})
  include_directories(${QT_INCLUDE_DIRS})
  link_directories(${QT_LIBRARY_DIRS})
endif()

if(BUILD_VECTORVIEW OR BUILD_VECTORGUI)
  add_library(vectorview_common STATIC ${common_sources})
  add_library(dspfilters SHARED ${dspfilters_sources})
  add_library(vectorview_filters SHARED ${forcefilter_sources})
  target_link_libraries(vectorview_filters dspfilters ${GAZEBO_LIBRARIES})
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
