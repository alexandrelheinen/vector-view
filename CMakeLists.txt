cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

## INITIAL DEFINITIONS
set(PROJECTNAME vectorview)
set(GUINAME     vectorGUI)
set(FILTERNAME  DspFilters)
project(${PROJECTNAME} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD          14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)  # find includes in build directories
set(CMAKE_AUTOMOC             ON)  # run moc automatically

include(FindPkgConfig)
find_package(PkgConfig REQUIRED)

## DIRECTORY LAYOUT
set(SRC_DIR     ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

set(FILTER_SOURCE      ${SRC_DIR}/${FILTERNAME})
set(FILTER_INCLUDE     ${INCLUDE_DIR}/${FILTERNAME})
set(VECTORVIEW_SOURCE  ${SRC_DIR}/${PROJECTNAME})
set(VECTORVIEW_INCLUDE ${INCLUDE_DIR}/${PROJECTNAME})
set(VECTORGUI_SOURCE   ${SRC_DIR}/${GUINAME})
set(VECTORGUI_INCLUDE  ${INCLUDE_DIR}/${GUINAME})

## EXPLICIT SOURCE LISTS
set(filter_source
  ${FILTER_SOURCE}/Bessel.cpp
  ${FILTER_SOURCE}/Biquad.cpp
  ${FILTER_SOURCE}/Butterworth.cpp
  ${FILTER_SOURCE}/Cascade.cpp
  ${FILTER_SOURCE}/ChebyshevI.cpp
  ${FILTER_SOURCE}/ChebyshevII.cpp
  ${FILTER_SOURCE}/Custom.cpp
  ${FILTER_SOURCE}/Design.cpp
  ${FILTER_SOURCE}/Documentation.cpp
  ${FILTER_SOURCE}/Elliptic.cpp
  ${FILTER_SOURCE}/Filter.cpp
  ${FILTER_SOURCE}/ForceFilter.cpp
  ${FILTER_SOURCE}/Legendre.cpp
  ${FILTER_SOURCE}/Param.cpp
  ${FILTER_SOURCE}/PoleFilter.cpp
  ${FILTER_SOURCE}/RBJ.cpp
  ${FILTER_SOURCE}/RootFinder.cpp
  ${FILTER_SOURCE}/State.cpp
)

set(vectorgui_source
  ${VECTORGUI_SOURCE}/Interface.cpp
  ${VECTORGUI_SOURCE}/VectorGUI.cpp
  ${VECTORGUI_SOURCE}/qcustomplot.cpp
)

## BUILD OPTIONS
option(BUILD_VECTORGUI  "Build the VectorGUI desktop application"      ON)
option(BUILD_VECTORVIEW "Build the VectorView Gazebo visual plugin"    ON)

## PACKAGE DISCOVERY
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

foreach(dir ${GAZEBO_INCLUDE_DIRS})
  list(APPEND GAZEBO_ROOT_DIRS ${dir}/gazebo)
endforeach()
execute_process(COMMAND ${CMAKE_COMMAND} -E echo "-- Gazebo directories: ${GAZEBO_ROOT_DIRS}")

## QT5 (required for VectorGUI)
if(BUILD_VECTORGUI)
  execute_process(COMMAND ${CMAKE_COMMAND} -E echo "-- VectorGUI will be built")
  find_package(Qt5 COMPONENTS Widgets Xml Core REQUIRED)
endif()

## DSPFILTERS SHARED LIBRARY
if(BUILD_VECTORVIEW OR BUILD_VECTORGUI)
  add_library(${FILTERNAME} SHARED ${filter_source})
  target_include_directories(${FILTERNAME}
    PUBLIC  ${INCLUDE_DIR}
    PRIVATE ${GAZEBO_INCLUDE_DIRS} ${GAZEBO_ROOT_DIRS})
  target_link_libraries(${FILTERNAME}
    PRIVATE ${GAZEBO_LIBRARIES})
  target_compile_options(${FILTERNAME}
    PRIVATE ${GAZEBO_CXX_FLAGS})
else()
  execute_process(COMMAND ${CMAKE_COMMAND} -E echo "-- Neither VectorView nor VectorGUI will be built")
endif()

## VECTORVIEW VISUAL PLUGIN
if(BUILD_VECTORVIEW)
  add_library(${PROJECTNAME} SHARED
    ${VECTORVIEW_SOURCE}/VectorView.cpp
    ${VECTORVIEW_SOURCE}/NameUtils.cpp
  )
  target_include_directories(${PROJECTNAME}
    PRIVATE ${INCLUDE_DIR}
            ${GAZEBO_INCLUDE_DIRS}
            ${GAZEBO_ROOT_DIRS}
            ${OGRE_INCLUDE_DIRS}
            ${OGRE-Terrain_INCLUDE_DIRS}
            ${PROTOBUF_INCLUDE_DIR}
            ${Boost_INCLUDE_DIRS})
  target_link_libraries(${PROJECTNAME}
    PRIVATE ${GAZEBO_LIBRARIES}
            ${OGRE_LIBRARIES}
            ${FILTERNAME}
            ${Boost_LIBRARIES}
            ${PROTOBUF_LIBRARY})
  target_compile_options(${PROJECTNAME}
    PRIVATE ${GAZEBO_CXX_FLAGS})
endif()

## VECTORGUI EXECUTABLE
if(BUILD_VECTORGUI)
  add_executable(${GUINAME} ${vectorgui_source})
  target_include_directories(${GUINAME}
    PRIVATE ${INCLUDE_DIR}
            ${GAZEBO_INCLUDE_DIRS}
            ${GAZEBO_ROOT_DIRS}
            ${OGRE_INCLUDE_DIRS}
            ${PROTOBUF_INCLUDE_DIR}
            ${Boost_INCLUDE_DIRS})
  target_link_libraries(${GUINAME}
    PRIVATE ${FILTERNAME}
            ${GAZEBO_LIBRARIES}
            ${OGRE_LIBRARIES}
            Qt5::Widgets
            Qt5::Xml
            Qt5::Core
            ${Boost_LIBRARIES}
            ${PROTOBUF_LIBRARY})
  target_compile_options(${GUINAME}
    PRIVATE ${GAZEBO_CXX_FLAGS})
endif()

## TESTS
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
