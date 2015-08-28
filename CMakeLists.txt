cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

## INITIAL DEFINITIONS
SET(PROJECTNAME vectorview)
SET(GUINAME vectorGUI)
set(FILTERNAME DspFilters)
project(${PROJECTNAME})

set(CMAKE_INCLUDE_CURRENT_DIR ON)	# Find includes in corresponding build directories
set(CMAKE_AUTOMOC ON)							# Instruct CMake to run moc automatically when needed.

include (FindPkgConfig)
find_package(PkgConfig REQUIRED)

## DIRECTORIES SETTING AND OPTIONS
SET(SRC_DIR                 ${PROJECT_SOURCE_DIR}/src)
SET(INCLUDE_DIR             ${PROJECT_SOURCE_DIR}/include)

SET(FILTER_SOURCE           ${SRC_DIR}/${FILTERNAME})
SET(FILTER_INCLUDE          ${INCLUDE_DIR}/${FILTERNAME})
SET(VECTORVIEW_SOURCE       ${SRC_DIR}/${PROJECTNAME})
SET(VECTORVIEW_INCLUDE      ${INCLUDE_DIR}/${PROJECTNAME})
SET(VECTORGUI_SOURCE        ${SRC_DIR}/${GUINAME})
SET(VECTORGUI_INCLUDE       ${INCLUDE_DIR}/${GUINAME})

FILE(GLOB filter_source     ${FILTER_SOURCE}/*.cpp)
FILE(GLOB filter_header     ${FILTER_INCLUDE}/*.h)
FILE(GLOB vectorgui_source  ${VECTORGUI_SOURCE}/*.cpp)
FILE(GLOB vectorgui_header  ${VECTORGUI_INCLUDE}/*.h)

SOURCE_GROUP("Source Files" FILES ${filter_source})
SOURCE_GROUP("Header Files" FILES ${filter_header})
SOURCE_GROUP("Source Files" FILES ${vectorgui_source})
SOURCE_GROUP("Header Files" FILES ${vectorgui_header})

option(BUILD_VECTORGUI  "Enables VectorGUI interface installation" TRUE)
option(BUILD_VECTORVIEW "Either VectorView Visual Plugin is built or not" TRUE)

## PACKAGES INPUT
if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
	pkg_check_modules(OGRE OGRE)
	pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

FOREACH(fileName ${GAZEBO_INCLUDE_DIRS})
	list(APPEND GAZEBO_ROOT_DIRS ${fileName}/gazebo)
ENDFOREACH(fileName)
execute_process(COMMAND echo "-- Gazebo directories:" ${GAZEBO_ROOT_DIRS})

## DEFAULT GAZEBO, OGRE, PROTOBUF AND LOCAL DIRS INCLUDES
include_directories(${GAZEBO_INCLUDE_DIRS}
		                ${GAZEBO_ROOT_DIRS}
                    ${OGRE_INCLUDE_DIRS}
		                ${OGRE-Terrain_INCLUDE_DIRS}
                    ${PROTOBUF_INCLUDE_DIR}
		                ${Boost_INCLUDE_DIRS}
                    ${INCLUDE_DIR})
link_directories   (${GAZEBO_LIBRARY_DIRS}
		                ${OGRE_LIBRARY_DIRS}
	                  ${Boost_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## IF VECTOR IS BUILT, QT IS NEEDED AS WELL
if(BUILD_VECTORGUI)
	execute_process(COMMAND echo "-- VectorGUI will be built")
	# FIND QT4 PACKAGES
	find_package       (Qt4 REQUIRED
			                             QtGui
				                        	 QtXml
				                         	 QtCore)
	include            (${QT_USE_FILE})
	add_definitions    (${QT_DEFINITIONS})
	include_directories(${QT_INCLUDE_DIRS})
	link_directories   (${QT_LIBRARY_DIRS})
endif()

## GENERATES SHARED LIBRARIES AND EXECUTABLES
if(BUILD_VECTORVIEW OR BUILD_VECTORGUI)
	add_library(${FILTERNAME}   SHARED ${filter_source} ${filter_header})
	target_link_libraries(      ${FILTERNAME}
	                            ${GAZEBO_LIBRARIES})
else()
	execute_process(COMMAND echo "-- Neither VectorView nor VectorGUI will be built")
endif()

if(BUILD_VECTORVIEW)
	add_library(${PROJECTNAME}  SHARED ${VECTORVIEW_SOURCE}/VectorView.cpp)
	target_link_libraries(      ${PROJECTNAME}
	                            ${GAZEBO_LIBRARIES}
				                      ${OGRE_LIBRARIES}
				                      ${FILTERNAME}
				                      ${Boost_LIBRARIES}
				                      ${PROTOBUF_LIBRARY})
endif()

if(BUILD_VECTORGUI)
  add_executable(${GUINAME} ${vectorgui_source} ${vectorgui_header})
  target_link_libraries(    ${GUINAME}
			                      ${FILTERNAME}
		                        ${GAZEBO_LIBRARIES}
			                      ${OGRE_LIBRARIES}
		                        ${QT_LIBRARIES}
			                      ${Boost_LIBRARIES}
			                      ${PROTOBUF_LIBRARY})
endif()
