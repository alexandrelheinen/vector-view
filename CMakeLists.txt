cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

## INITIAL DEFINITIONS
SET(PROJECTNAME vectorview)
project(${PROJECTNAME})
SET(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
SET(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
SET(BUILD_DIR ${PROJECT_SOURCE_DIR}/build)

set(CMAKE_INCLUDE_CURRENT_DIR ON)	# Find includes in corresponding build directories
set(CMAKE_AUTOMOC ON)							# Instruct CMake to run moc automatically when needed.

include (FindPkgConfig)
find_package(PkgConfig REQUIRED)

## EXTERNAL PROJECTS AND OPTIONS
include(ExternalProject)
set(FILTERNAME DSPFilters)

if(NOT EXISTS ${FILTERNAME})
	ExternalProject_Add(${FILTERNAME}																												# extern IIR filtering library
											GIT_REPOSITORY https://github.com/vinniefalco/DSPFilters.git        # URL of git repo
	  									GIT_TAG master																											# git branch
											SOURCE_DIR ${PROJECT_SOURCE_DIR}/${FILTERNAME}
											CONFIGURE_COMMAND echo "[    ] Don't need to configure ${FILTERNAME}"
											BUILD_COMMAND echo "[    ] ${FILTERNAME} will be built with the ${PROJECTNAME} main project"
											INSTALL_COMMAND echo "[    ] It is not necessary to install it, headers are included from this directory"
											)
endif()

option(BUILD_VECTORGUI "Enables VectorGUI interface installation" FALSE)									# either Interface GUI is built or not

SET(FILTER_SOURCE ${PROJECT_SOURCE_DIR}/${FILTERNAME}/shared/${FILTERNAME}/source)
SET(FILTER_INCLUDE ${PROJECT_SOURCE_DIR}/${FILTERNAME}/shared/${FILTERNAME}/include)

FILE(GLOB filter_source ${FILTER_SOURCE}/*.cpp)
FILE(GLOB filter_header ${FILTER_INCLUDE}/${FILTERNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${filter_source})
SOURCE_GROUP("Header Files" FILES ${filter_header})

## PACKAGES INPUT
if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
endif()

## IF NO INTERFACE GUI, NO QT NEEDED, ONLY GAZEBO DIRS INCLUDED
include_directories(${GAZEBO_INCLUDE_DIRS}
									  ${INCLUDE_DIR}
										${FILTER_INCLUDE}
									 )
link_directories(${GAZEBO_LIBRARY_DIRS}
								 ${BUILD_DIR}
								)

if(BUILD_VECTORGUI)
	# FIND QT4 PACKAGES
	find_package(Qt4 REQUIRED QtGui QtXml)
	include(${QT_USE_FILE})
	add_definitions(${QT_DEFINITIONS})

	## AND SO INCLUDE THE PACKAGE DIRECTORIES
	include_directories(${QT_INCLUDE_DIRS}
											)
	link_directories(		${QT_LIBRARY_DIRS}
											)
endif()

## GENERATES SHARED LIBRARIES AND EXECUTABLES

add_library(filters SHARED ${filter_source} ${filter_header})

add_library(${PROJECTNAME} SHARED ${SRC_DIR}/VectorView.cpp)
target_link_libraries(${PROJECTNAME}
  										${GAZEBO_LIBRARIES}
											)

if(BUILD_VECTORGUI)
	add_library(interface SHARED ${SRC_DIR}/Interface.cpp)
	target_link_libraries(interface
	  										${GAZEBO_LIBRARIES}
												${Boost_LIBRARIES}
												${PROTUBUF_LIBRARIES}
												${QT_LIBRARIES}
												)

	add_executable(VectorGUI ${SRC_DIR}/VectorGUI.cpp)
	target_link_libraries(VectorGUI
	  										${GAZEBO_LIBRARIES}
												${QT_LIBRARIES}
												interface
												)
endif()
