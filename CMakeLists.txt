cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(vector-view)

set(CMAKE_INCLUDE_CURRENT_DIR ON)	# Find includes in corresponding build directories
set(CMAKE_AUTOMOC ON)							# Instruct CMake to run moc automatically when needed.

include (FindPkgConfig)
find_package(PkgConfig REQUIRED)

if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
endif()

# FIND QT4 PACKAGES
find_package(Qt4 REQUIRED QtGui QtXml)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

## AND SO INCLUDE THE PACKAGE DIRECTORIES
include_directories(${PROJECT_SOURCE_DIR}/plugins/include
										${GAZEBO_INCLUDE_DIRS}
										${QT_INCLUDE_DIRS}
										)
link_directories(		${GAZEBO_LIBRARY_DIRS}
										${QT_LIBRARY_DIRS}
										${PROJECT_SOURCE_DIR}/build
										)

## GENERATES THE SHARED LIBRARIES
add_library(vectorview SHARED plugins/src/VectorView.cpp)
target_link_libraries(vectorview
  										${GAZEBO_LIBRARIES}
											)

add_library(interface SHARED plugins/src/Interface.cpp)
target_link_libraries(interface
											${GAZEBO_LIBRARIES}
											${QT_LIBRARY_DIRS}
										 )

add_library(systemgui SHARED plugins/src/SystemGUI.cpp)
target_link_libraries(systemgui
											${GAZEBO_LIBRARIES}
											${QT_LIBRARY_DIRS}
											interface
										 )
