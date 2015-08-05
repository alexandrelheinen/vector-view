cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iCubStage)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

## JUST ASSURE THAT THE GAZEBO PACKAGE IS WELL INPUT
include (FindPkgConfig)
find_package(PkgConfig REQUIRED)

if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
endif()

find_package(GAZEBO)

# FIND QT4 PACKAGES
find_package(Qt4 REQUIRED QtGui QtXml)

## AND SO INCLUDE THE PACKAGE DIRECTORIES
include_directories(${GAZEBO_INCLUDE_DIRS}
										${QT_INCLUDE_DIRS}
										)
link_directories(		${GAZEBO_LIBRARY_DIRS}
										${QT_LIBRARY_DIRS}
										)

## GENERATES THE SHARED LIBRARIES
add_library(upvector SHARED VectorView.cpp)
target_link_libraries(upvector
  ${GAZEBO_LIBRARIES}
)

## GENERATES THE SHARED LIBRARIES
add_library(linkview SHARED LinkManager.cpp)
target_link_libraries(linkview
  ${GAZEBO_LIBRARIES}
)

# add_library(contact SHARED ContactManager.cpp)
# target_link_libraries(contact
#  ${GAZEBO_LIBRARIES}
# )
