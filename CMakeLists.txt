
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS " -g ${CMAKE_CXX_FLAGS}" )

add_definitions( -DTYSOCBULLET_RESOURCES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/res/" )

# remove bullet examples from build
set( BUILD_BULLET2_DEMOS OFF CACHE BOOL "Do not build bullet-2 demos" )
set( BUILD_CPU_DEMOS OFF CACHE BOOL "Do not build bullet cpu demos" )
set( BUILD_EGL OFF CACHE BOOL "Do not build bullet-EGL" )
set( BUILD_EXTRAS OFF CACHE BOOL "Do not build bullet-extras" )
set( BUILD_CLSOCKET OFF CACHE BOOL "Do not build bullet-clsocket" )
set( BUILD_OPENGL3_DEMOS OFF CACHE BOOL "Do not build bullet-opengl3-demos" )
set( BUILD_UNIT_TESTS OFF CACHE BOOL "Do not build bullet-unittests" )

# Configure external dependencies
add_subdirectory( ext )

# Configure visualizer // @TODO: Should move this out from src to the ext directory
add_subdirectory( tysocViz )

# Configure tysocBullet project *************************************************

# grab all the sources for compilation
file( GLOB_RECURSE TYSOCBULLET_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

# add all dependencies to include path
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/bullet/src" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/tysocTerrain/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/tysocViz/include" )

message( "TYSOCBULLET_SOURCES: ${TYSOCBULLET_SOURCES}" )

# Configure the resulting library
add_library( tysocBullet
             ${TYSOCBULLET_SOURCES} )
target_link_libraries( tysocBullet
                       tysocViz
                       tysoc
                       BulletDynamics 
                       BulletCollision 
                       LinearMath )

# *******************************************************************************

# Configure examples
add_subdirectory( examples )
