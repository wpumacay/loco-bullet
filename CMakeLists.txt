
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS " -g ${CMAKE_CXX_FLAGS}" )

if ( NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY )
    message( "TYSOCBULLET> Sending generated libs to OWN build directory" )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/libs" )
else()
    message( "TYSOCBULLET> Sending generated libs to EXTERNAL build directory" )
endif()

message( "TYSOCBULLET> LIBRARY_OUT_DIR: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" )

set( TYSOCBULLET_IS_MASTER_PROJECT OFF )
if( CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( "TYSOCBULLET> Configuring as MASTER project" )
    set( TYSOCBULLET_IS_MASTER_PROJECT ON )
else()
    message( "TYSOCBULLET> Configuring as CHILD project" )
endif()

# Add core targets only if master project. If not, parent project must configure it
if ( TYSOCBULLET_IS_MASTER_PROJECT )
    # Configure core functionality
    add_subdirectory( tysoc )
endif()

# Configure tysocBullet project ************************************************

# Set the lbirary paths as props to allow usage from other projects
set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_BULLET "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocPhysicsBullet.so" )

# get properties defined in core library
get_property( VAR_TYSOC_PATH_RESOURCES              GLOBAL PROPERTY PROP_TYSOC_PATH_RESOURCES )
get_property( VAR_TYSOC_PATH_WORKING_DIR            GLOBAL PROPERTY PROP_TYSOC_PATH_WORKING_DIR )
get_property( VAR_TYSOC_PATH_MJCF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_MJCF_TEMPLATES )
get_property( VAR_TYSOC_PATH_URDF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_URDF_TEMPLATES )
get_property( VAR_TYSOC_PATH_RLSIM_TEMPLATES        GLOBAL PROPERTY PROP_TYSOC_PATH_RLSIM_TEMPLATES )
get_property( VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ      GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ )
get_property( VAR_TYSOC_BACKEND_PHYSICS_BULLET      GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_BULLET )

# repeat definitions from core library, as they are not propagated up
add_definitions( -DTYSOC_PATH_RESOURCES="${VAR_TYSOC_PATH_RESOURCES}" )
add_definitions( -DTYSOC_PATH_WORKING_DIR="${VAR_TYSOC_PATH_WORKING_DIR}" )
add_definitions( -DTYSOC_PATH_MJCF_TEMPLATES="${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_URDF_TEMPLATES="${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_RLSIM_TEMPLATES="${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
add_definitions( -DTYSOC_BACKEND_GRAPHICS_GLVIZ="${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
add_definitions( -DTYSOC_BACKEND_PHYSICS_BULLET="${VAR_TYSOC_BACKEND_PHYSICS_BULLET}" )

# test all properties
message( "PROP_TYSOC_PATH_RESOURCES:            ${VAR_TYSOC_PATH_RESOURCES}" )
message( "PROP_TYSOC_PATH_WORKING_DIR:          ${VAR_TYSOC_PATH_WORKING_DIR}" )
message( "PROP_TYSOC_PATH_MJCF_TEMPLATES:       ${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_URDF_TEMPLATES:       ${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_RLSIM_TEMPLATES:      ${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
message( "PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ:    ${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
message( "PROP_TYSOC_BACKEND_PHYSICS_BULLET:    ${VAR_TYSOC_BACKEND_PHYSICS_BULLET}" )

# grab all the sources for compilation
#file( GLOB_RECURSE TYSOCBULLET_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )
set( TYSOCBULLET_SOURCES src/bullet_common.cpp
                         src/bullet_utils.cpp
                         src/bullet_simulation.cpp
                         src/bullet_sandbox_body_wrapper.cpp )
message( "TYSOCBULLET_SOURCES: ${TYSOCBULLET_SOURCES}" )

# add this project headers to includepath
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )

# add the core dependency headers to includepath
if ( TYSOCBULLET_IS_MASTER_PROJECT )
    # core dependency to use is own submodule
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/include" )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/ext/json" )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/ext/tinyxml2" )
else()
    # core dependency to use if parent core submodule, names as "core"
    include_directories( "${CMAKE_SOURCE_DIR}/core/include" )
    include_directories( "${CMAKE_SOURCE_DIR}/core/ext/json" )
    include_directories( "${CMAKE_SOURCE_DIR}/core/ext/tinyxml2" )
endif()

# remove bullet examples from build
set( BUILD_BULLET2_DEMOS OFF CACHE BOOL "Do not build bullet-2 demos" )
set( BUILD_CPU_DEMOS OFF CACHE BOOL "Do not build bullet cpu demos" )
set( BUILD_EGL OFF CACHE BOOL "Do not build bullet-EGL" )
set( BUILD_EXTRAS OFF CACHE BOOL "Do not build bullet-extras" )
set( BUILD_CLSOCKET OFF CACHE BOOL "Do not build bullet-clsocket" )
set( BUILD_OPENGL3_DEMOS OFF CACHE BOOL "Do not build bullet-opengl3-demos" )
set( BUILD_UNIT_TESTS OFF CACHE BOOL "Do not build bullet-unittests" )
set( BUILD_SHARED_LIBS ON CACHE BOOL "Force shared libs" )

# add bullet project (@TODO: check if can use apt version, as it crashed last time)
add_subdirectory( ext/bullet )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/bullet/src" )

# Configure the resulting library
add_library( tysocPhysicsBullet SHARED
             ${TYSOCBULLET_SOURCES} )
target_link_libraries( tysocPhysicsBullet
                       tysocCore
                       BulletDynamics 
                       BulletCollision 
                       LinearMath )

# ******************************************************************************

# Only generate samples if not using as dependency
if ( TYSOCBULLET_IS_MASTER_PROJECT )
    # Configure examples
    add_subdirectory( examples )
endif()