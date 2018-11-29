cmake_minimum_required( VERSION 3.0.0 )

project( tinyxml2 )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}" )

add_library( tinyxml2
             tinyxml2.cpp )