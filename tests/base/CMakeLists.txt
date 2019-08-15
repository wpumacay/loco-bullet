
message( "Configuring tysocBullet::base_test_interface" )

include_directories( "${CMAKE_SOURCE_DIR}/ext/bullet3/src" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui/examples" )

add_library( test_interface SHARED
             test_interface.cpp )
target_link_libraries( test_interface
                       cat1
                       imgui
                       BulletDynamics
                       BulletCollision
                       LinearMath )