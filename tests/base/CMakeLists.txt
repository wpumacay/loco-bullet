
message( "Configuring tysocBullet::base_test_interface" )

include_directories( "../../ext/bullet/src" )
include_directories( "../../tysoc/ext/cat1/include" )

add_library( test_interface SHARED
             test_interface.cpp )
target_link_libraries( test_interface
                       cat1
                       BulletDynamics
                       BulletCollision
                       LinearMath )