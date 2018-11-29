
message( "Configuring examples" )


add_executable( test_helloworld
                test_helloworld.cpp )
target_link_libraries( test_helloworld
                       BulletDynamics 
                       BulletCollision 
                       LinearMath  )


add_executable( test_integration_renderer
                test_integration_renderer.cpp )
target_link_libraries( test_integration_renderer
                       cat1
                       BulletDynamics 
                       BulletCollision 
                       LinearMath  )


add_executable( test_tysoc
                test_tysoc.cpp )
target_link_libraries( test_tysoc
                       tysocBullet  )

add_executable( test_urdf
                test_urdf.cpp )
target_link_libraries( test_urdf
                       tysocBullet  )