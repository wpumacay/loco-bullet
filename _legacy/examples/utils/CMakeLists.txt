
message( "Configuring tysocBullet::examples::utils" )

FILE( GLOB UtilsExampleSources *.cpp )

include_directories( "${CMAKE_SOURCE_DIR}/include/" )

add_library( utils
             "${CMAKE_SOURCE_DIR}/src/bullet_common.cpp"
             "${CMAKE_SOURCE_DIR}/src/bullet_utils.cpp" )
target_link_libraries( utils
                       tysocCore
                       assimp
                       BulletDynamics 
                       BulletCollision 
                       LinearMath )

function( FcnBuildTestUtils pSourcesList pExecutableName )

    add_executable( ${pExecutableName}
                    ${pSourcesList} )

    # Link only against tysocCore, as the specific backend is linked dynamically
    target_link_libraries( ${pExecutableName}
                           utils )

endfunction()

foreach( exampleFile ${UtilsExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    FcnBuildTestUtils( ${exampleFile} ${execName} )

endforeach( exampleFile )
