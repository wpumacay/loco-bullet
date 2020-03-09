message( "LOCO::BULLET::tests::cpp >>> Configuring C/C++ loco-bullet-specific tests" )

include_directories( "${LOCO_BULLET_INCLUDE_DIRS}" )

function( FcnBuildBulletTestSpecific pSourcesList pExecutableName )
    add_executable( ${pExecutableName} ${pSourcesList} )
    target_link_libraries( ${pExecutableName} locoPhysicsBULLET loco_core gtest_main )
    add_test( NAME "${pExecutableName}_test" COMMAND "${pExecutableName}" )
endfunction()

FILE( GLOB TestBulletSources *.cpp )

foreach( testBulletFile ${TestBulletSources} )
    string( REPLACE ".cpp" "" executableLongName ${testBulletFile} )
    get_filename_component( execName ${executableLongName} NAME )
    FcnBuildBulletTestSpecific( ${testBulletFile} ${execName} )
endforeach( testBulletFile )