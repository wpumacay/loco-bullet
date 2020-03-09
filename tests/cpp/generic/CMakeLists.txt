message( "LOCO::BULLET::tests::cpp >>> Configuring C/C++ loco-bullet-generic tests" )

include_directories( "${LOCO_BULLET_INCLUDE_DIRS}" )

function( FcnBuildBulletTestGeneric pSourcesList pExecutableName )
    add_executable( ${pExecutableName} ${pSourcesList} )
    target_link_libraries( ${pExecutableName} loco_core gtest_main )
    add_test( NAME "${pExecutableName}_test" COMMAND "${pExecutableName}" )
endfunction()

FILE( GLOB TestBulletSources *.cpp )

foreach( testBulletFile ${TestBulletSources} )
    string( REPLACE ".cpp" "" executableLongName ${testBulletFile} )
    get_filename_component( execName ${executableLongName} NAME )
    FcnBuildBulletTestGeneric( ${testBulletFile} ${execName} )
endforeach( testBulletFile )