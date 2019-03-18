
message( "Configuring tysocBullet::examples" )

include_directories( "../include" )

# single builder function

function( FcnBuildTest pSourcesList pExecutableName )

    add_executable( ${pExecutableName}
                    ${pSourcesList} )

    # Link only against tysocCore, as the specific backend is linked dynamically
    target_link_libraries( ${pExecutableName}
                           tysocCore )

endfunction()

FILE( GLOB ExampleSources *.cpp )

foreach( exampleFile ${ExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    FcnBuildTest( ${exampleFile} ${execName} )

endforeach( exampleFile )
