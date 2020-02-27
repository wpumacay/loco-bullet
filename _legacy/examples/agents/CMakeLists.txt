
message( "Configuring tysocBullet::examples::constraints" )

FILE( GLOB AgentsExampleSources *.cpp )

foreach( exampleFile ${AgentsExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    FcnBuildTest( ${exampleFile} ${execName} )

endforeach( exampleFile )
