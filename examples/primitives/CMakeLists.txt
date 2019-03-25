
message( "Configuring tysocBullet::examples::primitives" )

FILE( GLOB PrimitivesExampleSources *.cpp )

foreach( exampleFile ${PrimitivesExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    FcnBuildTest( ${exampleFile} ${execName} )

endforeach( exampleFile )
