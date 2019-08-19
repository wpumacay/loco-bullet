
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

add_subdirectory( agents )
add_subdirectory( utils )