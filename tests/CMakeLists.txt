
message( "Configuring tysoc-mujoco::tests" )

include_directories( "~/.mujoco/mujoco200_linux/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui/examples" )
include_directories( "${CMAKE_SOURCE_DIR}/core/include" )

add_subdirectory( base )
include_directories( base )

# single builder function

function( FcnBuildTest pSourcesList pExecutableName )

    add_executable( ${pExecutableName}
                    ${pSourcesList} )

    # Link only against tysocCore, as the specific backend is linked dynamically
    target_link_libraries( ${pExecutableName}
                           test_interface )

endfunction()

file( GLOB TestFiles *.cpp )

foreach( testFile ${TestFiles} )
    # remove extension from sample file
    string( REPLACE ".cpp" "" executableLongName ${testFile} )
    # remove other path components from file
    get_filename_component( executableName ${executableLongName} NAME )

    FcnBuildTest( ${testFile} ${executableName} )

endforeach( testFile )