message( "LOCO::MUJOCO::tests::cpp >>> Configuring C/C++ loco-mujoco-generic tests" )

include_directories( "${LOCO_MUJOCO_INCLUDE_DIRS}" )

function( FcnBuildMujocoTestGeneric pSourcesList pExecutableName )
    add_executable( ${pExecutableName} ${pSourcesList} )
    target_link_libraries( ${pExecutableName} loco_core gtest_main )
    add_test( NAME "${pExecutableName}_test" COMMAND "${pExecutableName}" )
endfunction()

FILE( GLOB TestMujocoSources *.cpp )

foreach( testMujocoFile ${TestMujocoSources} )
    string( REPLACE ".cpp" "" executableLongName ${testMujocoFile} )
    get_filename_component( execName ${executableLongName} NAME )
    FcnBuildMujocoTestGeneric( ${testMujocoFile} ${execName} )
endforeach( testMujocoFile )