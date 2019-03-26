
message( "Configuring tysocMjc::examples::Constraints" )

FILE( GLOB ConstraintsExampleSources *.cpp )

foreach( exampleFile ${ConstraintsExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    FcnBuildTest( ${exampleFile} ${execName} )

endforeach( exampleFile )
