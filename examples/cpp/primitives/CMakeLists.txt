
message( "Configuring tysocMjc::examples::primitives" )

FILE( GLOB PrimitivesExampleSources *.cpp )

foreach( exampleFile ${PrimitivesExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    add_executable( ${execName}
                    ${exampleFile} )

    target_link_libraries( ${execName}
                           tysocCore )

endforeach( exampleFile )
