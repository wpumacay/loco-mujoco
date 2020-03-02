
message( "Configuring tysocMjc::examples::agents" )

FILE( GLOB AgentsExampleSources *.cpp )

foreach( exampleFile ${AgentsExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    add_executable( ${execName}
                    ${exampleFile} )

    target_link_libraries( ${execName}
                           tysocCore )

endforeach( exampleFile )
