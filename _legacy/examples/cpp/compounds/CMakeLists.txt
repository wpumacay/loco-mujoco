
message( "Configuring tysocMjc::examples::compounds" )

FILE( GLOB CompoundsExampleSources *.cpp )

foreach( exampleFile ${CompoundsExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    add_executable( ${execName}
                    ${exampleFile} )

    target_link_libraries( ${execName}
                           tysocCore )

endforeach( exampleFile )
