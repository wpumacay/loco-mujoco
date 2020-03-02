
message( "Configuring tysocMjc::examples::terrain" )

FILE( GLOB TerrainExampleSources *.cpp )

foreach( exampleFile ${TerrainExampleSources} )
    
    string( REPLACE ".cpp" "" executableLongName ${exampleFile} )
    get_filename_component( execName ${executableLongName} NAME )

    add_executable( ${execName}
                    ${exampleFile} )

    target_link_libraries( ${execName}
                           tysocCore )

endforeach( exampleFile )
