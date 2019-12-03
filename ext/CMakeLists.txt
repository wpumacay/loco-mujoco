
# Configure dependencies
message( "Configuring external dependencies" )

# build spdlog as a shared library
set( SPDLOG_BUILD_SHARED ON CACHE BOOL "Build shared library" )
# avoid building cat1 examples
set( CAT1_BUILD_EXAMPLES OFF CACHE BOOL "Do not build cat1 examples" )

add_subdirectory( imgui )
add_subdirectory( spdlog )
add_subdirectory( cat1 )
add_subdirectory( pybind11 )
