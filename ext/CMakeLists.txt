
# Configure dependencies
message( "Configuring external dependencies" )

# build spdlog as a shared library
set( SPDLOG_BUILD_SHARED ON CACHE BOOL "Build shared library" )

add_subdirectory( imgui )
add_subdirectory( spdlog )
add_subdirectory( cat1 )
add_subdirectory( pybind11 )
