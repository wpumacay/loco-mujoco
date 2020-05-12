message( "LOCO::MUJOCO >>> Configuring external dependencies" )

# Configure build-options for spdlog
set( SPDLOG_BUILD_SHARED ON CACHE BOOL "Build shared library" )
set( SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "Don't build spdlog examples" )
set( SPDLOG_BUILD_EXAMPLE_HO OFF CACHE BOOL "Don't build spdlog examples (ho-flag)" )
set( SPDLOG_BUILD_TESTS OFF CACHE BOOL "Don't build spdlog tests" )
set( SPDLOG_BUILD_TESTS_HO OFF CACHE BOOL "Don't build spdlog tests (ho-flag)" )
set( SPDLOG_BUILD_BENCH OFF CACHE BOOL "Don't build spdlog benchmarks" )

# Configure build-options for tiny_math
set( TINYMATH_BUILD_DOCS OFF CACHE BOOL "Don't build tinymath's documentation" )
set( TINYMATH_BUILD_EXAMPLES OFF CACHE BOOL "Don't build tinymath's examples" )
set( TINYMATH_BUILD_PYTHON_BINDINGS ON CACHE BOOL "Build tinymath's python-bindings" )

# Configure build-options for tiny_utils
set( TINYUTILS_BUILD_EXAMPLES OFF CACHE BOOL "Don't build tinyutils' examples" )
set( TINYUTILS_BUILD_PYTHON_BINDINGS ON CACHE BOOL "Build tinyutils' python-bindings" )

add_subdirectory( imgui )
add_subdirectory( spdlog )
add_subdirectory( pybind11 )
add_subdirectory( tiny_math )
add_subdirectory( tiny_utils )

if ( LOCO_CORE_BUILD_WINDOWED_VISUALIZER OR LOCO_CORE_BUILD_HEADLESS_VISUALIZER )
    if ( LOCO_CORE_BUILD_WINDOWED_VISUALIZER )
        set( TINYRENDERER_BUILD_WINDOWED_GLFW ON CACHE BOOL "Build tiny-renderer windowed-backend (glfw)" )
    else()
        set( TINYRENDERER_BUILD_WINDOWED_GLFW OFF CACHE BOOL "Don't build tiny-renderer windowed-bckend (glfw)" )
    endif()

    if ( LOCO_CORE_BUILD_HEADLESS_VISUALIZER )
        set( TINYRENDERER_BUILD_HEADLESS_EGL ON CACHE BOOL "Build tiny-renderer headless-backend (egl)" )
    else()
        set( TINYRENDERER_BUILD_HEADLESS_EGL OFF CACHE BOOL "Don't build tiny-renderer headless-backend (egl)" )
    endif()

    set( TINYRENDERER_BUILD_DOCS OFF CACHE BOOL "Don't build tinyrenderer's documentation" )
    set( TINYRENDERER_BUILD_EXAMPLES OFF CACHE BOOL "Don't build tinyrenderer's examples" )
    set( TINYRENDERER_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Don't build tinyrenderer's python bindings" )

    if ( LOCO_CORE_BUILD_WITH_LOGS OR ( CMAKE_BUILD_TYPE STREQUAL Debug ) )
        set( TINYRENDERER_BUILD_WITH_LOGS ON CACHE BOOL "Build tinyrenderer with logs enabled" )
    else()
        set( TINYRENDERER_BUILD_WITH_LOGS OFF CACHE BOOL "Don't build tinyrenderer with logs enabled" )
    endif()

    if ( LOCO_CORE_BUILD_WITH_TRACK_ALLOCS OR ( CMAKE_BUILD_TYPE STREQUAL Debug ) )
        set( TINYRENDERER_BUILD_WITH_TRACK_ALLOCS ON CACHE BOOL "Build tinyrenderer with logs enabled" )
    else()
        set( TINYRENDERER_BUILD_WITH_TRACK_ALLOCS OFF CACHE BOOL "Don't build tinyrenderer with logs enabled" )
    endif()

    add_subdirectory( tiny_renderer )
endif()

if ( LOCO_CORE_BUILD_TESTS )
    add_subdirectory( googletest )
endif()