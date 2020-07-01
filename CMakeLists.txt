cmake_minimum_required( VERSION 3.0.0 )

#### aka TysocMjc project
project( LocoMujoco )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}" )

# In case nobody set the project type, set it (parent project might have set it for all)
if ( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Debug )
endif()

if ( NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY )
    message( "LOCO::MUJOCO >>> Sending generated libs to OWN build directory" )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/libs" )
else()
    message( "LOCO::MUJOCO >>> Sending generated libs to EXTERNAL build directory" )
endif()
message( "LOCO::MUJOCO >>> LIBRARY_OUT_DIR: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" )

set( LOCO_MUJOCO_IS_MASTER_PROJECT OFF )
if( CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( "LOCO::MUJOCO >>> Configuring as MASTER project" )
    set( LOCO_MUJOCO_IS_MASTER_PROJECT ON )
else()
    message( "LOCO::MUJOCO >>> Configuring as CHILD project" )
endif()

# Add external dependencies only if master project. If not, parent project must configure it
if ( LOCO_MUJOCO_IS_MASTER_PROJECT )
    # Setup and add core library (Loco::Core)
    set( LOCO_CORE_BUILD_WINDOWED_VISUALIZER ON CACHE BOOL "Build Loco::Core OpenGL-based windowed(GLFW) visualizer" )
    set( LOCO_CORE_BUILD_HEADLESS_VISUALIZER ON CACHE BOOL "Build Loco::Core OpenGL-based headless(EGL) visualizer" )
    set( LOCO_CORE_BUILD_DOCS OFF CACHE BOOL "Don't build Loco::Core documentation" )
    set( LOCO_CORE_BUILD_EXAMPLES ON CACHE BOOL "Build Loco::Core C/C++ examples" )
    set( LOCO_CORE_BUILD_TESTS ON CACHE BOOL "Don't build Loco::Core C/C++ tests" )
    set( LOCO_CORE_BUILD_PYTHON_BINDINGS ON CACHE BOOL "Build Loco::Core Python-bindings" )
    set( LOCO_CORE_BUILD_WITH_LOGS ON CACHE BOOL "Build Loco::Core using logging functionality" )
    set( LOCO_CORE_BUILD_WITH_TRACK_ALLOCS OFF CACHE BOOL "Build Loco::Core using tracking of objects allocations|deallocations" )

    # Resources path: if not given by other project|setup-script, then use the default (this project's core/res folder location)
    if ( NOT LOCO_CORE_RESOURCES_PATH )
        set( LOCO_CORE_RESOURCES_PATH "${CMAKE_SOURCE_DIR}/core/res/" )
    endif()
    # Libraries path: if not given by other project|setup-script, then use the default (this project's build-libs folder location)
    if ( NOT LOCO_CORE_LIBRARIES_PATH )
        set( LOCO_CORE_LIBRARIES_PATH "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/" )
    endif()

    add_subdirectory( ext )
    add_subdirectory( core )

    # Repeat some required definitions (for mujoco-backend target and tests targets) ###############
    if ( LOCO_CORE_BUILD_WITH_LOGS )
        add_definitions( -DLOCO_CORE_USE_LOGS )
    endif()
    if ( LOCO_CORE_BUILD_WITH_TRACK_ALLOCS )
        add_definitions( -DLOCO_CORE_USE_TRACK_ALLOCS )
    endif()

    add_definitions( -DLOCO_PATH_RESOURCES="${LOCO_CORE_RESOURCES_PATH}" )
    if ( WIN32 )
        add_definitions( -DLOCO_BACKEND_GRAPHICS_GLVIZ_GLFW="${LOCO_CORE_LIBRARIES_PATH}liblocoRenderingGlvizGLFW.dll" )
        add_definitions( -DLOCO_BACKEND_GRAPHICS_GLVIZ_EGL="${LOCO_CORE_LIBRARIES_PATH}liblocoRenderingGlvizEGL.dll" )
        add_definitions( -DLOCO_BACKEND_PHYSICS_MUJOCO="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsMUJOCO.dll" )
        add_definitions( -DLOCO_BACKEND_PHYSICS_BULLET="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsBULLET.dll" )
        add_definitions( -DLOCO_BACKEND_PHYSICS_RAISIM="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsRAISIM.dll" )
        add_definitions( -DLOCO_BACKEND_PHYSICS_DART="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsDART.dll" )
    elseif ( UNIX )
        if ( APPLE )
            add_definitions( -DLOCO_BACKEND_GRAPHICS_GLVIZ_GLFW="${LOCO_CORE_LIBRARIES_PATH}liblocoRenderingGlvizGLFW.dylib" )
            add_definitions( -DLOCO_BACKEND_GRAPHICS_GLVIZ_EGL="${LOCO_CORE_LIBRARIES_PATH}liblocoRenderingGlvizEGL.dylib" )
            add_definitions( -DLOCO_BACKEND_PHYSICS_MUJOCO="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsMUJOCO.dylib" )
            add_definitions( -DLOCO_BACKEND_PHYSICS_BULLET="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsBULLET.dylib" )
            add_definitions( -DLOCO_BACKEND_PHYSICS_RAISIM="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsRAISIM.dylib" )
            add_definitions( -DLOCO_BACKEND_PHYSICS_DART="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsDART.dylib" )
        else ()
            add_definitions( -DLOCO_BACKEND_GRAPHICS_GLVIZ_GLFW="${LOCO_CORE_LIBRARIES_PATH}liblocoRenderingGlvizGLFW.so" )
            add_definitions( -DLOCO_BACKEND_GRAPHICS_GLVIZ_EGL="${LOCO_CORE_LIBRARIES_PATH}liblocoRenderingGlvizEGL.so" )
            add_definitions( -DLOCO_BACKEND_PHYSICS_MUJOCO="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsMUJOCO.so" )
            add_definitions( -DLOCO_BACKEND_PHYSICS_BULLET="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsBULLET.so" )
            add_definitions( -DLOCO_BACKEND_PHYSICS_RAISIM="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsRAISIM.so" )
            add_definitions( -DLOCO_BACKEND_PHYSICS_DART="${LOCO_CORE_LIBRARIES_PATH}liblocoPhysicsDART.so" )
        endif ()
    endif()
    ################################################################################################
endif()

set( LOCO_MUJOCO_SRCS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/loco_common_mujoco.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/loco_simulation_mujoco.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/primitives/loco_single_body_collider_adapter_mujoco.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/primitives/loco_single_body_constraint_adapter_mujoco.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/primitives/loco_single_body_adapter_mujoco.cpp" )

set( LOCO_MUJOCO_INCLUDE_DIRS
     "${CMAKE_CURRENT_SOURCE_DIR}/include"
     "${CMAKE_SOURCE_DIR}/core/include"
     "${CMAKE_SOURCE_DIR}/core/ext/json"
     "${CMAKE_SOURCE_DIR}/core/ext/tinyxml2"
     "${CMAKE_SOURCE_DIR}/ext/spdlog/include"
     "${CMAKE_SOURCE_DIR}/ext/tiny_math/include"
     "${CMAKE_SOURCE_DIR}/ext/tiny_utils/include" )
include_directories( ${LOCO_MUJOCO_INCLUDE_DIRS} )

if ( WIN32 )
    # Path to mujoco library in the windows system
    include_directories( "C:/.mujoco/mujoco200_win64/include" )
    link_directories( "C:/.mujoco/mujoco200_win64/bin" )
elseif ( UNIX )
    if ( APPLE )
        # Apple kind of needs to specify this (for some systems)
        include_directories( /usr/local/include )
        link_directories( /usr/local/lib )
        # Path to mujoco library in the macos system
        include_directories( "~/.mujoco/mujoco200_macos/include" )
        link_directories( "~/.mujoco/mujoco200_macos/bin" )
    else ()
        # Path to mujoco library in the linux system
        include_directories( "~/.mujoco/mujoco200_linux/include" )
        link_directories( "~/.mujoco/mujoco200_linux/bin" )
    endif()
endif()

# Extra flags for mujoco configuration
set( CMAKE_CXX_FLAGS "-mavx -pthread ${CMAKE_CXX_FLAGS}" )

if ( WIN32 )
    get_filename_component( LOCO_MUJOCO_LICENSE_ABSOLUTE_PATH "C:/.mujoco/mjkey.txt" ABSOLUTE )
elseif ( UNIX )
    get_filename_component( LOCO_MUJOCO_LICENSE_ABSOLUTE_PATH "~/.mujoco/mjkey.txt" ABSOLUTE )
endif()

add_definitions( -DLOCO_MUJOCO_LICENSE_FILE="${LOCO_MUJOCO_LICENSE_ABSOLUTE_PATH}" )

message( "LOCO::MUJOCO >>> MuJoCo license path: ${LOCO_MUJOCO_LICENSE_ABSOLUTE_PATH}" )

add_library( locoPhysicsMUJOCO SHARED
             ${LOCO_MUJOCO_SRCS} )
target_link_libraries( locoPhysicsMUJOCO
                       loco_core
                       mujoco200nogl )

if ( LOCO_MUJOCO_IS_MASTER_PROJECT AND LOCO_CORE_BUILD_TESTS )
    enable_testing()
    add_subdirectory( tests )
endif()

if ( LOCO_MUJOCO_IS_MASTER_PROJECT )
    message( "|---------------------------------------------------------|" )
    message( "|      LOCOMOTION SIMULATION TOOLKIT (MuJoCo backend)     |" )
    message( "|---------------------------------------------------------|" )
    message( "
                                   .--~~,__   (MuJoCo)
                      :-....,-------`~~'._.'
                       `-,,,  ,_      ;'~U'
                        _,-' ,'`-__; '--.
                       (_/'~~      ''''(;" )
endif()