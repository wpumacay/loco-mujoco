
#pragma once

#include <string>

// @TODO: Add checks for windows
#ifndef TYSOC_BACKEND_PHYSICS_MUJOCO
    #ifdef __APPLE__
        #define TYSOC_BACKEND_PHYSICS_MUJOCO "../libtysocPhysicsMujoco.dylib"
    #else
        #define TYSOC_BACKEND_PHYSICS_MUJOSO "../libtysocPhysicsMujoco.so"
    #endif
#endif

// @TODO: Add checks for windows
#ifndef TYSOC_BACKEND_GRAPHICS_MJCVIZ
    #ifdef __APPLE__
        #define TYSOC_BACKEND_GRAPHICS_MJCVIZ "../libtysocRenderingMjcViz.dylib"
    #else
        #define TYSOC_BACKEND_GRAPHICS_MJCVIZ "../libtysocRenderingMjcViz.so"
    #endif
#endif

namespace tysoc {
namespace config {

    namespace rendering
    {
        const std::string MJCVIZ = std::string( TYSOC_BACKEND_GRAPHICS_MJCVIZ );
    }

    namespace physics
    {
        const std::string MUJOCO = std::string( TYSOC_BACKEND_PHYSICS_MUJOCO );
    }

}}