
#pragma once

#include <string>

#ifndef TYSOC_BACKEND_PHYSICS_MUJOCO
    #define TYSOC_BACKEND_PHYSICS_MUJOCO "../libtysocPhysicsMujoco.so"
#endif

#ifndef TYSOC_BACKEND_GRAPHICS_MJCVIZ
    #define TYSOC_BACKEND_GRAPHICS_MJCVIZ "../libtysocRenderingMjcViz.so"
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