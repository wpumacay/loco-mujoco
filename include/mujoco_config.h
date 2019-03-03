
#pragma once

#include <string>

#ifndef TYSOC_MUJOCO_LIB
    #define TYSOC_MUJOCO_LIB "./libtysocPhysicsMujoco.so"
#endif

#ifndef TYSOC_MUJOCO_VIZ_LIB
    #define TYSOC_MUJOCO_VIZ_LIB "./mujocoviz/libtysocRenderingMjcViz.so"
#endif

namespace tysoc {
namespace config {

    namespace rendering
    {
        const std::string MUJOCOVIZ = std::string( TYSOC_MUJOCO_VIZ_LIB );
    }

    namespace physics
    {
        const std::string MUJOCO = std::string( TYSOC_MUJOCO_LIB );
    }

}}