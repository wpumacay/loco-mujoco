#pragma once

#include <loco_common.h>
// Main mujoco API
#include <mujoco.h>

#ifndef LOCO_MUJOCO_LICENSE_FILE
    #define LOCO_MUJOCO_LICENSE_FILE "~/.mujoco/mjkey.txt"
#endif

namespace loco {
namespace mujoco {

    const std::string LOCO_MUJOCO_LICENSE = LOCO_MUJOCO_LICENSE_FILE;

    struct MjcModelDeleter
    {
        void operator()( mjModel* model ) const;
    };

    struct MjcDataDeleter
    {
        void operator()( mjData* data ) const;
    };
}}