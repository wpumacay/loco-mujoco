
#pragma once

// main mujoco API
#include <mujoco.h>

// extra helper functionality
#include <tysoc_common.h>
#include <utils/parsers/mjcf/mjcf.h>
#include <utils/parsers/rlsim/rlsim.h>
#include <utils/parsers/urdf/urdf.h>

// some standard helper functions
#include <map>
#include <vector>
#include <queue>
#include <cmath>
#include <random>

#ifndef TYSOCMJC_RESOURCES_PATH
    #define TYSOCMJC_RESOURCES_PATH "../res"
#endif

namespace tysoc {
namespace mujoco {

    // @TODO: Here there be dragons

}}