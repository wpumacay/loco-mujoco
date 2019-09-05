
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

// and some configurations
#include <mujoco_config.h>

namespace tysoc {
namespace mujoco {

    TVec4 quat2MjcfQuat( const TVec4& quat );

}}