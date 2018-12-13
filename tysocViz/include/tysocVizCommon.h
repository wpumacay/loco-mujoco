
#pragma once

#include <LCommon.h>
#include <tysoc_common.h>

namespace tysoc {
namespace viz {

    engine::LMat4 fromTMat3( const TMat3& mat );
    engine::LVec3 fromTVec3( const TVec3& vec );

}}