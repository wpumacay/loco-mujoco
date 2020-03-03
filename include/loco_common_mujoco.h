#pragma once

#include <loco_common.h>
#include <components/loco_data.h>
// Main mujoco API
#include <mujoco.h>

#ifndef LOCO_MUJOCO_LICENSE_FILE
    #define LOCO_MUJOCO_LICENSE_FILE "~/.mujoco/mjkey.txt"
#endif

namespace loco {
namespace mujoco {

    const std::string LOCO_MUJOCO_LICENSE = LOCO_MUJOCO_LICENSE_FILE;

    const std::string LOCO_MJCF_GEOM_TAG = "geom";
    const std::string LOCO_MJCF_BODY_TAG = "body";
    const std::string LOCO_MJCF_JOINT_TAG = "joint";
    const std::string LOCO_MJCF_MESH_TAG = "mesh";
    const std::string LOCO_MJCF_HFIELD_TAG = "hfield";
    const std::string LOCO_MJCF_ASSET_TAG = "asset";
    const std::string LOCO_MJCF_WORLDBODY_TAG = "worldbody";

    struct MjcModelDeleter
    {
        void operator()( mjModel* model ) const;
    };

    struct MjcDataDeleter
    {
        void operator()( mjData* data ) const;
    };

    TVec4 quat_to_mjcQuat( const TVec4& quat );

    TSizef size_to_mjcSize( const eShapeType& shape, const TVec3& size );

    std::string enumJoint_to_mjcJoint( const eJointType& joint );

    std::string enumShape_to_mjcShape( const eShapeType& shape );

    float compute_primitive_rbound( const eShapeType& shape, const TVec3& size );

    float compute_primitive_volume( const eShapeType& shape, const TVec3& size );

    TSizef mjarray_to_sizef( const mjtNum* array_num, size_t array_size );

}}