
#pragma once

// main mujoco API
#include <mujoco.h>

// extra helper functionality
#include <tysoc_common.h>
#include <utils/parsers/mjcf/mjcf.h>
#include <utils/parsers/rlsim/rlsim.h>
#include <utils/parsers/urdf/urdf.h>
#include <components/data.h>

// and some configurations
#include <mujoco_config.h>

namespace tysoc {
namespace mujoco {

    TVec4 quat2MjcfQuat( const TVec4& quat );

    /* converts a type enum to the mjcf string type expected by its format */
    std::string shapeType2MjcfShapeType( const eShapeType& type );

    /* converts a vec3 from standard size to the mjcf size expected by its format */
    TVec3 size2MjcfSize( const eShapeType& type, const TVec3& size );

    /* compute rbound given a shape of some type and size */
    TScalar computeRbound( const eShapeType& type, const TVec3& size );

    /* converts joint-type enum to mjc-type enum */
    std::string enumJointToMjcType( const eJointType& type );

    /* converts shape-type enum to mjc-geom enum */
    std::string enumShapeToMjcType( const eShapeType& type );

    /* converts actuator-type enum to mjc-actuator enum */
    std::string enumActuatorToMjcType( const eActuatorType& type );

}}