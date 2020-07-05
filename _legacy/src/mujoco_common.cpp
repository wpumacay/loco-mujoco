
#include <mujoco_common.h>

namespace tysoc {
namespace mujoco {

    TVec4 quat2MjcfQuat( const TVec4& quat ) 
    { 
        return TVec4( quat.w, quat.x, quat.y, quat.z ); 
    }

    std::string shapeType2MjcfShapeType( const eShapeType& type )
    {
        switch ( type )
        {
            case eShapeType::PLANE      : return "plane";
            case eShapeType::BOX        : return "box";
            case eShapeType::SPHERE     : return "sphere";
            case eShapeType::CYLINDER   : return "cylinder";
            case eShapeType::CAPSULE    : return "capsule";
            case eShapeType::MESH       : return "mesh";
            case eShapeType::HEIGHTFIELD     : return "hfield";

            default : return "none";
        }
    }

    TVec3 size2MjcfSize( const eShapeType& type, const TVec3& size )
    {
        // Recall that the size dimensions are stored as required, e.g. for a cylinder, the
        // dimension for height is actually the z-axis, but we only need two dimension (radius,height)
        // so we only use the first two to represent the dimensions: in short: [radius-height-unused ]
        // instead of [radius-radius-height], as are the dimensions associated with the AABB in some
        // other engines, like in bullet

        switch ( type )
        {
            /* have to halve the size as it expectes half-width and half-depth */
            case eShapeType::PLANE : return { 0.5f * size.x, 0.5f * size.y, 0.5f * size.z };

            /* have to halve the size as it expectes half-width, half-depth and half-height */
            case eShapeType::BOX : return { 0.5f * size.x, 0.5f * size.y, 0.5f * size.z };

            /* same size, as it expects radius only */
            case eShapeType::SPHERE : return size;

            /* halve only the height dimension (stored in y) */
            case eShapeType::CYLINDER : return { size.x, 0.5f * size.y, size.z };

            /* halve only the height dimension (stored in y) */
            case eShapeType::CAPSULE : return { size.x, 0.5f * size.y, size.z };

            /* same size, as it is ignored by the engine (uses actual (asset) mesh-sizes instead */
            case eShapeType::MESH : return size;

            /* same size, as it is ignored by the engine (uses actual (asset) hfield-sizes instead */
            case eShapeType::HEIGHTFIELD : return size;

            default : return size;
        }
    }

    TScalar computeRbound( const eShapeType& type, const TVec3& size )
    {
        return 1.0f;
    }

    std::string enumJointToMjcType( const eJointType& type )
    {
        if ( type == eJointType::FIXED ) return ""; // fixed joints are just skipped in mjcf format
        if ( type == eJointType::REVOLUTE ) return "hinge";
        if ( type == eJointType::PRISMATIC ) return "slide";
        if ( type == eJointType::SPHERICAL ) return "ball";
        if ( type == eJointType::PLANAR ) return ""; // planar joints are not supported, so skip for now
        if ( type == eJointType::FREE ) return "free";

        std::cout << "WARNING> invalid eJointType enum given" << std::endl;

        return "";
    }

    std::string enumShapeToMjcType( const eShapeType& type )
    {
        if ( type == eShapeType::BOX ) return "box";
        if ( type == eShapeType::PLANE ) return "plane";
        if ( type == eShapeType::SPHERE ) return "sphere";
        if ( type == eShapeType::CYLINDER ) return "cylinder";
        if ( type == eShapeType::CAPSULE ) return "capsule";
        if ( type == eShapeType::ELLIPSOID ) return "ellipsoid";
        if ( type == eShapeType::MESH ) return "mesh";
        if ( type == eShapeType::HEIGHTFIELD ) return "hfield"; // @todo: move to mujoco_common

        std::cout << "WARNING> invalid eShapeType enum given" << std::endl;

        return "";
    }

    std::string enumActuatorToMjcType( const eActuatorType& type )
    {
        if ( type == eActuatorType::TORQUE ) return "motor";

        std::cout << "WARNING> unsupported type of actuator: " << tysoc::toString( type ) << std::endl;

        return "";
    }

}}