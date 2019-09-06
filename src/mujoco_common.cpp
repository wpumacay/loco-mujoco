
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

            /* same size, as it is ignored by the engine (uses actual mesh-sizes instead */
            case eShapeType::MESH : return size;

            default : return size;
        }
    }

    TScalar computeRbound( const eShapeType& type, const TVec3& size )
    {
        return 1.0f;
    }

}}