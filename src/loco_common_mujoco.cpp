
#include <loco_common_mujoco.h>

namespace loco {
namespace mujoco {

    void MjcModelDeleter::operator()( mjModel* model ) const
    {
        LOCO_CORE_ASSERT( model != nullptr, "MjcModelDeleter >>> should have a valid mjModel reference to release" );
        mj_deleteModel( model );
    }

    void MjcDataDeleter::operator()( mjData* data ) const
    {
        LOCO_CORE_ASSERT( data != nullptr, "MjcDataDeleter >>> should have a valid mjData reference to release" );
        mj_deleteData( data );
    }

    TVec4 quat_to_mjcQuat( const TVec4& quat )
    {
        return TVec4( quat.w(), quat.x(), quat.y(), quat.z() );
    }

    TSizef size_to_mjcSize( const eShapeType& shape, const TVec3& size )
    {
        switch ( shape )
        {
            case eShapeType::PLANE      : return { 0.5f * size.x(), 0.5f * size.y(), 1.0f };
            case eShapeType::BOX        : return { 0.5f * size.x(), 0.5f * size.y(), 0.5f * size.z() };
            case eShapeType::SPHERE     : return { size.x() };
            case eShapeType::CYLINDER   : return { size.x(), 0.5f * size.y() };
            case eShapeType::CAPSULE    : return { size.x(), 0.5f * size.y() };
            case eShapeType::ELLIPSOID  : return { size.x(), size.y(), size.z() };
            case eShapeType::MESH       : return {};
            case eShapeType::HFIELD     : return {};
        }

        LOCO_CORE_ERROR( "size_to_mjcSize >>> unsupported shape: {0}", ToString( shape ) );
        return {};
    }

    std::string enumJoint_to_mjcJoint( const eJointType& joint )
    {
        switch ( joint )
        {
            case eJointType::FREE       : return "free";
            case eJointType::REVOLUTE   : return "hinge";
            case eJointType::PRISMATIC  : return "slide";
            case eJointType::SPHERICAL  : return "ball";
            case eJointType::PLANAR     : return ""; // planar constraints are handled via 2 composite slide constrains
            case eJointType::FIXED      : return ""; // fixed constrains are handled by not placing any dof
        }

        LOCO_CORE_ERROR( "enumJoint_to_mjcJoint >>> unsupported joint: {0}", ToString( joint ) );
        return "";
    }

    std::string enumShape_to_mjcShape( const eShapeType& shape )
    {
        switch ( shape )
        {
            case eShapeType::BOX        : return "box";
            case eShapeType::PLANE      : return "plane";
            case eShapeType::SPHERE     : return "sphere";
            case eShapeType::CYLINDER   : return "cylinder";
            case eShapeType::CAPSULE    : return "capsule";
            case eShapeType::ELLIPSOID  : return "ellipsoid";
            case eShapeType::MESH       : return "mesh";
            case eShapeType::HFIELD     : return "hfield";
        }

        LOCO_CORE_ERROR( "enumShape_to_mjcShape >>> unsupported shape: {0}", ToString( shape ) );
        return "";
    }

    float compute_primitive_rbound( const eShapeType& shape, const TVec3& size )
    {
        switch ( shape )
        {
            case eShapeType::PLANE      : return 0.5f * std::sqrt( size.x() * size.x() + size.y() * size.y() );
            case eShapeType::BOX        : return 0.5f * std::sqrt( size.x() * size.x() + size.y() * size.y() + size.z() * size.z() );
            case eShapeType::SPHERE     : return size.x();
            case eShapeType::CYLINDER   : return std::sqrt( 2 * size.x() * size.x() + 0.25f * size.y() * size.y() );
            case eShapeType::CAPSULE    : return std::sqrt( 2 * size.x() * size.x() + 0.25f * size.y() * size.y() );
            case eShapeType::ELLIPSOID  : return std::sqrt( size.x() * size.x() + size.y() * size.y() + size.z() * size.z() );
        }

        LOCO_CORE_ERROR( "compute_rbound >>> unsupported shape: {0}", ToString( shape ) );
        return 1.0f;
    }

    TSizef mjarray_to_sizef( const mjtNum* array_num, size_t array_size )
    {
        TSizef arr_sf;
        if ( array_size > loco::MAX_NDIM )
        {
            LOCO_CORE_ERROR( "mjarray_to_sizef >>> mjarray has dimension {0}, which is larger than \
                              max. {1}. Returning empty array", array_size, loco::MAX_NDIM );
            return arr_sf;
        }

        for ( size_t i = 0; i < array_size; i++ )
            arr_sf[i] = array_num[i];
        return arr_sf;
    }

}}