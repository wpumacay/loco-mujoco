
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
            case eShapeType::PLANE          : return { 0.5f * size.x(), 0.5f * size.y(), 1.0f };
            case eShapeType::BOX            : return { 0.5f * size.x(), 0.5f * size.y(), 0.5f * size.z() };
            case eShapeType::SPHERE         : return { size.x() };
            case eShapeType::CYLINDER       : return { size.x(), 0.5f * size.y() };
            case eShapeType::CAPSULE        : return { size.x(), 0.5f * size.y() };
            case eShapeType::ELLIPSOID      : return { size.x(), size.y(), size.z() };
            case eShapeType::CONVEX_MESH    : return {};
            case eShapeType::TRIANGULAR_MESH: return {};
            case eShapeType::HEIGHTFIELD         : return {};
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
            case eShapeType::BOX            : return "box";
            case eShapeType::PLANE          : return "plane";
            case eShapeType::SPHERE         : return "sphere";
            case eShapeType::CYLINDER       : return "cylinder";
            case eShapeType::CAPSULE        : return "capsule";
            case eShapeType::ELLIPSOID      : return "ellipsoid";
            case eShapeType::CONVEX_MESH    : return "mesh";
            case eShapeType::TRIANGULAR_MESH: return "trimesh";
            case eShapeType::HEIGHTFIELD         : return "hfield";
        }

        LOCO_CORE_ERROR( "enumShape_to_mjcShape >>> unsupported shape: {0}", ToString( shape ) );
        return "";
    }

    // @todo: move to loco-core
    // @todo: check against mujoco-rbound (at least, capsules are different)
    double compute_primitive_rbound( const eShapeType& shape, const TVec3& size )
    {
        switch ( shape )
        {
            case eShapeType::PLANE      : return 0.0; // mujoco handles planes in a special way
            case eShapeType::BOX        : return 0.5 * std::sqrt( size.x() * size.x() + size.y() * size.y() + size.z() * size.z() );
            case eShapeType::SPHERE     : return size.x();
            case eShapeType::CYLINDER   : return std::sqrt( size.x() * size.x() + 0.25 * size.y() * size.y() );
            case eShapeType::CAPSULE    : return size.x() + 0.5 * size.y();
            case eShapeType::ELLIPSOID  : return std::max( std::max( size.x(), size.y() ), size.z() );
        }

        LOCO_CORE_ERROR( "compute_primitive_rbound >>> unsupported shape: {0}", ToString( shape ) );
        return 1.0;
    }

    // @todo: move to loco-core
    double compute_primitive_volume( const eShapeType& shape, const TVec3& size )
    {
        switch ( shape )
        {
            case eShapeType::BOX        : return size.x() * size.y() * size.z();
            case eShapeType::SPHERE     : return (4. / 3.) * loco::PI * size.x() * size.x() * size.x();
            case eShapeType::CYLINDER   : return loco::PI * size.x() * size.x() * size.y();
            case eShapeType::CAPSULE    : return loco::PI * size.x() * size.x() * size.y() + 
                                                   (4. / 3.) * loco::PI * size.x() * size.x() * size.x();
            case eShapeType::ELLIPSOID  : return (4. / 3.) * loco::PI * size.x() * size.y() * size.z();
        }

        LOCO_CORE_ERROR( "compute_primitive_volume >>> unsupported shape: {0}", ToString( shape ) );
        return 1.0;
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

        arr_sf.ndim = array_size;
        for ( size_t i = 0; i < array_size; i++ )
            arr_sf[i] = array_num[i];
        return arr_sf;
    }

    void SaveMeshToBinary( const std::string& mesh_file,
                           const std::vector<float>& mesh_vertices,
                           const std::vector<int>& mesh_faces )
    {
        std::ofstream fhandle( mesh_file.c_str(), std::ofstream::out | std::ofstream::binary );
        if ( !fhandle )
        {
            LOCO_CORE_ERROR( "SaveMeshToBinary >>> couldn't save user-defined binary mesh to the \
                              filepath {0}", mesh_file );
            return;
        }

        if ( mesh_vertices.size() % 3 != 0 )
            LOCO_CORE_ERROR( "SaveMeshToBinary >>> there must be 3 elements per vertex, got {0}/3 (mesh: {1})", mesh_vertices.size(), mesh_file );
        if ( mesh_faces.size() % 3 != 0 )
            LOCO_CORE_ERROR( "SaveMeshToBinary >>> there must be 3 elements per face, got {0}/3 (mesh: {1})", mesh_faces.size(), mesh_file );

        const int32_t nvertex = mesh_vertices.size() / 3;
        const int32_t nnormal = 0;
        const int32_t ntexcoord = 0;
        const int32_t nface = mesh_faces.size() / 3;

        fhandle.write( (const char*)&nvertex, sizeof( int32_t ) );
        fhandle.write( (const char*)&nnormal, sizeof( int32_t ) );
        fhandle.write( (const char*)&ntexcoord, sizeof( int32_t ) );
        fhandle.write( (const char*)&nface, sizeof( int32_t ) );
        fhandle.write( (const char*)mesh_vertices.data(), sizeof( float ) * mesh_vertices.size() );
        fhandle.write( (const char*)mesh_faces.data(), sizeof( int ) * mesh_faces.size() );
        fhandle.close();

        if ( !fhandle.good() )
            LOCO_CORE_ERROR( "SaveMeshToBinary >>> there was an error while trying to save mesh {0}", mesh_file );
    }
}}