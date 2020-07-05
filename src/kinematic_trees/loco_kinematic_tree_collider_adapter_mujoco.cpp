
#include <kinematic_trees/loco_kinematic_tree_collider_adapter_mujoco.h>

namespace loco {
namespace kintree {

    TMujocoKinematicTreeColliderAdapter::TMujocoKinematicTreeColliderAdapter( TKinematicTreeCollider* collider_ref )
        : TIKinematicTreeColliderAdapter( collider_ref )
    {
        m_Size = m_ColliderRef->size();
        m_Size0 = m_ColliderRef->size();
    }

    TMujocoKinematicTreeColliderAdapter::~TMujocoKinematicTreeColliderAdapter()
    {
        m_MjcGeomId = -1;
        m_MjcGeomMeshId = -1;
        m_MjcGeomMeshVertNum = -1;
        m_MjcGeomMeshFaceNum = -1;
        m_MjcGeomMeshVertStartAddr = -1;
        m_MjcGeomMeshFaceStartAddr = -1;

        m_MjcModelRef = nullptr;
        m_MjcDataRef = nullptr;
        m_MjcfElementResources = nullptr;
        m_MjcfElementAssetResources = nullptr;
    }

    void TMujocoKinematicTreeColliderAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_ColliderRef, "TMujocoKinematicTreeColliderAdapter::Build >>> must have a valid collison-object (got nullptr instead)" );

        m_MjcfElementResources = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_GEOM_TAG, parsing::eSchemaType::MJCF );
        m_MjcfElementResources->SetString( "name", m_ColliderRef->name() );
        // Set local transform of this collider w.r.t. to its parent body --------------------------
        auto local_tf = m_ColliderRef->local_tf();
        const auto rel_position = local_tf.col( 3 );
        const auto rel_quaternion = tinymath::quaternion( TMat3( local_tf ) );
        m_MjcfElementResources->SetVec3( "pos", rel_position );
        m_MjcfElementResources->SetVec4( "quat", mujoco::quat_to_mjcQuat( rel_quaternion ) );
        // -----------------------------------------------------------------------------------------
        m_MjcfElementResources->SetString( "type", mujoco::enumShape_to_mjcShape( m_ColliderRef->shape() ) );
        m_MjcfElementResources->SetInt( "contype", m_ColliderRef->collisionGroup() );
        m_MjcfElementResources->SetInt( "conaffinity", m_ColliderRef->collisionMask() );
        m_MjcfElementResources->SetVec3( "friction", m_ColliderRef->data().friction );
        auto array_size = mujoco::size_to_mjcSize( m_ColliderRef->shape(), m_ColliderRef->size() );
        if ( array_size.ndim > 0 )
            m_MjcfElementResources->SetArrayFloat( "size", array_size );

        const eShapeType shape = m_ColliderRef->shape();
        if ( shape == eShapeType::CONVEX_MESH )
        {
            auto& mesh_data = m_ColliderRef->data().mesh_data;
            if ( mesh_data.filename != "" )
            {
                const std::string mesh_file = mesh_data.filename;
                const std::string mesh_id = tinyutils::GetFilenameNoExtension( mesh_file );
                const auto mesh_scale = m_ColliderRef->size();

                m_MjcfElementAssetResources = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_MESH_TAG, parsing::eSchemaType::MJCF );
                m_MjcfElementAssetResources->SetString( "name", mesh_id );
                m_MjcfElementAssetResources->SetString( "file", mesh_file );
                m_MjcfElementAssetResources->SetVec3( "scale", mesh_scale );
                m_MjcfElementResources->SetString( "mesh", mesh_id );
            }
            else
            {
                LOCO_CORE_ERROR( "TMujocoKinematicTreeColliderAdapter::Build >>> kintree-mesh-collider {0} requires a \
                                  filename to be provided by the user. Creating a dummy tetrahedron instead.", m_ColliderRef->name() );

                const std::string mesh_file = m_ColliderRef->name() + ".msh";
                const std::string mesh_id = m_ColliderRef->name() + "_asset";
                const auto mesh_scale = m_ColliderRef->size();
                mesh_data.vertices = { 0.0f, 0.0f, 0.0f,
                                       1.0f, 0.0f, 0.0f,
                                       0.0f, 1.0f, 0.0f,
                                       0.0f, 0.0f, 1.0f };
                mesh_data.faces = { 0, 2, 1,
                                    0, 1, 3,
                                    1, 2, 3,
                                    0, 3, 2 };
                const auto& mesh_vertices = mesh_data.vertices;
                const auto& mesh_faces = mesh_data.faces;
                mujoco::SaveMeshToBinary( mesh_file, mesh_vertices, mesh_faces );

                m_MjcfElementAssetResources = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_MESH_TAG, parsing::eSchemaType::MJCF );
                m_MjcfElementAssetResources->SetString( "name", mesh_id );
                m_MjcfElementAssetResources->SetString( "file", mesh_file );
                m_MjcfElementAssetResources->SetVec3( "scale", mesh_scale );
                m_MjcfElementResources->SetString( "mesh", mesh_id );
            }
        }

        // If parent-body has only inertia.mass set, then we'll deal with it computing an appropriate density
        if ( auto parent_body = m_ColliderRef->parent() )
        {
            const auto& inertia = parent_body->data().inertia;
            if ( ( inertia.mass > loco::EPS ) && 
                 ( ( inertia.ixx <= loco::EPS ) || ( inertia.iyy <= loco::EPS ) || ( inertia.izz <= loco::EPS ) ||
                   ( inertia.ixy <= -loco::EPS ) || ( inertia.ixz <= -loco::EPS ) || ( inertia.iyz <= -loco::EPS ) ) )
            {
                if ( shape == eShapeType::CONVEX_MESH )
                    LOCO_CORE_WARN( "TMujocoKinematicTreeColliderAdapter::Build >>> can't compute inertia of mesh-collider {0}", m_ColliderRef->name() );

                const auto volume = mujoco::compute_primitive_volume( shape, m_ColliderRef->size() );
                const auto density = inertia.mass / volume;
                m_MjcfElementResources->SetFloat( "density", density );
            }
        }
    }

    void TMujocoKinematicTreeColliderAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeColliderAdapter::Initialize >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeColliderAdapter::Initialize >>> must have a valid mjData reference" );

        m_MjcGeomId = mj_name2id( m_MjcModelRef, mjOBJ_GEOM, m_ColliderRef->name().c_str() );
        if ( m_MjcGeomId < 0 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeColliderAdapter::Initialize >>> couldn't find associated \
                              mjc-geom for collider {0} (returned mjc-geom-id < 0)", m_ColliderRef->name() );
            return;
        }

        m_MjcGeomRbound = m_MjcModelRef->geom_rbound[m_MjcGeomId];
        // Grab information required for mesh-colliders (used for resizing purposes later)
        if ( m_ColliderRef->shape() == eShapeType::CONVEX_MESH )
        {
            m_MjcGeomMeshId = m_MjcModelRef->geom_dataid[m_MjcGeomId];
            if ( m_MjcGeomMeshId < 0 )
            {
                LOCO_CORE_ERROR( "TMujocoKinematicTreeColliderAdapter::Initialize >>> couldn't link to a \
                                  mjc-geom-mesh for kintree-mesh-collider {0}", m_ColliderRef->name() );
                return;
            }

            m_MjcGeomMeshVertNum = m_MjcModelRef->mesh_vertnum[m_MjcGeomMeshId];
            m_MjcGeomMeshFaceNum = m_MjcModelRef->mesh_facenum[m_MjcGeomMeshId];
            m_MjcGeomMeshVertStartAddr = m_MjcModelRef->mesh_vertadr[m_MjcGeomMeshId];
            m_MjcGeomMeshFaceStartAddr = m_MjcModelRef->mesh_faceadr[m_MjcGeomMeshId];
        }
    }

    void TMujocoKinematicTreeColliderAdapter::SetLocalTransform( const TMat4& local_tf )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeColliderAdapter::SetLocalTransform >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeColliderAdapter::SetLocalTransform >>> must have a valid mjData reference" );

        if ( m_MjcGeomId < 0 )
            return;

        const auto rel_position = local_tf.col( 3 );
        const auto rel_quaternion = tinymath::quaternion( TMat3( local_tf ) );
        m_MjcModelRef->geom_pos[3 * m_MjcGeomId + 0] = rel_position.x();
        m_MjcModelRef->geom_pos[3 * m_MjcGeomId + 1] = rel_position.y();
        m_MjcModelRef->geom_pos[3 * m_MjcGeomId + 2] = rel_position.z();
        m_MjcModelRef->geom_quat[4 * m_MjcGeomId + 0] = rel_quaternion.w();
        m_MjcModelRef->geom_quat[4 * m_MjcGeomId + 1] = rel_quaternion.x();
        m_MjcModelRef->geom_quat[4 * m_MjcGeomId + 2] = rel_quaternion.y();
        m_MjcModelRef->geom_quat[4 * m_MjcGeomId + 3] = rel_quaternion.z();
    }

    void TMujocoKinematicTreeColliderAdapter::ChangeSize( const TVec3& new_size )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeColliderAdapter::Initialize >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeColliderAdapter::Initialize >>> must have a valid mjData reference" );

        if ( tinymath::allclose( m_Size, new_size ) )
            return;
        if ( m_MjcGeomId < 0 )
            return;

        m_Size = new_size;
        if ( m_ColliderRef->shape() == eShapeType::CONVEX_MESH )
            _ResizeMesh( new_size );
        else
            _ResizePrimitive( new_size );
    }

    void TMujocoKinematicTreeColliderAdapter::_ResizeMesh( const TVec3& new_size )
    {
        // Size given by the user are scales (for kintree-mesh-colliders)
        const TVec3 effective_scale = { std::max( 1e-3f, new_size.x() / m_Size0.x() ),
                                        std::max( 1e-3f, new_size.y() / m_Size0.y() ),
                                        std::max( 1e-3f, new_size.z() / m_Size0.z() ) };

        TVec3 aabb_min( 1e6f, 1e6f, 1e6f );
        TVec3 aabb_max( -1e6f, -1e6f, -1e6f );
        for ( ssize_t v = 0; v < m_MjcGeomMeshVertNum; v++ )
        {
            m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 0] *= effective_scale.x();
            m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 1] *= effective_scale.y();
            m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 2] *= effective_scale.z();

            aabb_min.x() = std::min( aabb_min.x(), m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 0] );
            aabb_min.y() = std::min( aabb_min.y(), m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 1] );
            aabb_min.z() = std::min( aabb_min.z(), m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 2] );

            aabb_max.x() = std::max( aabb_max.x(), m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 0] );
            aabb_max.y() = std::max( aabb_max.y(), m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 1] );
            aabb_max.z() = std::max( aabb_max.z(), m_MjcModelRef->mesh_vert[m_MjcGeomMeshVertStartAddr + 3 * v + 2] );
        }
        m_MjcModelRef->geom_rbound[m_MjcGeomId] = 0.5f * ( aabb_max - aabb_min ).length();
        // New size becomes previous size for next resizing operation
        m_Size0 = new_size;

        // @todo: check if updating mass is handled by mujoco, or should we update inertial properties
    }

    void TMujocoKinematicTreeColliderAdapter::_ResizePrimitive( const TVec3& new_size )
    {
        const auto shape = m_ColliderRef->shape();
        const auto arr_size = mujoco::size_to_mjcSize( shape, new_size );
        for ( ssize_t i = 0; i < arr_size.ndim; i++ )
            m_MjcModelRef->geom_size[3 * m_MjcGeomId + i] = arr_size[i];
        m_MjcModelRef->geom_rbound[m_MjcGeomId] = mujoco::compute_primitive_rbound( shape, new_size );
    }

    void TMujocoKinematicTreeColliderAdapter::ChangeCollisionGroup( int collision_group )
    {
        if ( m_MjcGeomId >= 0 )
            m_MjcModelRef->geom_contype[m_MjcGeomId] = collision_group;
    }

    void TMujocoKinematicTreeColliderAdapter::ChangeCollisionMask( int collision_mask )
    {
        if ( m_MjcGeomId >= 0 )
            m_MjcModelRef->geom_conaffinity[m_MjcGeomId] = collision_mask;
    }
}}