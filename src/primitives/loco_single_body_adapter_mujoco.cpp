
#include <primitives/loco_single_body_adapter_mujoco.h>

namespace loco {
namespace mujoco {

    ssize_t TMujocoSingleBodyAdapter::s_DetachedNum = 0;

    TMujocoSingleBodyAdapter::TMujocoSingleBodyAdapter( TSingleBody* bodyRef )
        : TISingleBodyAdapter( bodyRef )
    {
        LOCO_CORE_ASSERT( bodyRef, "TMujocoSingleBodyAdapter >>> adaptee (body) should be a valid \
                          refernce (nullptr given)" );
        LOCO_CORE_ASSERT( bodyRef->collider(), "TMujocoSingleBodyAdapter >>> body {0} doesn't have \
                          a valid collider (found nullptr)", bodyRef->name() );

        m_mjcModelRef = nullptr;
        m_mjcDataRef = nullptr;

        m_mjcBodyId = -1;
        m_mjcJointId = -1;
        m_mjcJointQposAdr = -1;
        m_mjcJointQvelAdr = -1;
        m_mjcJointQposNum = -1;
        m_mjcJointQvelNum = -1;
        m_mjcGeomId = -1;

        m_mjcfElementResources = nullptr;
        m_mjcfElementAssetResources = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        const std::string name = ( m_BodyRef ) ? m_BodyRef->name() : "undefined";
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TMujocoSingleBodyAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TMujocoSingleBodyAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TMujocoSingleBodyAdapter::~TMujocoSingleBodyAdapter()
    {
        if ( m_BodyRef )
            m_BodyRef->DetachSim();

        m_BodyRef = nullptr;
        m_ColliderAdapter = nullptr;
        m_ConstraintAdapter = nullptr;

        m_mjcModelRef = nullptr;
        m_mjcDataRef = nullptr;
        m_mjcBodyId = -1;
        m_mjcJointId = -1;
        m_mjcJointQposAdr = -1;
        m_mjcJointQvelAdr = -1;
        m_mjcJointQposNum = -1;
        m_mjcJointQvelNum = -1;

        m_mjcfElementResources = nullptr;
        m_mjcfElementAssetResources = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        const std::string name = ( m_BodyRef ) ? m_BodyRef->name() : "undefined";
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TMujocoSingleBodyAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TMujocoSingleBodyAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TMujocoSingleBodyAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_BodyRef, "TMujocoSingleBodyAdapter::Build >>> must have a valid body-object (got nullptr instead)" );

        // Create collider-resources first
        auto collider = m_BodyRef->collider();
        LOCO_CORE_ASSERT( collider, "TMujocoSingleBodyAdapter::Build >>> single-body {0} doesn't have \
                          a valid collider (nullptr)", m_BodyRef->name() );

        m_ColliderAdapter = std::make_unique<TMujocoSingleBodyColliderAdapter>( collider );
        collider->SetColliderAdapter( m_ColliderAdapter.get() );

        auto mjc_collider_adapter = static_cast<TMujocoSingleBodyColliderAdapter*>( m_ColliderAdapter.get() );
        mjc_collider_adapter->Build();

        if ( auto constraint = m_BodyRef->constraint() )
        {
            const eConstraintType constraint_type = constraint->constraint_type();
            if ( constraint_type == eConstraintType::REVOLUTE )
            {
                m_ConstraintAdapter = std::make_unique<TMujocoSingleBodyRevoluteConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto mjc_constraint_adapter = dynamic_cast<TMujocoSingleBodyRevoluteConstraintAdapter*>( m_ConstraintAdapter.get() );
                mjc_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::PRISMATIC )
            {
                m_ConstraintAdapter = std::make_unique<TMujocoSingleBodyPrismaticConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto mjc_constraint_adapter = dynamic_cast<TMujocoSingleBodyPrismaticConstraintAdapter*>( m_ConstraintAdapter.get() );
                mjc_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::SPHERICAL )
            {
                m_ConstraintAdapter = std::make_unique<TMujocoSingleBodySphericalConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto mjc_constraint_adapter = dynamic_cast<TMujocoSingleBodySphericalConstraintAdapter*>( m_ConstraintAdapter.get() );
                mjc_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::TRANSLATIONAL3D )
            {
                m_ConstraintAdapter = std::make_unique<TMujocoSingleBodyTranslational3dConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto mjc_constraint_adapter = dynamic_cast<TMujocoSingleBodyTranslational3dConstraintAdapter*>( m_ConstraintAdapter.get() );
                mjc_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::UNIVERSAL3D )
            {
                m_ConstraintAdapter = std::make_unique<TMujocoSingleBodyUniversal3dConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto mjc_constraint_adapter = dynamic_cast<TMujocoSingleBodyUniversal3dConstraintAdapter*>( m_ConstraintAdapter.get() );
                mjc_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::PLANAR )
            {
                m_ConstraintAdapter = std::make_unique<TMujocoSingleBodyPlanarConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto mjc_constraint_adapter = dynamic_cast<TMujocoSingleBodyPlanarConstraintAdapter*>( m_ConstraintAdapter.get() );
                mjc_constraint_adapter->Build();
            }
            else
            {
                LOCO_CORE_ERROR( "TMujocoSingleBodyAdapter::Build >>> constraint type {0} not supported", ToString( constraint_type ) );
            }
        }

        // Only dynamic-bodies require actual bodies. Static ones only require geoms (expect meshes, that
        // can be added as static bodies without being pure geoms)
        const bool is_static_mesh = ( m_BodyRef->dyntype() == eDynamicsType::STATIC && 
                                      m_BodyRef->collider()->shape() == eShapeType::MESH );
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC || is_static_mesh )
        {
            m_mjcfElementResources = std::make_unique<parsing::TElement>( LOCO_MJCF_BODY_TAG, parsing::eSchemaType::MJCF );
            m_mjcfElementResources->SetString( "name", m_BodyRef->name() );
            m_mjcfElementResources->SetVec3( "pos", m_BodyRef->pos() );
            m_mjcfElementResources->SetVec4( "quat", quat_to_mjcQuat( m_BodyRef->quat() ) );

            // Inertial mjcf-element is added only if all inertia properties are given (mass + inertia matrix).
            // If only the mass is given, then the density is computed for the collider from the mass and volume.
            const auto& inertia = m_BodyRef->data().inertia;
            if ( ( inertia.mass > loco::EPS ) && ( inertia.ixx > loco::EPS ) && ( inertia.iyy > loco::EPS ) && 
                 ( inertia.izz > loco::EPS ) && ( inertia.ixy > -loco::EPS ) && ( inertia.ixz > -loco::EPS ) &&
                 ( inertia.iyz > -loco::EPS ) )
            {
                auto _inertia_element = m_mjcfElementResources->Add( "inertial" );
                _inertia_element->SetVec3( "pos", { 0.0, 0.0, 0.0 } );
                _inertia_element->SetFloat( "mass", inertia.mass );
                _inertia_element->SetArrayFloat( "fullinertia", { inertia.ixx, inertia.iyy, inertia.izz,
                                                                  inertia.ixy, inertia.ixz, inertia.iyz } );
            }

            LOCO_CORE_ASSERT( mjc_collider_adapter->element_resources(), "TMujocoSingleBodyAdapter::Build >>> \
                              collider must have valid mjcf-resources once built, for body named {0}", m_BodyRef->name() );
            m_mjcfElementResources->Add( parsing::TElement::CloneElement( mjc_collider_adapter->element_resources() ) );

            if ( m_ConstraintAdapter )
            {
                if ( auto mjc_constraint_adapter = dynamic_cast<TIMujocoSingleBodyConstraintAdapter*>( m_ConstraintAdapter.get() ) )
                {
                    auto mjcf_constraint_elements = mjc_constraint_adapter->elements_resources();
                    LOCO_CORE_ASSERT( mjcf_constraint_elements.size() > 0, "TMujocoSingleBodyAdapter::Build >>> \
                                      constraint must have valid mjcf-resources once built, for body named {0}", m_BodyRef->name() );
                    for ( auto mjcf_constraint_element : mjcf_constraint_elements )
                        m_mjcfElementResources->Add( parsing::TElement::CloneElement( mjcf_constraint_element ) );
                }
            }
            else if ( !is_static_mesh )
            {
                auto joint_element = m_mjcfElementResources->Add( "freejoint" );
                joint_element->SetString( "name", m_BodyRef->name() + "_freejnt" );
            }
        }
        else
        {
            LOCO_CORE_ASSERT( mjc_collider_adapter->element_resources(), "TMujocoSingleBodyAdapter::Build >>> \
                              collider must have valid mjcf-resources once built, for body named {0}", m_BodyRef->name() );
            m_mjcfElementResources = parsing::TElement::CloneElement( mjc_collider_adapter->element_resources() );
            m_mjcfElementResources->SetVec3( "pos", m_BodyRef->pos() );
            m_mjcfElementResources->SetVec4( "quat", quat_to_mjcQuat( m_BodyRef->quat() ) );
        }

        if ( auto collider_element_asset_resources = mjc_collider_adapter->element_asset_resources() )
        {
            m_mjcfElementAssetResources = std::make_unique<parsing::TElement>( LOCO_MJCF_ASSET_TAG, parsing::eSchemaType::MJCF );
            m_mjcfElementAssetResources->Add( parsing::TElement::CloneElement( collider_element_asset_resources ) );
        }
    }

    void TMujocoSingleBodyAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_mjcModelRef, "TMujocoSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid mjModel reference", m_BodyRef->name() );
        LOCO_CORE_ASSERT( m_mjcDataRef, "TMujocoSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid mjData reference", m_BodyRef->name() );
        LOCO_CORE_ASSERT( m_ColliderAdapter, "TMujocoSingleBodyAdapter::Initialize >>> body {0} must have \
                          a related mjc-collider-adapter for its collider. Perhaps forgot to call ->Build()?", m_BodyRef->name() );
        // Initialize collider-adapter first (might have geom-id required if static-body)
        auto mjc_collider_adapter = static_cast<TMujocoSingleBodyColliderAdapter*>( m_ColliderAdapter.get() );
        mjc_collider_adapter->Initialize();

        const bool is_static_mesh = ( m_BodyRef->dyntype() == eDynamicsType::STATIC &&
                                      m_BodyRef->collider()->shape() == eShapeType::MESH );
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC || is_static_mesh )
        {
            m_mjcBodyId = mj_name2id( m_mjcModelRef, mjOBJ_BODY, m_BodyRef->name().c_str() );
            if ( m_mjcBodyId < 0 )
            {
                LOCO_CORE_ERROR( "TMujocoSingleBodyAdapter::Initialize >>> couldn't find associated \
                                 mjc-body for single-body {0}", m_BodyRef->name() );
                return;
            }
            if ( m_BodyRef->constraint() )
            {
                m_ConstraintAdapter->Initialize();
            }
            else if ( !is_static_mesh )
            {
                m_mjcJointId = m_mjcModelRef->body_jntadr[m_mjcBodyId];
                if ( m_mjcJointId < 0 )
                {
                    LOCO_CORE_ERROR( "TMujocoSingleBodyAdapter::Initialize >>> couldn't find associated \
                                      mjc-joint (free-joint) for non-static single-body {0}", m_BodyRef->name() );
                    return;
                }

                const auto jnt_type = m_mjcModelRef->jnt_type[m_mjcJointId];
                if ( jnt_type != mjJNT_FREE )
                {
                    LOCO_CORE_ERROR( "TMujocoSingleBodyAdapter::Initialize >>> joint associated to single-body \
                                      {0} must be a free-joint", m_BodyRef->name() );
                }
                else
                {
                    m_mjcJointQposAdr = m_mjcModelRef->jnt_qposadr[m_mjcJointId];
                    m_mjcJointQvelAdr = m_mjcModelRef->jnt_dofadr[m_mjcJointId];
                    m_mjcJointQposNum = 7;
                    m_mjcJointQvelNum = 6;

                    // @todo: is this required? (qpos0 should be computed from pos|quat in xml, which is our desired qpos0)
                    // Set the body's initial configuration
                    const TVec3 position0 = m_BodyRef->pos0();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 0] = position0.x();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 1] = position0.y();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 2] = position0.z();
                    const TVec4 quaternion0 = m_BodyRef->quat0();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 3] = quaternion0.w();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 4] = quaternion0.x();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 5] = quaternion0.y();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 6] = quaternion0.z();
                    SetLinearVelocity( m_BodyRef->linear_vel0() );
                    SetAngularVelocity( m_BodyRef->angular_vel0() );
                }
            }
            else
            {
                const TVec3 position0 = m_BodyRef->pos0();
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = position0.x();
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = position0.y();
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = position0.z();
                const TVec4 quaternion0 = m_BodyRef->quat0();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = quaternion0.w();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = quaternion0.x();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = quaternion0.y();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = quaternion0.z();
            }
        }
        else
        {
            m_mjcGeomId = mjc_collider_adapter->mjc_geom_id();
            LOCO_CORE_ASSERT( m_mjcGeomId != -1, "TMujocoSingleBodyAdapter::Initialize >>> static-body {0} must have \
                              a valid mjc-geom associated with it", m_BodyRef->name() );
            const TVec3 position0 = m_BodyRef->pos0();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 0] = position0.x();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 1] = position0.y();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 2] = position0.z();
            const TVec4 quaternion0 = m_BodyRef->quat0();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 0] = quaternion0.w();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 1] = quaternion0.x();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 2] = quaternion0.y();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 3] = quaternion0.z();
        }
    }

    void TMujocoSingleBodyAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_mjcModelRef, "TMujocoSingleBodyAdapter::Reset >>> {0} must have a valid mjModel reference", m_BodyRef->name() );
        LOCO_CORE_ASSERT( m_mjcDataRef, "TMujocoSingleBodyAdapter::Reset >>> {0} must have a valid mjData reference", m_BodyRef->name() );
        LOCO_CORE_ASSERT( m_mjcBodyId, "TMujocoSingleBodyAdapter::Reset >>> {0} must be linked to a valid mjc-body", m_BodyRef->name() );

        const bool is_static_mesh = ( m_BodyRef->dyntype() == eDynamicsType::STATIC &&
                                      m_BodyRef->collider()->shape() == eShapeType::MESH );
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC || is_static_mesh )
        {
            if ( m_BodyRef->constraint() )
            {
                m_ConstraintAdapter->Reset();
            }
            else if ( !is_static_mesh )
            {
                for ( ssize_t i = 0; i < m_mjcJointQposNum; i++ )
                    m_mjcDataRef->qpos[m_mjcJointQposAdr + i] = m_mjcModelRef->qpos0[m_mjcJointQposAdr + i];
                SetLinearVelocity( m_BodyRef->linear_vel0() );
                SetAngularVelocity( m_BodyRef->angular_vel0() );
            }
            else
            {
                const TVec3 position0 = m_BodyRef->pos0();
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = position0.x();
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = position0.y();
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = position0.z();
                const TVec4 quaternion0 = m_BodyRef->quat0();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = quaternion0.w();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = quaternion0.x();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = quaternion0.y();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = quaternion0.z();
            }
        }
        else
        {
            const TVec3 position0 = m_BodyRef->pos0();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 0] = position0.x();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 1] = position0.y();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 2] = position0.z();
            const TVec4 quaternion0 = m_BodyRef->quat0();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 0] = quaternion0.w();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 1] = quaternion0.x();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 2] = quaternion0.y();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 3] = quaternion0.z();
        }
    }

    void TMujocoSingleBodyAdapter::OnDetach()
    {
        const TVec3 grid_rest_position = DETACHED_REST_GRID_START + 
                                         TVec3( (s_DetachedNum % DETACHED_REST_GRID_SIZE) * DETACHED_REST_GRID_DELTA.x(),
                                                (s_DetachedNum / DETACHED_REST_GRID_SIZE) * DETACHED_REST_GRID_DELTA.y(),
                                                (s_DetachedNum / DETACHED_REST_GRID_SIZE_POW2) * DETACHED_REST_GRID_DELTA.z() );
        m_DetachedRestTransform.set( grid_rest_position, 3 );
        m_Detached = true;
        m_BodyRef = nullptr;
        s_DetachedNum++;
    }

    void TMujocoSingleBodyAdapter::SetTransform( const TMat4& transform )
    {
        const TVec3 position = TVec3( transform.col( 3 ) );
        const TVec4 quaternion = tinymath::quaternion( TMat3( transform ) );
        const bool is_static_mesh = ( m_BodyRef->dyntype() == eDynamicsType::STATIC && 
                                      m_BodyRef->collider()->shape() == eShapeType::MESH );
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC || is_static_mesh )
        {
            LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetTransform >>> {0} must be \
                              linked to a mjc-body", m_BodyRef->name() );
            if ( !m_BodyRef->constraint() && !is_static_mesh )
            {
                LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::SetTransform >>> {0} is a non-static \
                                  single-body, so it requires to have an associated mjc-joint", m_BodyRef->name() );
                m_mjcDataRef->qpos[m_mjcJointQposAdr + 0] = position.x();
                m_mjcDataRef->qpos[m_mjcJointQposAdr + 1] = position.y();
                m_mjcDataRef->qpos[m_mjcJointQposAdr + 2] = position.z();
                m_mjcDataRef->qpos[m_mjcJointQposAdr + 3] = quaternion.w();
                m_mjcDataRef->qpos[m_mjcJointQposAdr + 4] = quaternion.x();
                m_mjcDataRef->qpos[m_mjcJointQposAdr + 5] = quaternion.y();
                m_mjcDataRef->qpos[m_mjcJointQposAdr + 6] = quaternion.z();
            }
            else
            {
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = position.x();
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = position.y();
                m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = position.z();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = quaternion.w();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = quaternion.x();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = quaternion.y();
                m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = quaternion.z();
            }
        }
        else
        {
            LOCO_CORE_ASSERT( m_mjcGeomId >= 0, "TMujocoSingleBodyAdapter::SetTransform >>> {0} must be \
                              linked to a mjc-geom", m_BodyRef->name() );
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 0] = position.x();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 1] = position.y();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 2] = position.z();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 0] = quaternion.w();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 1] = quaternion.x();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 2] = quaternion.y();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 3] = quaternion.z();
        }
    }

    void TMujocoSingleBodyAdapter::SetLinearVelocity( const TVec3& linear_vel )
    {
        // @todo: change to check of constraint-type when supporting single-body constraints, for
        //        now, it just acts as the velocity of a free-body, with nv=6 <> [3xyz,3rpy]
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            if ( !m_BodyRef->constraint() )
            {
                LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetLinearVelocity >>> {0} must be \
                                  linked to a mjc-body", m_BodyRef->name() );
                LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::SetLinearVelocity >>> {0} is a non-static \
                                  single-body, so it requires to have an associated mjc-joint", m_BodyRef->name() );

                m_mjcDataRef->qvel[m_mjcJointQvelAdr + 0] = linear_vel.x();
                m_mjcDataRef->qvel[m_mjcJointQvelAdr + 1] = linear_vel.y();
                m_mjcDataRef->qvel[m_mjcJointQvelAdr + 2] = linear_vel.z();
            }
        }
    }

    void TMujocoSingleBodyAdapter::SetAngularVelocity( const TVec3& angular_vel )
    {
        // @todo: change to check of constraint-type when supporting single-body constraints, for
        //        now, it just acts as the velocity of a free-body, with nv=6 <> [3xyz,3rpy]
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            if ( !m_BodyRef->constraint() )
            {
                LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetAngularVelocity >>> {0} must be \
                                  linked to a mjc-body", m_BodyRef->name() );
                LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::SetAngularVelocity >>> {0} is a non-static \
                                  single-body, so it requires to have an associated mjc-joint", m_BodyRef->name() );

                m_mjcDataRef->qvel[m_mjcJointQvelAdr + 3] = angular_vel.x();
                m_mjcDataRef->qvel[m_mjcJointQvelAdr + 4] = angular_vel.y();
                m_mjcDataRef->qvel[m_mjcJointQvelAdr + 5] = angular_vel.z();
            }
        }
    }

    void TMujocoSingleBodyAdapter::SetForceCOM( const TVec3& force_com )
    {
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetForceCOM >>> {0} must be \
                              linked to a mjc-body", m_BodyRef->name() );

            m_mjcDataRef->xfrc_applied[6 * m_mjcBodyId + 0] = force_com.x();
            m_mjcDataRef->xfrc_applied[6 * m_mjcBodyId + 1] = force_com.y();
            m_mjcDataRef->xfrc_applied[6 * m_mjcBodyId + 2] = force_com.z();
        }
    }

    void TMujocoSingleBodyAdapter::SetTorqueCOM( const TVec3& torque_com )
    {
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetTorqueCOM >>> {0} must be \
                              linked to a mjc-body", m_BodyRef->name() );

            m_mjcDataRef->xfrc_applied[6 * m_mjcBodyId + 3] = torque_com.x();
            m_mjcDataRef->xfrc_applied[6 * m_mjcBodyId + 4] = torque_com.y();
            m_mjcDataRef->xfrc_applied[6 * m_mjcBodyId + 5] = torque_com.z();
        }
    }

    void TMujocoSingleBodyAdapter::GetTransform( TMat4& dst_transform ) /* const */
    {
        const bool is_static_mesh = ( m_BodyRef->dyntype() == eDynamicsType::STATIC && 
                                      m_BodyRef->collider()->shape() == eShapeType::MESH );
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC || is_static_mesh )
        {
            LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::GetTransform >>> {0} must be \
                              linked to a mjc-body", m_BodyRef->name() );
            const TVec3 position = { (TScalar) m_mjcDataRef->xpos[3 * m_mjcBodyId + 0],
                                     (TScalar) m_mjcDataRef->xpos[3 * m_mjcBodyId + 1],
                                     (TScalar) m_mjcDataRef->xpos[3 * m_mjcBodyId + 2] };
            const TVec4 quaternion = { (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 1],
                                       (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 2],
                                       (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 3],
                                       (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 0] };
            dst_transform.set( position, 3 );
            dst_transform.set( tinymath::rotation( quaternion ) );
        }
        else
        {
            const TVec3 position = { (TScalar) m_mjcDataRef->geom_xpos[3 * m_mjcGeomId + 0],
                                     (TScalar) m_mjcDataRef->geom_xpos[3 * m_mjcGeomId + 1],
                                     (TScalar) m_mjcDataRef->geom_xpos[3 * m_mjcGeomId + 2] };
            const mjtNum mjc_rotmatrix[9] = { m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 0],
                                              m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 1],
                                              m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 2],
                                              m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 3],
                                              m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 4],
                                              m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 5],
                                              m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 6],
                                              m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 7],
                                              m_mjcDataRef->geom_xmat[9 * m_mjcGeomId + 8], };
            mjtNum mjc_quaternion[4];
            mju_mat2Quat( mjc_quaternion, mjc_rotmatrix );
            const TVec4 quaternion = { (TScalar) mjc_quaternion[1],
                                       (TScalar) mjc_quaternion[2],
                                       (TScalar) mjc_quaternion[3],
                                       (TScalar) mjc_quaternion[0] };
            dst_transform.set( position, 3 );
            dst_transform.set( tinymath::rotation( quaternion ) );
        }
    }

    void TMujocoSingleBodyAdapter::GetLinearVelocity( TVec3& dst_linear_vel ) /* const */
    {
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::GetLinearVelocity >>> {0} must be \
                              linked to a mjc-body", m_BodyRef->name() );

            dst_linear_vel.x() = m_mjcDataRef->cvel[6 * m_mjcBodyId + 3];
            dst_linear_vel.y() = m_mjcDataRef->cvel[6 * m_mjcBodyId + 4];
            dst_linear_vel.z() = m_mjcDataRef->cvel[6 * m_mjcBodyId + 5];
        }
    }

    void TMujocoSingleBodyAdapter::GetAngularVelocity( TVec3& dst_angular_vel ) /* const */
    {
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::GetAngularVelocity >>> {0} must be \
                              linked to a mjc-body", m_BodyRef->name() );

            dst_angular_vel.x() = m_mjcDataRef->cvel[6 * m_mjcBodyId + 0];
            dst_angular_vel.y() = m_mjcDataRef->cvel[6 * m_mjcBodyId + 1];
            dst_angular_vel.z() = m_mjcDataRef->cvel[6 * m_mjcBodyId + 2];
        }
    }

    void TMujocoSingleBodyAdapter::SetMjcModel( mjModel* mjModelRef )
    {
        m_mjcModelRef = mjModelRef;
        if ( auto mjc_collider_adapter = dynamic_cast<TMujocoSingleBodyColliderAdapter*>( m_ColliderAdapter.get() ) )
            mjc_collider_adapter->SetMjcModel( m_mjcModelRef );

        if ( auto mjc_constraint_adapter = dynamic_cast<TIMujocoSingleBodyConstraintAdapter*>( m_ConstraintAdapter.get() ) )
            mjc_constraint_adapter->SetMjcModel( m_mjcModelRef );
    }

    void TMujocoSingleBodyAdapter::SetMjcData( mjData* mjDataRef )
    {
        m_mjcDataRef = mjDataRef;
        if ( auto mjc_collider_adapter = dynamic_cast<TMujocoSingleBodyColliderAdapter*>( m_ColliderAdapter.get() ) )
            mjc_collider_adapter->SetMjcData( m_mjcDataRef );

        if ( auto mjc_constraint_adapter = dynamic_cast<TIMujocoSingleBodyConstraintAdapter*>( m_ConstraintAdapter.get() ) )
            mjc_constraint_adapter->SetMjcData( m_mjcDataRef );
    }

    void TMujocoSingleBodyAdapter::HideMjcObject()
    {
        const auto position = TVec3( m_DetachedRestTransform.col( 3 ) );
        const auto quaternion = tinymath::quaternion( m_DetachedRestTransform );

        LOCO_CORE_ASSERT( m_mjcModelRef, "TMujocoSingleBodyAdapter::HideMjcObject >>> recycled adapter must have \
                          a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_mjcDataRef, "TMujocoSingleBodyAdapter::HideMjcObject >>> recycled adapter must have \
                          a valid mjData reference" );
        if ( m_mjcJointId != -1 ) // Is a dynamic body
        {
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 0] = position.x();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 1] = position.y();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 2] = position.z();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 3] = quaternion.w();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 4] = quaternion.x();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 5] = quaternion.y();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 6] = quaternion.z();
        }
        else if ( m_mjcBodyId != -1 )
        {
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = position.x();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = position.y();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = position.z();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = quaternion.w();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = quaternion.x();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = quaternion.y();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = quaternion.z();
        }
        else if ( m_mjcGeomId != -1 ) // If not, then it's a static body
        {
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 0] = position.x();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 1] = position.y();
            m_mjcModelRef->geom_pos[3 * m_mjcGeomId + 2] = position.z();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 0] = quaternion.w();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 1] = quaternion.x();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 2] = quaternion.y();
            m_mjcModelRef->geom_quat[4 * m_mjcGeomId + 3] = quaternion.z();
        }
    }
}}