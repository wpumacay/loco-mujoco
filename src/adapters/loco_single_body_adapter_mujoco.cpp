
#include <adapters/loco_single_body_adapter_mujoco.h>

namespace loco {
namespace mujoco {

    TMujocoSingleBodyAdapter::TMujocoSingleBodyAdapter( TIBody* bodyRef )
        : TIBodyAdapter( bodyRef )
    {
        LOCO_CORE_ASSERT( bodyRef, "TMujocoSingleBodyAdapter >>> adaptee (body) should be a valid \
                          refernce (nullptr given)" );
        LOCO_CORE_ASSERT( bodyRef->classType() == loco::eBodyClassType::SINGLE_BODY,
                          "TMujocoSingleBodyAdapter >>> body {0} is not of class-type single-body", bodyRef->name() );
        LOCO_CORE_ASSERT( bodyRef->collision(), "TMujocoSingleBodyAdapter >>> body {0} doesn't have \
                          a valid collider (found nullptr)", bodyRef->name() );

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
        const std::string name = ( m_bodyRef ) ? m_bodyRef->name() : "undefined";
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TMujocoSingleBodyAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TMujocoSingleBodyAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TMujocoSingleBodyAdapter::~TMujocoSingleBodyAdapter()
    {
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
        const std::string name = ( m_bodyRef ) ? m_bodyRef->name() : "undefined";
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TMujocoSingleBodyAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TMujocoSingleBodyAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TMujocoSingleBodyAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_bodyRef, "TMujocoSingleBodyAdapter::Build >>> must have a valid body-object (got nullptr instead)" );

        m_mjcfElementResources = std::make_unique<parsing::TElement>( LOCO_MJCF_BODY_TAG, parsing::eSchemaType::MJCF );
        m_mjcfElementResources->SetString( "name", m_bodyRef->name() );
        m_mjcfElementResources->SetVec3( "pos", m_bodyRef->pos() );
        m_mjcfElementResources->SetVec4( "quat", quat_to_mjcQuat( m_bodyRef->quat() ) );

        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            auto joint_element = m_mjcfElementResources->Add( "freejoint" );
            joint_element->SetString( "name", m_bodyRef->name() + "_freejnt" );
        }

        // Inertial mjcf-element is added only if all inertia properties are given (mass + inertia matrix).
        // If only the mass is given, then the density is computed for the collider from the mass and volume.
        const auto& inertia = m_bodyRef->data().inertia;
        if ( ( inertia.mass > loco::EPS ) && ( inertia.ixx > loco::EPS ) && ( inertia.iyy > loco::EPS ) && 
             ( inertia.izz > loco::EPS ) && ( inertia.ixy > -loco::EPS ) && ( inertia.ixz > -loco::EPS ) &&
             ( inertia.iyz > -loco::EPS ) )
        {
            auto _inertia_element = m_mjcfElementResources->Add( "inertial" );
            _inertia_element->SetFloat( "mass", inertia.mass );
            _inertia_element->SetArrayFloat( "fullinertia", { inertia.ixx, inertia.iyy, inertia.izz,
                                                              inertia.ixy, inertia.ixz, inertia.iyz } );
        }

        auto collision = m_bodyRef->collision();
        LOCO_CORE_ASSERT( collision, "TMujocoSingleBodyAdapter::Build >>> single-body {0} doesn't have \
                          a valid collider (nullptr)", m_bodyRef->name() );

        if ( auto collision_adapter = dynamic_cast<TMujocoCollisionAdapter*>( collision->adapter() ) )
        {
            collision_adapter->Build();
            if ( auto collider_element_resources = collision_adapter->element_resources() )
                m_mjcfElementResources->Add( parsing::TElement::CloneElement( collider_element_resources ) );
            if ( auto collider_element_asset_resources = collision_adapter->element_asset_resources() )
            {
                m_mjcfElementAssetResources = std::make_unique<parsing::TElement>( LOCO_MJCF_ASSET_TAG, parsing::eSchemaType::MJCF );
                m_mjcfElementAssetResources->Add( parsing::TElement::CloneElement( collider_element_asset_resources ) );
            }
        }
    }

    void TMujocoSingleBodyAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_mjcModelRef, "TMujocoSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid mjModel reference", m_bodyRef->name() );
        LOCO_CORE_ASSERT( m_mjcDataRef, "TMujocoSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid mjData reference", m_bodyRef->name() );

        m_mjcBodyId = mj_name2id( m_mjcModelRef, mjOBJ_BODY, m_bodyRef->name().c_str() );
        if ( m_mjcBodyId < 0 )
        {
            LOCO_CORE_ERROR( "TMujocoSingleBodyAdapter::Initialize >>> couldn't find associated \
                              mjc-body for single-body {0}", m_bodyRef->name() );
            return;
        }

        if ( auto collision = m_bodyRef->collision() )
        {
            if ( auto collision_adapter = dynamic_cast<TMujocoCollisionAdapter*>( collision->adapter() ) )
            {
                collision_adapter->SetMjcModel( m_mjcModelRef );
                collision_adapter->SetMjcData( m_mjcDataRef );
                collision_adapter->Initialize();
            }
        }

        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            m_mjcJointId = m_mjcModelRef->body_jntadr[m_mjcBodyId];
            if ( m_mjcJointId < 0 )
            {
                LOCO_CORE_ERROR( "TMujocoSingleBodyAdapter::Initialize >>> couldn't find associated \
                                  mjc-joint (free-joint) for non-static single-body {0}", m_bodyRef->name() );
            }
            else
            {
                const auto jnt_type = m_mjcModelRef->jnt_type[m_mjcJointId];
                if ( jnt_type != mjJNT_FREE )
                {
                    LOCO_CORE_ERROR( "TMujocoSingleBodyAdapter::Initialize >>> joint associated to single-body \
                                      {0} must be a free-joint", m_bodyRef->name() );
                }
                else
                {
                    m_mjcJointQposAdr = m_mjcModelRef->jnt_qposadr[m_mjcJointId];
                    m_mjcJointQvelAdr = m_mjcModelRef->jnt_dofadr[m_mjcJointId];
                    m_mjcJointQposNum = 7;
                    m_mjcJointQvelNum = 6;

                    // @todo: is this required? (qpos0 should be computed from pos|quat in xml, which is our desired qpos0)
                    // Set the body's initial configuration
                    const TVec3 position0 = m_bodyRef->pos0();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 0] = position0.x();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 1] = position0.y();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 2] = position0.z();
                    const TVec4 quaternion0 = m_bodyRef->quat0();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 3] = quaternion0.w();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 4] = quaternion0.x();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 5] = quaternion0.y();
                    m_mjcModelRef->qpos0[m_mjcJointQposAdr + 6] = quaternion0.z();
                }
            }
        }
    }

    void TMujocoSingleBodyAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_mjcModelRef, "TMujocoSingleBodyAdapter::Reset >>> {0} must have a valid mjModel reference", m_bodyRef->name() );
        LOCO_CORE_ASSERT( m_mjcDataRef, "TMujocoSingleBodyAdapter::Reset >>> {0} must have a valid mjData reference", m_bodyRef->name() );
        LOCO_CORE_ASSERT( m_mjcBodyId, "TMujocoSingleBodyAdapter::Reset >>> {0} must be linked to a valid mjc-body", m_bodyRef->name() );

        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            for ( ssize_t i = 0; i < m_mjcJointQposNum; i++ )
                m_mjcDataRef->qpos[m_mjcJointQposAdr + i] = m_mjcModelRef->qpos0[m_mjcJointQposAdr + i];
            for ( ssize_t i = 0; i < m_mjcJointQvelNum; i++ )
                m_mjcDataRef->qvel[m_mjcJointQvelAdr + i] = 0.0;
        }
        else
        {
            const TVec3 position0 = m_bodyRef->pos0();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = position0.x();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = position0.y();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = position0.z();
            const TVec4 quaternion0 = m_bodyRef->quat0();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = quaternion0.w();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = quaternion0.x();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = quaternion0.y();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = quaternion0.z();
        }
    }

    void TMujocoSingleBodyAdapter::PreStep()
    {
        // Nothing to prepare before to a simulation step
    }

    void TMujocoSingleBodyAdapter::PostStep()
    {
        // Nothing to process after to a simulation step
    }

    void TMujocoSingleBodyAdapter::SetPosition( const TVec3& position )
    {
        LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetPosition >>> {0} must be \
                          linked to a mjc-body", m_bodyRef->name() );

        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::SetRotation >>> {0} is a non-static \
                              single-body, so it requires to have an associated mjc-joint", m_bodyRef->name() );
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 0] = position.x();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 1] = position.y();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 2] = position.z();
        }
        else
        {
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = position.x();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = position.y();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = position.z();
        }
    }

    void TMujocoSingleBodyAdapter::SetRotation( const TMat3& rotation )
    {
        LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetRotation >>> {0} must be \
                          linked to a mjc-body", m_bodyRef->name() );

        const TVec4 quaternion = tinymath::quaternion( rotation );
        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::SetRotation >>> {0} is a non-static \
                              single-body, so it requires to have an associated mjc-joint", m_bodyRef->name() );
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 3] = quaternion.w();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 4] = quaternion.x();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 5] = quaternion.y();
            m_mjcDataRef->qpos[m_mjcJointQposAdr + 6] = quaternion.z();
        }
        else
        {
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = quaternion.w();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = quaternion.x();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = quaternion.y();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = quaternion.z();
        }
    }

    void TMujocoSingleBodyAdapter::SetTransform( const TMat4& transform )
    {
        const TVec3 position = TVec3( transform.col( 3 ) );
        const TMat3 rotation = TMat3( transform );
        SetPosition( position );
        SetRotation( rotation );
    }

    void TMujocoSingleBodyAdapter::GetPosition( TVec3& dst_position )
    {
        LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::GetPosition >>> {0} must be \
                          linked to a mjc-body", m_bodyRef->name() );

        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::GetPosition >>> {0} is a non-static \
                              single-body, so it requires to have an associated mjc-joint", m_bodyRef->name() );
            dst_position.x() = m_mjcDataRef->qpos[m_mjcJointQposAdr + 0];
            dst_position.y() = m_mjcDataRef->qpos[m_mjcJointQposAdr + 1];
            dst_position.z() = m_mjcDataRef->qpos[m_mjcJointQposAdr + 2];
        }
        else
        {
            dst_position.x() = m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0];
            dst_position.y() = m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1];
            dst_position.z() = m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2];
        }
    }

    void TMujocoSingleBodyAdapter::GetRotation( TMat3& rotation )
    {
        LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::GetRotation >>> {0} must be \
                          linked to a mjc-body", m_bodyRef->name() );

        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::GetRotation >>> {0} is a non-static \
                              single-body, so it requires to have an associated mjc-joint", m_bodyRef->name() );
            const TVec4 quaternion = { (TScalar) m_mjcDataRef->qpos[m_mjcJointQposAdr + 4],
                                       (TScalar) m_mjcDataRef->qpos[m_mjcJointQposAdr + 5],
                                       (TScalar) m_mjcDataRef->qpos[m_mjcJointQposAdr + 6],
                                       (TScalar) m_mjcDataRef->qpos[m_mjcJointQposAdr + 3] };
            rotation = tinymath::rotation( quaternion );
        }
        else
        {
            const TVec4 quaternion = { (TScalar) m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1],
                                       (TScalar) m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2],
                                       (TScalar) m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3],
                                       (TScalar) m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] };
            rotation = tinymath::rotation( quaternion );
        }
    }

    void TMujocoSingleBodyAdapter::GetTransform( TMat4& transform )
    {
        TVec3 position;
        TMat3 rotation;
        GetPosition( position );
        GetRotation( rotation );

        transform.set( position, 3 );
        transform.set( rotation );
    }

    void TMujocoSingleBodyAdapter::SetInitialPosition( const TVec3& position )
    {
        LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetPosition >>> {0} must be \
                          linked to a mjc-body", m_bodyRef->name() );

        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::SetRotation >>> {0} is a non-static \
                              single-body, so it requires to have an associated mjc-joint", m_bodyRef->name() );
            m_mjcModelRef->qpos0[m_mjcJointQposAdr + 0] = position.x();
            m_mjcModelRef->qpos0[m_mjcJointQposAdr + 1] = position.y();
            m_mjcModelRef->qpos0[m_mjcJointQposAdr + 2] = position.z();
        }
        else
        {
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = position.x();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = position.y();
            m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = position.z();
        }
    }

    void TMujocoSingleBodyAdapter::SetInitialRotation( const TMat3& rotation )
    {
        LOCO_CORE_ASSERT( m_mjcBodyId >= 0, "TMujocoSingleBodyAdapter::SetRotation >>> {0} must be \
                          linked to a mjc-body", m_bodyRef->name() );

        const TVec4 quaternion = tinymath::quaternion( rotation );
        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            LOCO_CORE_ASSERT( m_mjcJointId >= 0, "TMujocoSingleBodyAdapter::SetRotation >>> {0} is a non-static \
                              single-body, so it requires to have an associated mjc-joint", m_bodyRef->name() );
            m_mjcModelRef->qpos0[m_mjcJointQposAdr + 3] = quaternion.w();
            m_mjcModelRef->qpos0[m_mjcJointQposAdr + 4] = quaternion.x();
            m_mjcModelRef->qpos0[m_mjcJointQposAdr + 5] = quaternion.y();
            m_mjcModelRef->qpos0[m_mjcJointQposAdr + 6] = quaternion.z();
        }
        else
        {
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = quaternion.w();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = quaternion.x();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = quaternion.y();
            m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = quaternion.z();
        }
    }

    void TMujocoSingleBodyAdapter::SetInitialTransform( const TMat4& transform )
    {
        const TVec3 position = TVec3( transform.col( 3 ) );
        const TMat3 rotation = TMat3( transform );
        SetInitialPosition( position );
        SetInitialRotation( rotation );
    }

    void TMujocoSingleBodyAdapter::SetLocalPosition( const TVec3& position )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TMujocoSingleBodyAdapter::SetLocalRotation( const TMat3& rotation )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TMujocoSingleBodyAdapter::SetLocalTransform( const TMat4& transform )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TMujocoSingleBodyAdapter::GetLocalPosition( TVec3& dstPosition )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TMujocoSingleBodyAdapter::GetLocalRotation( TMat3& dstRotation )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TMujocoSingleBodyAdapter::GetLocalTransform( TMat4& dstTransform )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TMujocoSingleBodyAdapter::SetInitialLocalPosition( const TVec3& position )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TMujocoSingleBodyAdapter::SetInitialLocalRotation( const TMat3& rotation )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TMujocoSingleBodyAdapter::SetInitialLocalTransform( const TMat4& transform )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }
}}