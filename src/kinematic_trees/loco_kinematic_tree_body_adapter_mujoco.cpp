
#include <kinematic_trees/loco_kinematic_tree_body_adapter_mujoco.h>

namespace loco {
namespace kintree {

    TMujocoKinematicTreeBodyAdapter::TMujocoKinematicTreeBodyAdapter( TKinematicTreeBody* body_ref )
        : TIKinematicTreeBodyAdapter( body_ref ) {}

    TMujocoKinematicTreeBodyAdapter::~TMujocoKinematicTreeBodyAdapter()
    {
        m_MjcBodyId = -1;
        m_MjcModelRef = nullptr;
        m_MjcDataRef = nullptr;
        m_MjcfElementResources = nullptr;
        m_MjcfElementAssetsResources = nullptr;
    }

    void TMujocoKinematicTreeBodyAdapter::Build()
    {
        m_MjcfElementResources = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_BODY_TAG, parsing::eSchemaType::MJCF );
        m_MjcfElementResources->SetString( "name", m_BodyRef->name() );
        // Set local transform of this body w.r.t. to either its parent body or the kintree (if root)
        TMat4 local_tf;
        if ( m_BodyRef->parent() ) // non-root body (use the local-transform to its parent)
            local_tf = m_BodyRef->local_tf();
        else if ( m_BodyRef->kintree() ) // root-body (use the world-transform of the kintree)
            local_tf = m_BodyRef->kintree()->tf0();
        else
            LOCO_CORE_ERROR( "TMujocoKinematicTreeBodyAdapter::Build >>> Root-body {0} should be linked to a valid kintree", m_BodyRef->name() );
        const auto local_pos = TVec3( local_tf.col( 3 ) );
        const auto local_quat = tinymath::quaternion( local_tf );
        m_MjcfElementResources->SetVec3( "pos", local_pos );
        m_MjcfElementResources->SetVec4( "quat", mujoco::quat_to_mjcQuat( local_quat ) );
        // -----------------------------------------------------------------------------------------
        const auto& inertia = m_BodyRef->data().inertia;
        if ( ( inertia.mass > loco::EPS ) && 
             ( inertia.ixx > loco::EPS ) && ( inertia.iyy > loco::EPS ) && ( inertia.izz > loco::EPS ) && 
             ( inertia.ixy > -loco::EPS ) && ( inertia.ixz > -loco::EPS ) && ( inertia.iyz > -loco::EPS ) )
        {
            const auto iframe_local_pos = TVec3( inertia.localTransform.col( 3 ) );
            const auto iframe_local_quat = mujoco::quat_to_mjcQuat( tinymath::quaternion( inertia.localTransform ) );
            auto inertial_elm = m_MjcfElementResources->Add( "inertial" );
            inertial_elm->SetVec3( "pos", iframe_local_pos );
            inertial_elm->SetVec4( "quat", iframe_local_quat );
            inertial_elm->SetFloat( "mass", inertia.mass );
            inertial_elm->SetArrayFloat( "fullinertia", { inertia.ixx, inertia.iyy, inertia.izz,
                                                          inertia.ixy, inertia.ixz, inertia.iyz } );
        }

        if ( auto collider = m_BodyRef->collider() )
        {
            m_ColliderAdapter = std::make_unique<TMujocoKinematicTreeColliderAdapter>( collider );
            collider->SetColliderAdapter( m_ColliderAdapter.get() );

            auto mjc_collider_adapter = static_cast<TMujocoKinematicTreeColliderAdapter*>( m_ColliderAdapter.get() );
            mjc_collider_adapter->Build();

            auto mjcf_xml_geoms = mjc_collider_adapter->elements_resources();
            LOCO_CORE_ASSERT( mjcf_xml_geoms.size() > 0, "TMujocoKinematicTreeBodyAdapter::Build >>> collider "
                              "must have mjcf-geom-resources (at least 1) once built, for body named {0}", m_BodyRef->name() );
            for ( auto mjcf_geom : mjcf_xml_geoms )
                m_MjcfElementResources->Add( parsing::TElement::CloneElement( mjcf_geom ) );

            if ( auto collider_element_assets_resources = mjc_collider_adapter->element_assets_resources() )
            {
                if ( !m_MjcfElementAssetsResources )
                    m_MjcfElementAssetsResources = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_ASSET_TAG, parsing::eSchemaType::MJCF );
                m_MjcfElementAssetsResources->Add( parsing::TElement::CloneElement( collider_element_assets_resources ) );
            }
        }

        if ( auto joint = m_BodyRef->joint() )
        {
            m_JointAdapter = std::make_unique<TMujocoKinematicTreeJointAdapter>( joint );
            joint->SetJointAdapter( m_JointAdapter.get() );

            auto mjc_collider_adapter = static_cast<TMujocoKinematicTreeJointAdapter*>( m_JointAdapter.get() );
            mjc_collider_adapter->Build();

            auto vec_jnt_elements = static_cast<TMujocoKinematicTreeJointAdapter*>( mjc_collider_adapter )->elements_resources();
            for ( auto joint_element_resource : vec_jnt_elements )
                m_MjcfElementResources->Add( parsing::TElement::CloneElement( joint_element_resource ) );
        }
    }

    void TMujocoKinematicTreeBodyAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeBodyAdapter::Initialize >>> must have a valid mjModel reference (got nullptr)" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeBodyAdapter::Initialize >>> must have a valid mjData reference (got nullptr)" );

        m_MjcBodyId = mj_name2id( m_MjcModelRef, mjOBJ_BODY, m_BodyRef->name().c_str() );
        if ( m_MjcBodyId < 0 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeBodyAdapter::Initialize >>> couldn't find associated "
                             "mjc-body for body {0} (returned mjc-body-id) < 0", m_BodyRef->name() );
            return;
        }
    }

    void TMujocoKinematicTreeBodyAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeBodyAdapter::Reset >>> must have a valid mjModel reference (got nullptr)" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeBodyAdapter::Reset >>> must have a valid mjData reference (got nullptr)" );

        // Nothing to reset, the collider|joint' adapters are handled by its related objects
    }

    void TMujocoKinematicTreeBodyAdapter::SetForceCOM( const TVec3& force )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeBodyAdapter::SetForceCOM >>> must have a valid mjModel reference (got nullptr)" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeBodyAdapter::SetForceCOM >>> must have a valid mjData reference (got nullptr)" );

        if ( m_MjcBodyId < 0 )
            return;

        m_MjcDataRef->xfrc_applied[6 * m_MjcBodyId + 0] = force.x();
        m_MjcDataRef->xfrc_applied[6 * m_MjcBodyId + 1] = force.y();
        m_MjcDataRef->xfrc_applied[6 * m_MjcBodyId + 2] = force.z();
    }

    void TMujocoKinematicTreeBodyAdapter::SetTorqueCOM( const TVec3& torque )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeBodyAdapter::SetTorqueCOM >>> must have a valid mjModel reference (got nullptr)" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeBodyAdapter::SetTorqueCOM >>> must have a valid mjData reference (got nullptr)" );

        if ( m_MjcBodyId < 0 )
            return;

        m_MjcDataRef->xfrc_applied[6 * m_MjcBodyId + 3] = torque.x();
        m_MjcDataRef->xfrc_applied[6 * m_MjcBodyId + 4] = torque.y();
        m_MjcDataRef->xfrc_applied[6 * m_MjcBodyId + 5] = torque.z();
    }

    void TMujocoKinematicTreeBodyAdapter::GetTransform( TMat4& dst_transform )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeBodyAdapter::GetTransform >>> must have a valid mjModel reference (got nullptr)" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeBodyAdapter::GetTransform >>> must have a valid mjData reference (got nullptr)" );

        if ( m_MjcBodyId < 0 )
            return;

        const TVec3 world_pos = { (TScalar) m_MjcDataRef->xpos[3 * m_MjcBodyId + 0],
                                  (TScalar) m_MjcDataRef->xpos[3 * m_MjcBodyId + 1],
                                  (TScalar) m_MjcDataRef->xpos[3 * m_MjcBodyId + 2] };
        const TVec4 world_quat = { (TScalar) m_MjcDataRef->xquat[4 * m_MjcBodyId + 1],
                                   (TScalar) m_MjcDataRef->xquat[4 * m_MjcBodyId + 2],
                                   (TScalar) m_MjcDataRef->xquat[4 * m_MjcBodyId + 3],
                                   (TScalar) m_MjcDataRef->xquat[4 * m_MjcBodyId + 0] };
        dst_transform.set( world_pos, 3 );
        dst_transform.set( tinymath::rotation( world_quat ) );
    }

    void TMujocoKinematicTreeBodyAdapter::SetMjcModel( mjModel* mj_model_ref )
    {
        m_MjcModelRef = mj_model_ref;
        if ( auto mjc_collider_adapter = dynamic_cast<TMujocoKinematicTreeColliderAdapter*>( m_ColliderAdapter.get() ) )
            mjc_collider_adapter->SetMjcModel( mj_model_ref );

        if ( auto mjc_joint_adapter = dynamic_cast<TMujocoKinematicTreeJointAdapter*>( m_JointAdapter.get() ) )
            mjc_joint_adapter->SetMjcModel( mj_model_ref );
    }

    void TMujocoKinematicTreeBodyAdapter::SetMjcData( mjData* mj_data_ref )
    {
        m_MjcDataRef = mj_data_ref;
        if ( auto mjc_collider_adapter = dynamic_cast<TMujocoKinematicTreeColliderAdapter*>( m_ColliderAdapter.get() ) )
            mjc_collider_adapter->SetMjcData( mj_data_ref );

        if ( auto mjc_joint_adapter = dynamic_cast<TMujocoKinematicTreeJointAdapter*>( m_JointAdapter.get() ) )
            mjc_joint_adapter->SetMjcData( mj_data_ref );
    }
}}