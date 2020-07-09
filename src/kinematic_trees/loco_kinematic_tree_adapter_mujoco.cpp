
#include <kinematic_trees/loco_kinematic_tree_adapter_mujoco.h>

namespace loco {
namespace kintree {

    TMujocoKinematicTreeAdapter::TMujocoKinematicTreeAdapter( TKinematicTree* kintree_ref )
        : TIKinematicTreeAdapter( kintree_ref ) {}

    TMujocoKinematicTreeAdapter::~TMujocoKinematicTreeAdapter()
    {
        m_MjcModelRef = nullptr;
        m_MjcDataRef = nullptr;
        m_MjcRootBodyId = -1;
        m_MjcfElementResources = nullptr;
        m_MjcfElementAssetsResources = nullptr;
    }

    void TMujocoKinematicTreeAdapter::Build()
    {
        m_MjcfElementResources = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_WORLDBODY_TAG, parsing::eSchemaType::MJCF );
        auto root_body = m_KintreeRef->root();
        std::stack<std::pair<TKinematicTreeBody*, parsing::TElement*>> dfs_body_parentElm;
        dfs_body_parentElm.push( { root_body, m_MjcfElementResources.get() } );
        while ( !dfs_body_parentElm.empty() )
        {
            auto body_parentElm_pair = dfs_body_parentElm.top();
            dfs_body_parentElm.pop();
            auto curr_body = body_parentElm_pair.first;
            auto curr_parent_elm = body_parentElm_pair.second;
            if ( !curr_body || !curr_parent_elm )
            {
                LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::Build >>> found nullptr, either body or "
                                 "parsing-resource. Error found while processing kintree {0}", m_KintreeRef->name() );
                continue;
            }

            auto mjc_body_adapter = std::make_unique<TMujocoKinematicTreeBodyAdapter>( curr_body );
            curr_body->SetBodyAdapter( mjc_body_adapter.get() );
            m_BodyAdapters.push_back( std::move( mjc_body_adapter ) );
            m_BodyAdapters.back()->Build();

            if ( auto body_element_resources = static_cast<TMujocoKinematicTreeBodyAdapter*>(
                                                m_BodyAdapters.back().get() )->element_resources() )
            {
                curr_parent_elm->Add( parsing::TElement::CloneElement( body_element_resources ) );
            }
            if ( auto body_element_assets_resources = static_cast<TMujocoKinematicTreeBodyAdapter*>(
                                                        m_BodyAdapters.back().get() )->element_assets_resources() )
            {
                if ( !m_MjcfElementAssetsResources )
                    m_MjcfElementAssetsResources = std::make_unique<parsing::TElement>( 
                                                            mujoco::LOCO_MJCF_ASSET_TAG, parsing::eSchemaType::MJCF );
                for ( ssize_t i = 0; i < body_element_assets_resources->num_children(); i++ )
                    m_MjcfElementAssetsResources->Add( parsing::TElement::CloneElement( 
                                                            body_element_assets_resources->get_child( i ) ) );
            }

            auto children = curr_body->children();
            for ( auto child : children )
                dfs_body_parentElm.push( { child, curr_parent_elm->get_child( curr_parent_elm->num_children() - 1 ) } );
        }
    }

    void TMujocoKinematicTreeAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeAdapter::Initialize >>> must have a valid mjModel "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeAdapter::Initialize >>> must have a valid mjData "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        auto root_body = m_KintreeRef->root();
        if ( !root_body )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::Initialize >>> kintree {0} doesn't have a root-body", m_KintreeRef->name() );
            return;
        }
        m_MjcRootBodyId = mj_name2id( m_MjcModelRef, mjOBJ_BODY, root_body->name().c_str() );
    }

    void TMujocoKinematicTreeAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeAdapter::Reset >>> must have a valid mjModel "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeAdapter::Reset >>> must have a valid mjData "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        // Grab the root joint and reset accordingly to its type
        auto root_body = m_KintreeRef->root();
        if ( !root_body )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::Reset >>> kintree {0} doesn't have a root-body", m_KintreeRef->name() );
            return;
        }
        auto root_joint_candidates = root_body->joints();
        if ( root_joint_candidates.size() != 1 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::Reset >>> there must be only one root-joint "
                             "for the root-body. Found {0} joints at the root-body for kintree {1}",
                             root_joint_candidates.size(), m_KintreeRef->name() );
            return;
        }
        auto root_joint = root_joint_candidates.front();
        const eJointType root_joint_type = root_joint->type();
        /**/ if ( root_joint_type == eJointType::FREE )
        {
            /* @todo: check if tf0 != qpos0, as they should match */
            _SetTransformFreeJoint( root_joint, m_KintreeRef->tf0() );
            /* @todo: check if lin-ang-vel != qvel0, as they should match */
            _SetLinearVelFreeJoint( root_joint, m_KintreeRef->linear_vel0() );
            _SetAngularVelFreeJoint( root_joint, m_KintreeRef->angular_vel0() );
        }
        else if ( root_joint_type == eJointType::PLANAR )
        {
            _SetTransformPlanarJoint( root_joint, m_KintreeRef->tf0() );
            _SetLinearVelPlanarJoint( root_joint, m_KintreeRef->linear_vel0() );
            _SetAngularVelPlanarJoint( root_joint, m_KintreeRef->angular_vel0() );
        }
        else if ( root_joint_type == eJointType::FIXED )
        {
            _SetTransformFixedJoint( root_body, m_KintreeRef->tf0() );
        }
        else if ( root_joint_type == eJointType::SPHERICAL )
        {
            const auto qpos0 = root_joint->qpos0();
            const auto qvel0 = root_joint->qvel0();
            root_joint->SetQpos( { qpos0[3], qpos0[0], qpos0[1], qpos0[2] } );
            root_joint->SetQvel( { qvel0[0], qvel0[1], qvel0[2] } );
        }
        else if ( root_joint_type == eJointType::REVOLUTE ||
                  root_joint_type == eJointType::PRISMATIC )
        {
            root_joint->SetQpos( root_joint->qpos0() );
            root_joint->SetQvel( root_joint->qvel0() );
        }
    }

    void TMujocoKinematicTreeAdapter::SetTransform( const TMat4& tf )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeAdapter::SetTransform >>> must have a valid mjModel "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeAdapter::SetTransform >>> must have a valid mjData "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        auto root_body = m_KintreeRef->root();
        if ( !root_body )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::SetTransform >>> kintree {0} doesn't have a root-body", m_KintreeRef->name() );
            return;
        }
        auto root_joint_candidates = root_body->joints();
        if ( root_joint_candidates.size() != 1 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::Reset >>> there must be only one root-joint "
                             "for the root-body. Found {0} joints at the root-body for kintree {1}",
                             root_joint_candidates.size(), m_KintreeRef->name() );
            return;
        }
        auto root_joint = root_joint_candidates.front();
        const eJointType root_joint_type = root_joint->type();
        /**/ if ( root_joint_type == eJointType::FREE )
            _SetTransformFreeJoint( root_joint, tf );
        else if ( root_joint_type == eJointType::PLANAR )
            _SetTransformPlanarJoint( root_joint, tf );
        else if ( root_joint_type == eJointType::FIXED )
            _SetTransformFixedJoint( root_body, tf );
    }

    void TMujocoKinematicTreeAdapter::SetLinearVelocity( const TVec3& linear_vel )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeAdapter::SetLinearVelocity >>> must have a valid mjModel "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeAdapter::SetLinearVelocity >>> must have a valid mjData "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        auto root_body = m_KintreeRef->root();
        if ( !root_body )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::SetLinearVelocity >>> kintree {0} doesn't have a root-body",
                             m_KintreeRef->name() );
            return;
        }
        auto root_joint_candidates = root_body->joints();
        if ( root_joint_candidates.size() != 1 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::Reset >>> there must be only one root-joint "
                             "for the root-body. Found {0} joints at the root-body for kintree {1}",
                             root_joint_candidates.size(), m_KintreeRef->name() );
            return;
        }
        auto root_joint = root_joint_candidates.front();
        const eJointType root_joint_type = root_joint->type();
        /**/ if ( root_joint_type == eJointType::FREE )
            _SetLinearVelFreeJoint( root_joint, linear_vel );
        else if ( root_joint_type == eJointType::PLANAR )
            _SetLinearVelPlanarJoint( root_joint, linear_vel );
    }

    void TMujocoKinematicTreeAdapter::SetAngularVelocity( const TVec3& angular_vel )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeAdapter::SetAngularVelocity >>> must have a valid mjModel "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeAdapter::SetAngularVelocity >>> must have a valid mjData "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        auto root_body = m_KintreeRef->root();
        if ( !root_body )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::SetAngularVelocity >>> kintree {0} doesn't have a root-body",
                             m_KintreeRef->name() );
            return;
        }
        auto root_joint_candidates = root_body->joints();
        if ( root_joint_candidates.size() != 1 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::Reset >>> there must be only one root-joint "
                             "for the root-body. Found {0} joints at the root-body for kintree {1}",
                             root_joint_candidates.size(), m_KintreeRef->name() );
            return;
        }
        auto root_joint = root_joint_candidates.front();
        const eJointType root_joint_type = root_joint->type();
        /**/ if ( root_joint_type == eJointType::FREE )
            _SetAngularVelFreeJoint( root_joint, angular_vel );
        else if ( root_joint_type == eJointType::PLANAR )
            _SetAngularVelPlanarJoint( root_joint, angular_vel );
    }

    void TMujocoKinematicTreeAdapter::GetTransform( TMat4& dst_transform )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeAdapter::GetTransform >>> must have a valid mjModel "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeAdapter::GetTransform >>> must have a valid mjData "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        if ( m_MjcRootBodyId < 0 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::GetTransform >>> kintree {0} doesn't have "
                             "a valid mjc-root-body-id", m_KintreeRef->name() );
            return;
        }
        const TVec3 position( (TScalar) m_MjcDataRef->xpos[3 * m_MjcRootBodyId + 0],
                              (TScalar) m_MjcDataRef->xpos[3 * m_MjcRootBodyId + 1],
                              (TScalar) m_MjcDataRef->xpos[3 * m_MjcRootBodyId + 2] );
        const TVec4 quaternion( (TScalar) m_MjcDataRef->xquat[4 * m_MjcRootBodyId + 1],
                                (TScalar) m_MjcDataRef->xquat[4 * m_MjcRootBodyId + 2],
                                (TScalar) m_MjcDataRef->xquat[4 * m_MjcRootBodyId + 3],
                                (TScalar) m_MjcDataRef->xquat[4 * m_MjcRootBodyId + 0] );
        dst_transform.set( position, 3 );
        dst_transform.set( tinymath::rotation( quaternion ) );
    }

    void TMujocoKinematicTreeAdapter::GetLinearVelocity( TVec3& dst_linear_vel )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeAdapter::GetLinearVelocity >>> must have a valid mjModel "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeAdapter::GetLinearVelocity >>> must have a valid mjData "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        if ( m_MjcRootBodyId < 0 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::GetLinearVelocity >>> kintree {0} doesn't have "
                             "a valid mjc-root-body-id", m_KintreeRef->name() );
            return;
        }
        dst_linear_vel.x() = m_MjcDataRef->cvel[6 * m_MjcRootBodyId + 3];
        dst_linear_vel.y() = m_MjcDataRef->cvel[6 * m_MjcRootBodyId + 4];
        dst_linear_vel.z() = m_MjcDataRef->cvel[6 * m_MjcRootBodyId + 5];
    }

    void TMujocoKinematicTreeAdapter::GetAngularVelocity( TVec3& dst_angular_vel )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeAdapter::GetAngularVelocity >>> must have a valid mjModel "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeAdapter::GetAngularVelocity >>> must have a valid mjData "
                          "reference, but got nullptr. Error found while processing kintree {0}", m_KintreeRef->name() );
        if ( m_MjcRootBodyId < 0 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::GetAngularVelocity >>> kintree {0} doesn't have "
                             "a valid mjc-root-body-id", m_KintreeRef->name() );
            return;
        }
        dst_angular_vel.x() = m_MjcDataRef->cvel[6 * m_MjcRootBodyId + 0];
        dst_angular_vel.y() = m_MjcDataRef->cvel[6 * m_MjcRootBodyId + 1];
        dst_angular_vel.z() = m_MjcDataRef->cvel[6 * m_MjcRootBodyId + 2];
    }

    void TMujocoKinematicTreeAdapter::SetMjcModel( mjModel* mj_model_ref )
    {
        m_MjcModelRef = mj_model_ref;
        for ( auto& body_adapter : m_BodyAdapters )
            if ( auto mjc_body_adapter = dynamic_cast<TMujocoKinematicTreeBodyAdapter*>( body_adapter.get() ) )
                mjc_body_adapter->SetMjcModel( mj_model_ref );
    }

    void TMujocoKinematicTreeAdapter::SetMjcData( mjData* mj_data_ref )
    {
        m_MjcDataRef = mj_data_ref;
        for ( auto& body_adapter : m_BodyAdapters )
            if ( auto mjc_body_adapter = dynamic_cast<TMujocoKinematicTreeBodyAdapter*>( body_adapter.get() ) )
                mjc_body_adapter->SetMjcData( mj_data_ref );
    }

    void TMujocoKinematicTreeAdapter::_SetTransformFreeJoint( TKinematicTreeJoint* joint_ref, const TMat4& tf )
    {
        const auto world_pos = TVec3( tf.col( 3 ) );
        const auto world_quat = tinymath::quaternion( tf );
        joint_ref->SetQpos( { world_pos.x(), world_pos.y(), world_pos.z(), /* position x-y-z */
                              world_quat.w(), world_quat.x(), world_quat.y(), world_quat.z() /* rotation w-x-y-z */ } );
    }

    void TMujocoKinematicTreeAdapter::_SetLinearVelFreeJoint( TKinematicTreeJoint* joint_ref, const TVec3& linear_vel )
    {
        const ssize_t free_jnt_id = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, joint_ref->name().c_str() );
        const ssize_t free_jnt_qveladr = m_MjcModelRef->jnt_dofadr[free_jnt_id];
        /* @todo: set qvel as well, as there might be a mismatch of one update step between stored qvel and internal-backend qvels */
        m_MjcDataRef->qvel[free_jnt_qveladr + 0] = linear_vel.x();
        m_MjcDataRef->qvel[free_jnt_qveladr + 1] = linear_vel.y();
        m_MjcDataRef->qvel[free_jnt_qveladr + 2] = linear_vel.z();
    }

    void TMujocoKinematicTreeAdapter::_SetAngularVelFreeJoint( TKinematicTreeJoint* joint_ref, const TVec3& angular_vel )
    {
        const ssize_t free_jnt_id = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, joint_ref->name().c_str() );
        const ssize_t free_jnt_qveladr = m_MjcModelRef->jnt_dofadr[free_jnt_id];
        /* @todo: set qvel as well, as there might be a mismatch of one update step between stored qvel and internal-backend qvels */
        m_MjcDataRef->qvel[free_jnt_qveladr + 3] = angular_vel.x();
        m_MjcDataRef->qvel[free_jnt_qveladr + 4] = angular_vel.y();
        m_MjcDataRef->qvel[free_jnt_qveladr + 5] = angular_vel.z();
    }

    void TMujocoKinematicTreeAdapter::_SetTransformPlanarJoint( TKinematicTreeJoint* joint_ref, const TMat4& tf )
    {
        const ssize_t pj_trans_1_id = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, ( joint_ref->name() + "_trans_1" ).c_str() );
        const ssize_t pj_trans_2_id = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, ( joint_ref->name() + "_trans_2" ).c_str() );
        const ssize_t pj_rot_id = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, ( joint_ref->name() + "_rot" ).c_str() );

        const ssize_t pj_trans_1_qposadr = m_MjcModelRef->jnt_qposadr[pj_trans_1_id];
        const ssize_t pj_trans_2_qposadr = m_MjcModelRef->jnt_qposadr[pj_trans_2_id];
        const ssize_t pj_rot_qposadr = m_MjcModelRef->jnt_qposadr[pj_rot_id];

        const auto vec_trans_1 = joint_ref->data().plane_axis_1.normalized();
        const auto vec_trans_2 = joint_ref->data().plane_axis_2.normalized();
        const auto world_pos = TVec3( tf.col( 3 ) );
        const auto pos_axis_1 = vec_trans_1.dot( world_pos );
        const auto pos_axis_2 = vec_trans_2.dot( world_pos );
        m_MjcDataRef->qpos[pj_trans_1_qposadr] = pos_axis_1;
        m_MjcDataRef->qpos[pj_trans_2_qposadr] = pos_axis_2;
        m_MjcDataRef->qpos[pj_rot_qposadr] = 0.0f; /* @todo: should use the tf information to set the rotation angle */
    }

    void TMujocoKinematicTreeAdapter::_SetLinearVelPlanarJoint( TKinematicTreeJoint* joint_ref, const TVec3& linear_vel )
    {
            const ssize_t pj_trans_1_id = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, ( joint_ref->name() + "_trans_1" ).c_str() );
            const ssize_t pj_trans_2_id = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, ( joint_ref->name() + "_trans_2" ).c_str() );
            const ssize_t pj_trans_1_qveladr = m_MjcModelRef->jnt_dofadr[pj_trans_1_id];
            const ssize_t pj_trans_2_qveladr = m_MjcModelRef->jnt_dofadr[pj_trans_2_id];

            const auto vec_trans_1 = joint_ref->data().plane_axis_1.normalized();
            const auto vec_trans_2 = joint_ref->data().plane_axis_2.normalized();
            const auto vel_axis_1 = vec_trans_1.dot( linear_vel );
            const auto vel_axis_2 = vec_trans_2.dot( linear_vel );
            m_MjcDataRef->qvel[pj_trans_1_qveladr] = vel_axis_1;
            m_MjcDataRef->qvel[pj_trans_2_qveladr] = vel_axis_2;
    }

    void TMujocoKinematicTreeAdapter::_SetAngularVelPlanarJoint( TKinematicTreeJoint* joint_ref, const TVec3& angular_vel )
    {
        const ssize_t pj_rot_id = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, ( joint_ref->name() + "_rot" ).c_str() );
        const ssize_t pj_rot_qveladr = m_MjcModelRef->jnt_dofadr[pj_rot_id];

        const auto vec_trans_1 = joint_ref->data().plane_axis_1;
        const auto vec_trans_2 = joint_ref->data().plane_axis_2;
        const auto vec_rotation = tinymath::cross( vec_trans_1, vec_trans_2 ).normalized();
        const auto ang_speed = vec_rotation.dot( angular_vel );
        m_MjcDataRef->qvel[pj_rot_qveladr] = ang_speed;
    }

    void TMujocoKinematicTreeAdapter::_SetTransformFixedJoint( TKinematicTreeBody* body_ref, const TMat4& tf )
    {
        const ssize_t fj_rootbody_id = mj_name2id( m_MjcModelRef, mjOBJ_BODY, body_ref->name().c_str() );
        const auto world_pos = TVec3( tf.col( 3 ) );
        const auto world_quat = tinymath::quaternion( tf );
        m_MjcModelRef->body_pos[3 * fj_rootbody_id + 0] = world_pos.x();
        m_MjcModelRef->body_pos[3 * fj_rootbody_id + 1] = world_pos.y();
        m_MjcModelRef->body_pos[3 * fj_rootbody_id + 2] = world_pos.z();
        m_MjcModelRef->body_quat[4 * fj_rootbody_id + 0] = world_quat.w();
        m_MjcModelRef->body_quat[4 * fj_rootbody_id + 1] = world_quat.x();
        m_MjcModelRef->body_quat[4 * fj_rootbody_id + 2] = world_quat.y();
        m_MjcModelRef->body_quat[4 * fj_rootbody_id + 3] = world_quat.z();
    }
}}