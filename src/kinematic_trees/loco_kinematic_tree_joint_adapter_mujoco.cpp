
#include <kinematic_trees/loco_kinematic_tree_joint_adapter_mujoco.h>

namespace loco {
namespace kintree {

    TMujocoKinematicTreeJointAdapter::TMujocoKinematicTreeJointAdapter( TKinematicTreeJoint* joint_ref )
        : TIKinematicTreeJointAdapter( joint_ref ) {}

    TMujocoKinematicTreeJointAdapter::~TMujocoKinematicTreeJointAdapter()
    {
        m_MjcJointId = -1;
        m_MjcDofId = -1;
        m_MjcJointQposAdr = -1;
        m_MjcJointQvelAdr = -1;

        m_MjcModelRef = nullptr;
        m_MjcDataRef = nullptr;
        m_MjcfElementsResources.clear();
    }

    void TMujocoKinematicTreeJointAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_JointRef, "TMujocoKinematicTreeJointAdapter::Build >>> must have a valid kintree-joint object (got nullptr)" );

        const auto joint_type = m_JointRef->type();
        if ( joint_type == eJointType::FIXED )
            return; // Fixed-joints behave like non-existent mjcf elements in the mjcf format

        if ( joint_type != eJointType::PLANAR )
        {
            auto mjcf_element = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_JOINT_TAG, parsing::eSchemaType::MJCF );
            mjcf_element->SetString( "name", m_JointRef->name() );
            mjcf_element->SetString( "type", mujoco::enumJoint_to_mjcJoint( joint_type ) );
            // Set local position of this joint w.r.t. to its parent body ------------------------------
            auto local_tf = m_JointRef->local_tf();
            const auto rel_position = local_tf.col( 3 );
            const auto rel_rotation = TMat3( local_tf );
            mjcf_element->SetVec3( "pos", rel_position );
            // -----------------------------------------------------------------------------------------
            if ( joint_type == eJointType::REVOLUTE || joint_type == eJointType::PRISMATIC )
                mjcf_element->SetVec3( "axis", rel_rotation * m_JointRef->data().local_axis );
            const auto limits = m_JointRef->data().limits;
            const auto limited = limits.x() < limits.y();
            mjcf_element->SetString( "limited", ( limited ) ? "true" : "false" );
            if ( limited )
            {
                /**/ if ( joint_type == eJointType::SPHERICAL )
                    mjcf_element->SetVec2( "range", { 0.0f, Rad2degrees( limits.y() ) } );
                else if ( joint_type == eJointType::REVOLUTE )
                    mjcf_element->SetVec2( "range", { Rad2degrees( limits.x() ), Rad2degrees( limits.y() ) } );
                else if ( joint_type == eJointType::PRISMATIC )
                    mjcf_element->SetVec2( "range", limits );
            }

            if ( m_JointRef->data().stiffness > loco::EPS )
                mjcf_element->SetFloat( "stiffness", m_JointRef->data().stiffness );
            if ( m_JointRef->data().armature > loco::EPS )
                mjcf_element->SetFloat( "armature", m_JointRef->data().armature );
            if ( m_JointRef->data().damping > loco::EPS )
                mjcf_element->SetFloat( "damping", m_JointRef->data().damping );

            m_MjcfElementsResources.push_back( std::move( mjcf_element ) );
        }
        else
        {
            const auto trans_axis_1 = m_JointRef->data().plane_axis_1;
            const auto trans_axis_2 = m_JointRef->data().plane_axis_2;
            const auto rot_axis = tinymath::cross( trans_axis_1, trans_axis_2 );

            auto mjcf_slide_axis_1 = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_JOINT_TAG, parsing::eSchemaType::MJCF );
            mjcf_slide_axis_1->SetString( "name", m_JointRef->name() + "_trans_1" );
            mjcf_slide_axis_1->SetString( "type", "slide" );
            mjcf_slide_axis_1->SetString( "limited", "false" );
            mjcf_slide_axis_1->SetVec3( "pos", { 0.0f, 0.0f, 0.0f } );
            mjcf_slide_axis_1->SetVec3( "axis", trans_axis_1 );
            m_MjcfElementsResources.push_back( std::move( mjcf_slide_axis_1 ) );

            auto mjcf_slide_axis_2 = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_JOINT_TAG, parsing::eSchemaType::MJCF );
            mjcf_slide_axis_2->SetString( "name", m_JointRef->name() + "_trans_2" );
            mjcf_slide_axis_2->SetString( "type", "slide" );
            mjcf_slide_axis_2->SetString( "limited", "false" );
            mjcf_slide_axis_2->SetVec3( "pos", { 0.0f, 0.0f, 0.0f } );
            mjcf_slide_axis_2->SetVec3( "axis", trans_axis_2 );
            m_MjcfElementsResources.push_back( std::move( mjcf_slide_axis_2 ) );

            auto mjcf_hinge_axis = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_JOINT_TAG, parsing::eSchemaType::MJCF );
            mjcf_hinge_axis->SetString( "name", m_JointRef->name() + "_rot" );
            mjcf_hinge_axis->SetString( "type", "hinge" );
            mjcf_hinge_axis->SetString( "limited", "false" );
            mjcf_hinge_axis->SetVec3( "pos", { 0.0f, 0.0f, 0.0f } );
            mjcf_hinge_axis->SetVec3( "axis", rot_axis );
            m_MjcfElementsResources.push_back( std::move( mjcf_hinge_axis ) );
        }
    }

    void TMujocoKinematicTreeJointAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::Initialize >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::Initialize >>> must have a valid mjData reference" );

        const auto joint_type = m_JointRef->type();
        if ( joint_type == eJointType::FIXED )
            return; // Fixed joints don't have any handle to mjc-joints
        if ( joint_type == eJointType::PLANAR )
            return; // A planar joint requires multiple mjc-joints, so don't use handles

        m_MjcJointId = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, m_JointRef->name().c_str() );
        if ( m_MjcJointId < 0 )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeJointAdapter::Initialize >>> couldn't find associated \
                              mjc-joint for kintree-joint {0} (returned mjc-joint-id < 0)", m_JointRef->name() );
            return;
        }

        m_MjcJointQposAdr = m_MjcModelRef->jnt_qposadr[m_MjcJointId];
        m_MjcJointQvelAdr = m_MjcModelRef->jnt_dofadr[m_MjcJointId];

        // Get the dof-id, which can be referenced by the body-parent information
        ssize_t mjc_body_parent_id = m_MjcModelRef->jnt_bodyid[m_MjcJointId];
        m_MjcDofId = m_MjcModelRef->body_dofadr[mjc_body_parent_id];
    }

    void TMujocoKinematicTreeJointAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::Reset >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::Reset >>> must have a valid mjData reference" );

        if ( m_MjcJointId < 0 )
            return;

        const bool is_root_joint = ( ( m_JointRef->parent() ) && ( !m_JointRef->parent()->parent() ) );
        if ( is_root_joint )
            return; // Root-joint case is handled by the kinematic-tree itself

        const auto joint_type = m_JointRef->type();
        const auto qpos0 = m_JointRef->qpos0();
        const auto qvel0 = m_JointRef->qvel0();
        const auto num_qpos = m_JointRef->num_qpos();
        const auto num_qvel = m_JointRef->num_qvel();

        if ( joint_type == eJointType::SPHERICAL ) // take into account w-x-y-z layout used by mujoco
        {
            m_MjcDataRef->qpos[m_MjcJointQposAdr + 0] = qpos0[3];
            m_MjcDataRef->qpos[m_MjcJointQposAdr + 1] = qpos0[0];
            m_MjcDataRef->qpos[m_MjcJointQposAdr + 2] = qpos0[1];
            m_MjcDataRef->qpos[m_MjcJointQposAdr + 3] = qpos0[2];
        }
        else
        {
            for ( ssize_t i = 0; i < num_qpos; i++ )
                m_MjcDataRef->qpos[m_MjcJointQposAdr + i] = qpos0[i];
        }

        for ( ssize_t i = 0; i < num_qvel; i++ )
            m_MjcDataRef->qvel[m_MjcJointQvelAdr + i] = qvel0[i];
    }

    void TMujocoKinematicTreeJointAdapter::SetQpos( const std::vector<TScalar>& qpos )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::SetQpos >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::SetQpos >>> must have a valid mjData reference" );

        if ( m_MjcJointId < 0 )
            return;

        const ssize_t num_qpos = m_JointRef->num_qpos();
        if ( qpos.size() != num_qpos )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeJointAdapter::SetQpos >>> mismatch between the number of generalized \
                              coordinates expected for joint {0} of type {1}. Expected: {2}, got: {3}", m_JointRef->name(),
                              loco::ToString( m_JointRef->type() ), std::to_string( num_qpos ), std::to_string( qpos.size() ) );
            return;
        }

        for ( ssize_t i = 0; i < num_qpos; i++ )
            m_MjcDataRef->qpos[m_MjcJointQposAdr + i] = qpos[i];
    }

    void TMujocoKinematicTreeJointAdapter::SetQvel( const std::vector<TScalar>& qvel )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::SetQvel >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::SetQvel >>> must have a valid mjData reference" );

        if ( m_MjcJointId < 0 )
            return;

        const ssize_t num_qvel = m_JointRef->num_qvel();
        if ( qvel.size() != num_qvel )
        {
            LOCO_CORE_ERROR( "TMujocoKinematicTreeJointAdapter::SetQVEL >>> mismatch between the number of degrees of \
                              freedom expected for joint {0} of type {1}. Expected: {2}, got: {3}", m_JointRef->name(),
                              loco::ToString( m_JointRef->type() ), std::to_string( num_qvel ), std::to_string( qvel.size() ) );
            return;
        }

        for ( ssize_t i = 0; i < num_qvel; i++ )
            m_MjcDataRef->qvel[m_MjcJointQvelAdr + i] = qvel[i];
    }

    void TMujocoKinematicTreeJointAdapter::SetLocalTransform( const TMat4& local_tf )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::SetLocalTransform >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::SetLocalTransform >>> must have a valid mjData reference" );

        if ( m_MjcJointId < 0 )
            return;
        // @todo: check case for rel-rotation changes, as it might change the local axis
        const auto rel_position = local_tf.col( 3 );
        m_MjcModelRef->jnt_pos[3 * m_MjcJointId + 0] = rel_position.x();
        m_MjcModelRef->jnt_pos[3 * m_MjcJointId + 1] = rel_position.y();
        m_MjcModelRef->jnt_pos[3 * m_MjcJointId + 2] = rel_position.z();
    }

    void TMujocoKinematicTreeJointAdapter::ChangeStiffness( const TScalar& stiffness )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::ChangeStiffness >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::ChangeStiffness >>> must have a valid mjData reference" );

        if ( m_MjcJointId < 0 )
            return;

        m_MjcModelRef->jnt_stiffness[m_MjcJointId] = stiffness;
    }

    void TMujocoKinematicTreeJointAdapter::ChangeArmature( const TScalar& armature )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::ChangeArmature >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::ChangeArmature >>> must have a valid mjData reference" );

        if ( m_MjcDofId < 0 )
            return;

        m_MjcModelRef->dof_armature[m_MjcDofId] = armature;
    }

    void TMujocoKinematicTreeJointAdapter::ChangeDamping( const TScalar& damping )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::ChangeDamping >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::ChangeDamping >>> must have a valid mjData reference" );

        if ( m_MjcDofId < 0 )
            return;

        m_MjcModelRef->dof_damping[m_MjcDofId] = damping;
    }

    void TMujocoKinematicTreeJointAdapter::ChangeAxis( const TVec3& axis )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::ChangeAxis >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::ChangeAxis >>> must have a valid mjData reference" );

        if ( m_MjcJointId < 0 )
            return;

        const auto axis_normalized = axis.normalized();
        m_MjcModelRef->jnt_axis[3 * m_MjcDofId + 0] = axis_normalized.x();
        m_MjcModelRef->jnt_axis[3 * m_MjcDofId + 1] = axis_normalized.y();
        m_MjcModelRef->jnt_axis[3 * m_MjcDofId + 2] = axis_normalized.z();
    }

    void TMujocoKinematicTreeJointAdapter::ChangeLimits( const TVec2& limits )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::ChangeAxis >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::ChangeAxis >>> must have a valid mjData reference" );

        if ( m_MjcJointId < 0 )
            return;

        const auto joint_type = m_JointRef->type();
        const auto limited = ( limits.x() < limits.y() );
        m_MjcModelRef->jnt_limited[m_MjcJointId] = (limited) ? 1 : 0;
        if ( limited )
        {
            /**/ if ( joint_type == eJointType::SPHERICAL )
            {
                m_MjcModelRef->jnt_range[2 * m_MjcJointId + 0] = 0.0f;
                m_MjcModelRef->jnt_range[2 * m_MjcJointId + 1] = limits.y();
            }
            else if ( joint_type == eJointType::REVOLUTE )
            {
                // @todo: check if limits for revolute joints are required in degrees or radians
                m_MjcModelRef->jnt_range[2 * m_MjcJointId + 0] = Rad2degrees( limits.x() );
                m_MjcModelRef->jnt_range[2 * m_MjcJointId + 1] = Rad2degrees( limits.y() );
            }
            else if ( joint_type == eJointType::PRISMATIC )
            {
                m_MjcModelRef->jnt_range[2 * m_MjcJointId + 0] = limits.x();
                m_MjcModelRef->jnt_range[2 * m_MjcJointId + 1] = limits.y();
            }
        }
    }

    void TMujocoKinematicTreeJointAdapter::GetQpos( std::vector<TScalar>& dst_qpos )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::ChangeAxis >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::ChangeAxis >>> must have a valid mjData reference" );

        dst_qpos.clear();
        if ( m_MjcJointId < 0 )
            return;

        const ssize_t num_qpos = m_JointRef->num_qpos();
        for ( ssize_t i = 0; i < num_qpos; i++ )
            dst_qpos.push_back( m_MjcDataRef->qpos[m_MjcJointQposAdr + i] );
    }

    void TMujocoKinematicTreeJointAdapter::GetQvel( std::vector<TScalar>& dst_qvel )
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoKinematicTreeJointAdapter::ChangeAxis >>> must have a valid mjModel reference" );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoKinematicTreeJointAdapter::ChangeAxis >>> must have a valid mjData reference" );

        dst_qvel.clear();
        if ( m_MjcJointId < 0 )
            return;

        const ssize_t num_qvel = m_JointRef->num_qvel();
        for ( ssize_t i = 0; i < num_qvel; i++ )
            dst_qvel.push_back( m_MjcDataRef->qvel[m_MjcJointQvelAdr + i] );
    }

    std::vector<parsing::TElement*> TMujocoKinematicTreeJointAdapter::elements_resources()
    {
        std::vector<parsing::TElement*> mjcf_elements;
        for ( auto& mjcf_element : m_MjcfElementsResources )
            mjcf_elements.push_back( mjcf_element.get() );
        return mjcf_elements;
    }

    std::vector<const parsing::TElement*> TMujocoKinematicTreeJointAdapter::elements_resources() const
    {
        std::vector<const parsing::TElement*> mjcf_elements;
        for ( auto& mjcf_element : m_MjcfElementsResources )
            mjcf_elements.push_back( mjcf_element.get() );
        return mjcf_elements;
    }

}}