
#include <primitives/loco_single_body_constraint_adapter_mujoco.h>

namespace loco {
namespace mujoco {

    //********************************************************************************************//
    //                              Mujoco-Adapter Interface Impl.                                //
    //********************************************************************************************//

    TIMujocoSingleBodyConstraintAdapter::TIMujocoSingleBodyConstraintAdapter()
    {
        m_MjcModelRef = nullptr;
        m_MjcDataRef = nullptr;

        m_MjcJointId = -1;
        m_MjcJointQposAdr = -1;
        m_MjcJointQvelAdr = -1;
        m_MjcJointQposNum = -1;
        m_MjcJointQvelNum = -1;
    }

    TIMujocoSingleBodyConstraintAdapter::~TIMujocoSingleBodyConstraintAdapter()
    {
        m_MjcModelRef = nullptr;
        m_MjcDataRef = nullptr;

        m_MjcJointId = -1;
        m_MjcJointQposAdr = -1;
        m_MjcJointQvelAdr = -1;
        m_MjcJointQposNum = -1;
        m_MjcJointQvelNum = -1;

        m_MjcfElementsResources.clear();
    }

    std::vector<parsing::TElement*> TIMujocoSingleBodyConstraintAdapter::elements_resources()
    {
        std::vector<parsing::TElement*> mjcf_elements;
        for ( auto& mjcf_element : m_MjcfElementsResources )
            mjcf_elements.push_back( mjcf_element.get() );
        return mjcf_elements;
    }

    std::vector<const parsing::TElement*> TIMujocoSingleBodyConstraintAdapter::elements_resources() const
    {
        std::vector<const parsing::TElement*> mjcf_elements;
        for ( auto& mjcf_element : m_MjcfElementsResources )
            mjcf_elements.push_back( mjcf_element.get() );
        return mjcf_elements;
    }

    //********************************************************************************************//
    //                              Revolute-constraint Adapter Impl                              //
    //********************************************************************************************//

    TMujocoSingleBodyRevoluteConstraintAdapter::~TMujocoSingleBodyRevoluteConstraintAdapter()
    {
        if ( m_ConstraintRef )
            m_ConstraintRef->DetachSim();
        m_ConstraintRef = nullptr;
    }

    void TMujocoSingleBodyRevoluteConstraintAdapter::Build()
    {
        auto revolute_constraint = dynamic_cast<TSingleBodyRevoluteConstraint*>( m_ConstraintRef );
        LOCO_CORE_ASSERT( revolute_constraint, "TMujocoSingleBodyRevoluteConstraintAdapter::Build >>> \
                          constraint reference must be of type \"Revolute\", for constraint named {0}", m_ConstraintRef->name() );

        auto mjcf_element = std::make_unique<parsing::TElement>( LOCO_MJCF_JOINT_TAG, parsing::eSchemaType::MJCF );
        mjcf_element->SetString( "name", revolute_constraint->name() );
        mjcf_element->SetString( "type", "hinge" );
        mjcf_element->SetString( "limited", "false" );
        mjcf_element->SetVec3( "pos", TVec3( revolute_constraint->local_tf().col( 3 ) ) );
        mjcf_element->SetVec3( "axis", TMat3( revolute_constraint->local_tf() ) * revolute_constraint->axis() );
        m_MjcfElementsResources.push_back( std::move( mjcf_element ) );
    }

    void TMujocoSingleBodyRevoluteConstraintAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoSingleBodyRevoluteConstraintAdapter::Initialize >>> \
                          constraint {0} must have a valid mjModel reference", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoSingleBodyRevoluteConstraintAdapter::Initialize >>> \
                          constraint {0} must have a valid mjData reference", m_ConstraintRef->name() );

        m_MjcJointId = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, m_ConstraintRef->name().c_str() );
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyRevoluteConstraintAdapter::Initialize >>> couldn't find \
                          associated mjc-joint for constraint {0}", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_MjcModelRef->jnt_type[m_MjcJointId] == mjJNT_HINGE, "TMujocoSingleBodyRevoluteConstraintAdapter::Initialize >>> \
                          joint associated to constraint {0} must be a hinge-joint", m_ConstraintRef->name() );

        m_MjcJointQposAdr = m_MjcModelRef->jnt_qposadr[m_MjcJointId];
        m_MjcJointQvelAdr = m_MjcModelRef->jnt_dofadr[m_MjcJointId];
        m_MjcJointQposNum = 1;
        m_MjcJointQvelNum = 1;
    }

    void TMujocoSingleBodyRevoluteConstraintAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyRevoluteConstraintAdapter::Reset >>> \
                          constraint \"{0}\" must be linked to a valid mjc-joint", m_ConstraintRef->name() );

        m_MjcDataRef->qpos[m_MjcJointQposAdr + 0] = 0.0;
        m_MjcDataRef->qvel[m_MjcJointQvelAdr + 0] = 0.0;
    }

    void TMujocoSingleBodyRevoluteConstraintAdapter::OnDetach()
    {
        m_Detached = true;
        m_ConstraintRef = nullptr;
    }

    void TMujocoSingleBodyRevoluteConstraintAdapter::SetHingeAngle( TScalar hinge_angle )
    {
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyRevoluteConstraintAdapter::SetHingeAngle >>> \
                          constraint \"{0}\" must be linked to a valid mjc-joint", m_ConstraintRef->name() );

        m_MjcDataRef->qpos[m_MjcJointQposAdr + 0] = hinge_angle;
    }

    void TMujocoSingleBodyRevoluteConstraintAdapter::SetLimits( const TVec2& limits )
    {
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyRevoluteConstraintAdapter::SetLimits >>> \
                          constraint \"{0}\" must be linked to a valid mjc-joint", m_ConstraintRef->name() );

        const bool limited = ( limits.x() > limits.y() );
        m_MjcModelRef->jnt_limited[m_MjcJointId] = limited ? 0 : 1;
        if ( limited )
        {
            m_MjcModelRef->jnt_range[2 * m_MjcJointId + 0] = limits.x();
            m_MjcModelRef->jnt_range[2 * m_MjcJointId + 1] = limits.y();
        }
    }

    void TMujocoSingleBodyRevoluteConstraintAdapter::GetHingeAngle( TScalar& dst_hinge_angle )
    {
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyRevoluteConstraintAdapter::GetHingeAngle >>> \
                          constraint \"{0}\" must be linked to a valid mjc-joint", m_ConstraintRef->name() );

        dst_hinge_angle = m_MjcDataRef->qpos[m_MjcJointQposAdr + 0];
    }

    //********************************************************************************************//
    //                              Prismatic-constraint Adapter Impl                             //
    //********************************************************************************************//

    TMujocoSingleBodyPrismaticConstraintAdapter::~TMujocoSingleBodyPrismaticConstraintAdapter()
    {
        if ( m_ConstraintRef )
            m_ConstraintRef->DetachSim();
        m_ConstraintRef = nullptr;
    }

    void TMujocoSingleBodyPrismaticConstraintAdapter::Build()
    {
        auto prismatic_constraint = dynamic_cast<TSingleBodyPrismaticConstraint*>( m_ConstraintRef );
        LOCO_CORE_ASSERT( prismatic_constraint, "TMujocoSingleBodyPrismaticConstraintAdapter::Build >>> \
                          constraint reference must be of type \"Prismatic\", for constraint named {0}", m_ConstraintRef->name() );

        auto mjcf_element = std::make_unique<parsing::TElement>( LOCO_MJCF_JOINT_TAG, parsing::eSchemaType::MJCF );
        mjcf_element->SetString( "name", prismatic_constraint->name() );
        mjcf_element->SetString( "type", "slide" );
        mjcf_element->SetString( "limited", "false" );
        mjcf_element->SetVec3( "pos", TVec3( prismatic_constraint->local_tf().col( 3 ) ) );
        mjcf_element->SetVec3( "axis", TMat3( prismatic_constraint->local_tf() ) * prismatic_constraint->axis() );
        m_MjcfElementsResources.push_back( std::move( mjcf_element ) );
    }

    void TMujocoSingleBodyPrismaticConstraintAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_MjcModelRef, "TMujocoSingleBodyPrismaticConstraintAdapter::Initialize >>> \
                          constraint {0} must have a valid mjModel reference", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_MjcDataRef, "TMujocoSingleBodyPrismaticConstraintAdapter::Initialize >>> \
                          constraint {0} must have a valid mjData reference", m_ConstraintRef->name() );

        m_MjcJointId = mj_name2id( m_MjcModelRef, mjOBJ_JOINT, m_ConstraintRef->name().c_str() );
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyPrismaticConstraintAdapter::Initialize >>> couldn't find \
                          associated mjc-joint for constraint {0}", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_MjcModelRef->jnt_type[m_MjcJointId] == mjJNT_SLIDE, "TMujocoSingleBodyPrismaticConstraintAdapter::Initialize >>> \
                          joint associated to constraint {0} must be a slide-joint", m_ConstraintRef->name() );

        m_MjcJointQposAdr = m_MjcModelRef->jnt_qposadr[m_MjcJointId];
        m_MjcJointQvelAdr = m_MjcModelRef->jnt_dofadr[m_MjcJointId];
        m_MjcJointQposNum = 1;
        m_MjcJointQvelNum = 1;
    }

    void TMujocoSingleBodyPrismaticConstraintAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyPrismaticConstraintAdapter::Reset >>> \
                          constraint \"{0}\" must be linked to a valid mjc-joint", m_ConstraintRef->name() );

        m_MjcDataRef->qpos[m_MjcJointQposAdr + 0] = 0.0;
        m_MjcDataRef->qvel[m_MjcJointQvelAdr + 0] = 0.0;
    }

    void TMujocoSingleBodyPrismaticConstraintAdapter::OnDetach()
    {
        m_Detached = true;
        m_ConstraintRef = nullptr;
    }

    void TMujocoSingleBodyPrismaticConstraintAdapter::SetSlidePosition( TScalar slide_position )
    {
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyPrismaticConstraintAdapter::SetHingeAngle >>> \
                          constraint \"{0}\" must be linked to a valid mjc-joint", m_ConstraintRef->name() );

        m_MjcDataRef->qpos[m_MjcJointQposAdr + 0] = slide_position;
    }

    void TMujocoSingleBodyPrismaticConstraintAdapter::SetLimits( const TVec2& limits )
    {
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyPrismaticConstraintAdapter::SetLimits >>> \
                          constraint \"{0}\" must be linked to a valid mjc-joint", m_ConstraintRef->name() );

        const bool limited = ( limits.x() > limits.y() );
        m_MjcModelRef->jnt_limited[m_MjcJointId] = limited ? 0 : 1;
        if ( limited )
        {
            m_MjcModelRef->jnt_range[2 * m_MjcJointId + 0] = limits.x();
            m_MjcModelRef->jnt_range[2 * m_MjcJointId + 1] = limits.y();
        }
    }

    void TMujocoSingleBodyPrismaticConstraintAdapter::GetSlidePosition( TScalar& dst_slide_position )
    {
        LOCO_CORE_ASSERT( m_MjcJointId >= 0, "TMujocoSingleBodyPrismaticConstraintAdapter::GetHingeAngle >>> \
                          constraint \"{0}\" must be linked to a valid mjc-joint", m_ConstraintRef->name() );

        dst_slide_position = m_MjcDataRef->qpos[m_MjcJointQposAdr + 0];
    }

}}