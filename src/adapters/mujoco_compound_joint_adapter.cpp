
#include <adapters/mujoco_compound_joint_adapter.h>

namespace tysoc {

    TMjcCompoundJointAdapter::TMjcCompoundJointAdapter( TJoint* jointRef )
        : TIJointAdapter( jointRef )
    {
        m_mjcModelRef = nullptr;
        m_mjcDataRef = nullptr;
        m_mjcfXmlResource = nullptr;

        m_mjcJointId = -1;
        m_mjcQposAdr = -1;
        m_mjcQvelAdr = -1;
        m_mjcQposNum = 0;
        m_mjcQvelNum = 0;
    }

    TMjcCompoundJointAdapter::~TMjcCompoundJointAdapter()
    {
        m_mjcModelRef = nullptr;
        m_mjcDataRef = nullptr;
        m_mjcfXmlResource = nullptr;
    }

    void TMjcCompoundJointAdapter::build()
    {
        if ( !m_jointRef )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundJointAdapter::build() >>> reference-joint is nullptr. There\
                               must always be a valid joint for an adapter" );
            return;
        }

        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model

        m_mjcfXmlResource = new mjcf::GenericElement( "joint" );
        m_mjcfXmlResource->setAttributeString( "name", m_jointRef->name() );
        m_mjcfXmlResource->setAttributeString( "type", mujoco::enumJointToMjcType( m_jointRef->type() ) );

        if ( m_jointRef->type() == eJointType::FREE )
            return; // free joints require no more information

        m_mjcfXmlResource->setAttributeVec3( "pos", m_jointRef->localTf().getPosition() );
        m_mjcfXmlResource->setAttributeVec3( "axis", m_jointRef->axis() ); // @todo: check if axis should actually be in local-space

        auto _limited = ( m_jointRef->limitLow() < m_jointRef->limitHigh() );
        m_mjcfXmlResource->setAttributeString( "limited", ( _limited ) ? "true" : "false" );
        if ( _limited )
        {
            if ( m_jointRef->type() == eJointType::SPHERICAL )
                m_mjcfXmlResource->setAttributeVec2( "range", { 0.0f, rad2degrees( m_jointRef->limitHigh() ) } );
            else
                m_mjcfXmlResource->setAttributeVec2( "range", { rad2degrees( m_jointRef->limitLow() ),
                                                               rad2degrees( m_jointRef->limitHigh() ) } );
        }

        auto _data = m_jointRef->data();
        if ( _data.stiffness != 0.0f )
            m_mjcfXmlResource->setAttributeFloat( "stiffness", _data.stiffness );
        if ( _data.armature != 0.0f )
            m_mjcfXmlResource->setAttributeFloat( "armature", _data.armature );
        if ( _data.damping != 0.0f )
            m_mjcfXmlResource->setAttributeFloat( "damping", _data.damping );
        if ( _data.ref != 0.0f )
            m_mjcfXmlResource->setAttributeFloat( "ref", _data.ref );
    }

    void TMjcCompoundJointAdapter::reset()
    {
        if ( m_jointRef->type() == eJointType::FREE )
            return;

        for ( int i = 0; i < m_mjcQposNum; i++ )
            m_mjcDataRef->qpos[m_mjcQposAdr + i] = m_mjcModelRef->qpos0[m_mjcQposAdr + i];

        for ( int i = 0; i < m_mjcQvelNum; i++ )
            m_mjcDataRef->qvel[m_mjcQvelAdr + i] = 0.0f;
    }

    void TMjcCompoundJointAdapter::preStep()
    {
        // nothing to setup previous to a simulation step (other methods are enough)
    }

    void TMjcCompoundJointAdapter::postStep()
    {
        // nothing to update after a simulation step was taken (other methods are enough)
    }

    void TMjcCompoundJointAdapter::setLocalPosition( const TVec3& localPosition )
    {
        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model
        assert( m_mjcJointId != -1 );

        /* change the pivot of the joint */
        m_mjcModelRef->jnt_pos[m_mjcJointId + 0] = localPosition.x;
        m_mjcModelRef->jnt_pos[m_mjcJointId + 1] = localPosition.y;
        m_mjcModelRef->jnt_pos[m_mjcJointId + 2] = localPosition.z;
    }

    void TMjcCompoundJointAdapter::setLocalRotation( const TMat3& localRotation )
    {
        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model
        assert( m_mjcJointId != -1 );

        /* change the axis to point to the z-axis of the rotation matrix */
        m_mjcModelRef->jnt_axis[m_mjcJointId + 0] = localRotation.buff[6];
        m_mjcModelRef->jnt_axis[m_mjcJointId + 1] = localRotation.buff[7];
        m_mjcModelRef->jnt_axis[m_mjcJointId + 2] = localRotation.buff[8];
    }

    void TMjcCompoundJointAdapter::setLocalTransform( const TMat4& localTransform )
    {
        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model
        assert( m_mjcJointId != -1 );

        auto _localPosition = localTransform.getPosition();
        auto _localRotation = localTransform.getRotation();

        /* change the pivot of the joint */
        m_mjcModelRef->jnt_pos[m_mjcJointId + 0] = _localPosition.x;
        m_mjcModelRef->jnt_pos[m_mjcJointId + 1] = _localPosition.y;
        m_mjcModelRef->jnt_pos[m_mjcJointId + 2] = _localPosition.z;

        /* change the axis to point to the z-axis of the rotation matrix */
        m_mjcModelRef->jnt_axis[m_mjcJointId + 0] = _localRotation.buff[6];
        m_mjcModelRef->jnt_axis[m_mjcJointId + 1] = _localRotation.buff[7];
        m_mjcModelRef->jnt_axis[m_mjcJointId + 2] = _localRotation.buff[8];
    }

    void TMjcCompoundJointAdapter::setQpos( const std::array< TScalar, TYSOC_MAX_NUM_QPOS >& qpos )
    {
        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model
        assert( m_mjcJointId != -1 );

        for ( int i = 0; i < m_mjcQposNum; i++ )
            m_mjcDataRef->qpos[m_mjcQposAdr + i] = qpos[i];
    }

    void TMjcCompoundJointAdapter::setQvel( const std::array< TScalar, TYSOC_MAX_NUM_QVEL >& qvel )
    {
        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model
        assert( m_mjcJointId != -1 );

        for ( int i = 0; i < m_mjcQvelNum; i++ )
            m_mjcDataRef->qvel[m_mjcQvelAdr + i] = qvel[i];
    }

    void TMjcCompoundJointAdapter::getQpos( std::array< TScalar, TYSOC_MAX_NUM_QPOS >& dstQpos )
    {
        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model
        assert( m_mjcJointId != -1 );

        for ( int i = 0; i < m_mjcQposNum; i++ )
            dstQpos[i] = m_mjcDataRef->qpos[m_mjcQposAdr + i];
    }

    void TMjcCompoundJointAdapter::getQvel( std::array< TScalar, TYSOC_MAX_NUM_QVEL >& dstQvel )
    {
        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model
        assert( m_mjcJointId != -1 );

        for ( int i = 0; i < m_mjcQvelNum; i++ )
            dstQvel[i] = m_mjcDataRef->qvel[m_mjcQvelAdr + i];
    }

    void TMjcCompoundJointAdapter::changeLimits( const TVec2& limits )
    {
        assert( m_mjcJointId != -1 );

        bool _limited = ( limits.x < limits.y );
        m_mjcModelRef->jnt_limited[m_mjcJointId] = ( _limited ) ? 1 : 0;
        if ( _limited )
        {
            auto _jointType = m_jointRef->type();
            if ( _jointType == eJointType::REVOLUTE )
            {
                m_mjcModelRef->jnt_range[m_mjcJointId + 0] = tysoc::rad2degrees( limits.x );
                m_mjcModelRef->jnt_range[m_mjcJointId + 1] = tysoc::rad2degrees( limits.y );
            }
            else if ( _jointType == eJointType::PRISMATIC )
            {
                m_mjcModelRef->jnt_range[m_mjcJointId + 0] = limits.x;
                m_mjcModelRef->jnt_range[m_mjcJointId + 1] = limits.y;
            }
            else if ( _jointType == eJointType::SPHERICAL )
            {
                // @todo: check proper usage of spherical joints in mujoco (usage of axis instead of angles)
                m_mjcModelRef->jnt_range[m_mjcJointId + 0] = 0;
                m_mjcModelRef->jnt_range[m_mjcJointId + 1] = limits.y;
            }
            else // @todo: add support for planar joints
            {
                TYSOC_CORE_WARN( "TMjcCompoundJointAdapter::changeLimits() >>> joint \"{0}\" has \
                                  unsupported (yet) joint-type \"{1}\"", m_jointRef->name(), tysoc::toString( _jointType ) );
            }
        }
    }

    void TMjcCompoundJointAdapter::onResourcesCreated()
    {
        assert( m_mjcModelRef );
        assert( m_mjcDataRef );
        assert( m_jointRef );

        if ( m_jointRef->type() == eJointType::FIXED )
            return; // fixed-joints are just not added in a mjc-model

        m_mjcJointId = mj_name2id( m_mjcModelRef, mjOBJ_JOINT, m_jointRef->name().c_str() );

        if ( m_mjcJointId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundJointAdapter::onResourcesCreated() >>> couldn't find the \
                               associated mjc-joint for joint \"{0}\"", m_jointRef->name() );
            return;
        }

        // grab the qpos and qvel addresses as well to set both qpos and qvel as needed
        m_mjcQposAdr = m_mjcModelRef->jnt_qposadr[m_mjcJointId];
        m_mjcQvelAdr = m_mjcModelRef->jnt_dofadr[m_mjcJointId];

        // setup the number of generalized coordinates of this joint according to its type
        auto _jointType = m_jointRef->type();
        auto& _jointData = m_jointRef->dataRef();
        if ( _jointType == eJointType::REVOLUTE )
        {
            // revolute joints have just 1dof, and 1qpos for this dof
            m_mjcQposNum = _jointData.nqpos = 1;
            m_mjcQvelNum = _jointData.nqvel = 1;
        }
        else if ( _jointType == eJointType::PRISMATIC )
        {
            // prismatic joints have just 1dof, and 1qpos for this dof
            m_mjcQposNum = _jointData.nqpos = 1;
            m_mjcQvelNum = _jointData.nqvel = 1;
        }
        else if ( _jointType == eJointType::SPHERICAL )
        {
            // spherical joints have 3dofs, and 4qpos (quaternion) representing it
            m_mjcQposNum = _jointData.nqpos = 4;
            m_mjcQvelNum = _jointData.nqvel = 3;
        }
        else if ( _jointType == eJointType::FREE )
        {
            // free joints have 6dofs, and 7qpos (quat + xyz) representing it
            m_mjcQposNum = _jointData.nqpos = 7;
            m_mjcQvelNum = _jointData.nqvel = 6;
        }
        else if ( _jointType == eJointType::FIXED )
        {
            // fixed joints have 0dofs, as expected
            m_mjcQposNum = _jointData.nqpos = 0;
            m_mjcQvelNum = _jointData.nqvel = 0;
        }
        else
        {
            TYSOC_CORE_ERROR( "TMjcCompoundJointAdapter::onResourcesCreated() >>> joint \"{0}\" has \
                               unsupported type \"{1}\"", m_jointRef->name(), tysoc::toString( _jointType ) );
        }

        /* setup initial generalized coordinates and degrees of freedom */
        auto _qpos0 = m_jointRef->getQpos0();
        for ( int i = 0; i < m_mjcQposNum; i++ )
            m_mjcModelRef->qpos0[m_mjcQposAdr + i] = _qpos0[i];
    }

    extern "C" TIJointAdapter* simulation_createCompoundJointAdapter( TJoint* jointRef )
    {
        return new TMjcCompoundJointAdapter( jointRef );
    }
}