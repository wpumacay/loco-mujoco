
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

    }

    void TMjcCompoundJointAdapter::postStep()
    {

    }

    void TMjcCompoundJointAdapter::setLocalPosition( const TVec3& position )
    {

    }

    void TMjcCompoundJointAdapter::setLocalRotation( const TMat3& rotation )
    {

    }

    void TMjcCompoundJointAdapter::setLocalTransform( const TMat4& transform )
    {

    }

    void TMjcCompoundJointAdapter::setQpos( const std::array< TScalar, TYSOC_MAX_NUM_QPOS >& qpos )
    {
        for ( int i = 0; i < m_mjcQposNum; i++ )
            m_mjcDataRef->qpos[m_mjcQposAdr + i] = qpos[i];
    }

    void TMjcCompoundJointAdapter::setQvel( const std::array< TScalar, TYSOC_MAX_NUM_QVEL >& qvel )
    {
        for ( int i = 0; i < m_mjcQvelNum; i++ )
            m_mjcDataRef->qvel[m_mjcQvelAdr + i] = qvel[i];
    }

    void TMjcCompoundJointAdapter::getQpos( std::array< TScalar, TYSOC_MAX_NUM_QPOS >& dstQpos )
    {
        for ( int i = 0; i < m_mjcQposNum; i++ )
            dstQpos[i] = m_mjcDataRef->qpos[m_mjcQposAdr + i];
    }

    void TMjcCompoundJointAdapter::getQvel( std::array< TScalar, TYSOC_MAX_NUM_QVEL >& dstQvel )
    {
        for ( int i = 0; i < m_mjcQvelNum; i++ )
            dstQvel[i] = m_mjcDataRef->qvel[m_mjcQvelAdr + i];
    }

    void TMjcCompoundJointAdapter::changeLimits( const TVec2& limits )
    {

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
            TYSOC_CORE_ERROR( "TMjcCompoundJointAdapter::onResourcesCreated() >>> couldn't fin the \
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