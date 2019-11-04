
#include <adapters/mujoco_body_adapter.h>

// @todo: replace wild referencing using mjcf generic elements, as we're currently leaking if restarting (not freeing xmlassets)
// @todo: define some sort of ownership relation between generic elements to avoid dangling references

namespace tysoc {

    TMjcBodyAdapter::TMjcBodyAdapter( TBody* bodyPtr )
        : TIBodyAdapter( bodyPtr )
    {
        m_mjcModelPtr       = nullptr;
        m_mjcDataPtr        = nullptr;
        m_mjcfXmlResource   = nullptr;
        m_mjcBodyId         = -1;
        m_mjcJointId        = -1;
        m_mjcQposAdr        = -1;
        m_mjcQvelAdr        = -1;
    }

    TMjcBodyAdapter::~TMjcBodyAdapter()
    {
        if ( m_mjcfXmlResource )
            delete m_mjcfXmlResource;

        m_mjcBodyId  = -1;
        m_mjcJointId = -1;
        m_mjcQposAdr = -1;
        m_mjcQvelAdr = -1;

        m_mjcModelPtr = nullptr;
        m_mjcDataPtr = nullptr;
        m_mjcfXmlResource = nullptr;
        m_mjcfXmlAssetResources = nullptr;
    }

    void TMjcBodyAdapter::build()
    {
        if ( !m_bodyPtr )
        {
            std::cout << "ERROR> tried to create mjcf resources for a null body" << std::endl;
            return;
        }

        m_mjcfXmlResource = new mjcf::GenericElement( "body" );
        m_mjcfXmlAssetResources = new mjcf::GenericElement( "asset" );

        m_mjcfXmlResource->setAttributeString( "name", m_bodyPtr->name() );
        m_mjcfXmlResource->setAttributeVec3( "pos", m_bodyPtr->pos() );
        m_mjcfXmlResource->setAttributeVec4( "quat", mujoco::quat2MjcfQuat( m_bodyPtr->quat() ) );

        // add a free-joint in case the object is not static
        if ( m_bodyPtr->dyntype() != eDynamicsType::STATIC )
        {
            auto _freejointXmlRes = new mjcf::GenericElement( "freejoint" );
            _freejointXmlRes->setAttributeString( "name", m_bodyPtr->name() + "_jnt_free" );

            // add this freejoint to our body resource
            m_mjcfXmlResource->children.push_back( _freejointXmlRes );
        }

        /* collect resources from colliders */
        auto _collisions = m_bodyPtr->collisions();
        for ( auto _collision : _collisions )
        {
            auto _collisionAdapter = dynamic_cast< TMjcCollisionAdapter* >( _collision->adapter() );
            _collisionAdapter->build();

            m_mjcfXmlResource->children.push_back( _collisionAdapter->mjcfResource() );
            if ( _collisionAdapter->mjcfAssetResource() )
                m_mjcfXmlAssetResources->children.push_back( _collisionAdapter->mjcfAssetResource() );
        }
    }

    void TMjcBodyAdapter::reset()
    {
        assert( m_bodyPtr );
        assert( m_mjcBodyId != -1 );

        if ( m_bodyPtr->dyntype() != eDynamicsType::STATIC )
        {
            // set all generalized coordinates to their initial values
            for ( size_t i = 0; i < m_mjcQposNum; i++ )
                m_mjcDataPtr->qpos[m_mjcQposAdr + i] = m_mjcModelPtr->qpos0[m_mjcQposAdr + i];

            // set all generalized velocities to 0
            for ( size_t i = 0; i < m_mjcQvelNum; i++ )
                m_mjcDataPtr->qvel[m_mjcQvelAdr + i] = 0.0;
        }
        else
        {
            // use directly the relative position stored in the model
            auto _pos0 = m_bodyPtr->pos0();
            auto _quat0 = m_bodyPtr->quat0();

            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 0] = _pos0.x;
            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 1] = _pos0.y;
            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 2] = _pos0.z;

            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 0] = _quat0.w;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 1] = _quat0.x;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 2] = _quat0.y;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 3] = _quat0.z;
        }
    }

    void TMjcBodyAdapter::update()
    {
        // do nothing for now, because so far we only need to use the overriden methods
    }

    void TMjcBodyAdapter::setPosition( const TVec3& position )
    {
        assert( m_bodyPtr );
        assert( m_mjcBodyId != -1 );

        if ( m_bodyPtr->dyntype() != eDynamicsType::STATIC )
        {
            m_mjcDataPtr->qpos[m_mjcQposAdr + 0] = position.x;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 1] = position.y;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 2] = position.z;
        }
        else
        {
            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 0] = position.x;
            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 1] = position.y;
            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 2] = position.z;
        }
    }

    void TMjcBodyAdapter::setRotation( const TMat3& rotation )
    {
        assert( m_bodyPtr );
        assert( m_mjcBodyId != -1 );

        auto _quat = TMat3::toQuaternion( rotation );

        if ( m_bodyPtr->dyntype() != eDynamicsType::STATIC )
        {
            m_mjcDataPtr->qpos[m_mjcQposAdr + 3] = _quat.w;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 4] = _quat.x;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 5] = _quat.y;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 6] = _quat.z;
        }
        else
        {
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 0] = _quat.w;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 1] = _quat.x;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 2] = _quat.y;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 3] = _quat.z;
        }
    }

    void TMjcBodyAdapter::setTransform( const TMat4& transform )
    {
        assert( m_bodyPtr );
        assert( m_mjcBodyId != -1 );

        auto _pos = transform.getPosition();
        auto _quat = transform.getRotQuaternion();

        if ( m_bodyPtr->dyntype() != eDynamicsType::STATIC )
        {
            m_mjcDataPtr->qpos[m_mjcQposAdr + 0] = _pos.x;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 1] = _pos.y;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 2] = _pos.z;

            m_mjcDataPtr->qpos[m_mjcQposAdr + 3] = _quat.w;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 4] = _quat.x;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 5] = _quat.y;
            m_mjcDataPtr->qpos[m_mjcQposAdr + 6] = _quat.z;
        }
        else
        {
            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 0] = _pos.x;
            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 1] = _pos.y;
            m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 2] = _pos.z;

            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 0] = _quat.w;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 1] = _quat.x;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 2] = _quat.y;
            m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 3] = _quat.z;
        }
    }

    void TMjcBodyAdapter::getPosition( TVec3& dstPosition )
    {
        assert( m_bodyPtr );
        assert( m_mjcBodyId != -1 );

        if ( m_bodyPtr->dyntype() != eDynamicsType::STATIC )
        {
            dstPosition.x = m_mjcDataPtr->qpos[m_mjcQposAdr + 0];
            dstPosition.y = m_mjcDataPtr->qpos[m_mjcQposAdr + 1];
            dstPosition.z = m_mjcDataPtr->qpos[m_mjcQposAdr + 2];
        }
        else
        {
            dstPosition.x = m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 0];
            dstPosition.y = m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 1];
            dstPosition.z = m_mjcModelPtr->body_pos[3 * m_mjcBodyId + 2];
        }
    }

    void TMjcBodyAdapter::getRotation( TMat3& dstRotation )
    {
        assert( m_bodyPtr );
        assert( m_mjcBodyId != -1 );

        if ( m_bodyPtr->dyntype() != eDynamicsType::STATIC )
        {
            TVec4 _quat;
            _quat.w = m_mjcDataPtr->qpos[m_mjcQposAdr + 3];
            _quat.x = m_mjcDataPtr->qpos[m_mjcQposAdr + 4];
            _quat.y = m_mjcDataPtr->qpos[m_mjcQposAdr + 5];
            _quat.z = m_mjcDataPtr->qpos[m_mjcQposAdr + 6];

            dstRotation = TMat3::fromQuaternion( _quat );
        }
        else
        {
            TVec4 _quat;
            _quat.w = m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 0];
            _quat.x = m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 1];
            _quat.y = m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 2];
            _quat.z = m_mjcModelPtr->body_quat[4 * m_mjcBodyId + 3];

            dstRotation = TMat3::fromQuaternion( _quat );
        }
    }

    void TMjcBodyAdapter::getTransform( TMat4& dstTransform )
    {
        TVec3 _pos;
        TMat3 _rot;

        getPosition( _pos );
        getRotation( _rot );

        dstTransform.setPosition( _pos );
        dstTransform.setRotation( _rot );
    }

    void TMjcBodyAdapter::onResourcesCreated()
    {
        assert( m_mjcModelPtr );
        assert( m_mjcDataPtr );

        if ( m_mjcBodyId == -1 )
        {
            std::cout << "ERROR> mjc-body-adapter should have been assigned a mjc-id by now" << std::endl;
            return;
        }

        // set the start position in the model (only if non-static)
        if ( m_bodyPtr->dyntype() != eDynamicsType::STATIC )
        {
            // grab the joint-addr for the free-joint created for this body
            m_mjcJointId = m_mjcModelPtr->body_jntadr[m_mjcBodyId];
            assert( m_mjcJointId != -1 );

            // grab the qpos and qvel addresses as well to set both qpos and qvel as needed
            m_mjcQposAdr = m_mjcModelPtr->jnt_qposadr[m_mjcJointId];
            m_mjcQvelAdr = m_mjcModelPtr->jnt_dofadr[m_mjcJointId];

            // define the number of generalized coordinates for this body
            m_mjcQposNum = 7;
            m_mjcQvelNum = 6;
            assert( m_mjcQvelNum == m_mjcModelPtr->body_dofnum[m_mjcBodyId] );

            // set the starting generalized coordinates (qpos0) in the mjc-model
            auto _pos0 = m_bodyPtr->pos0();
            auto _quat0 = m_bodyPtr->quat0();

            m_mjcModelPtr->qpos0[m_mjcQposAdr + 0] = _pos0.x;
            m_mjcModelPtr->qpos0[m_mjcQposAdr + 1] = _pos0.y;
            m_mjcModelPtr->qpos0[m_mjcQposAdr + 2] = _pos0.z;

            m_mjcModelPtr->qpos0[m_mjcQposAdr + 3] = _quat0.w;
            m_mjcModelPtr->qpos0[m_mjcQposAdr + 4] = _quat0.x;
            m_mjcModelPtr->qpos0[m_mjcQposAdr + 5] = _quat0.y;
            m_mjcModelPtr->qpos0[m_mjcQposAdr + 6] = _quat0.z;
        }
        else
        {
            // no generalized coordinates for this body (if user wants to set pos, use body_pos )
            m_mjcQposNum = 0;
            m_mjcQvelNum = 0;
        }
    }

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr )
    {
        return new TMjcBodyAdapter( bodyPtr );
    }

}