
#include <adapters/mujoco_compound_body_adapter.h>

namespace tysoc {

    TMjcCompoundBodyAdapter::TMjcCompoundBodyAdapter( TCompoundBody* bodyRef )
        : TIBodyAdapter( bodyRef )
    {
        m_mjcModelRef = nullptr;
        m_mjcDataRef = nullptr;
        m_mjcfXmlResource = nullptr;
        m_mjcfXmlAssetResources = nullptr;

        /* mujoco backend identifier (assigned once simulation is created) */
        m_mjcBodyId = -1;
    }

    TMjcCompoundBodyAdapter::~TMjcCompoundBodyAdapter()
    {
        m_mjcfXmlResource = nullptr;
        m_mjcfXmlAssetResources = nullptr;
        m_mjcDataRef = nullptr;
        m_mjcModelRef = nullptr;
    }

    void TMjcCompoundBodyAdapter::build()
    {
        if ( !m_bodyRef )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::build() >>> can't construct adapter for null body" );
            return;
        }

        if ( m_bodyRef->classType() != eBodyClassType::COMPOUND_BODY )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::build() >>> this adapter is meant for compound-bodies,\
                               but we've been given a \"{0}\" for the body called \"{1}\". Use either TMjcSingleBodyAdapter\
                               or TMjcKinTreeBodyAdapter instead.", tysoc::toString( m_bodyRef->classType() ), m_bodyRef->name() );
            return;
        }
        auto _compoundBodyRef = dynamic_cast< TCompoundBody* >( m_bodyRef );

        // create the body-resource where to place the body components (body-data, collision-data, joint-data)
        m_mjcfXmlResource = new mjcf::GenericElement( "body" );
        // create the assets-resource where to place the mesh-assets (for colliders if needed)
        m_mjcfXmlAssetResources = new mjcf::GenericElement( "asset" );

        m_mjcfXmlResource->setAttributeString( "name", m_bodyRef->name() );
        m_mjcfXmlResource->setAttributeVec3( "pos", m_bodyRef->localPos0() );
        m_mjcfXmlResource->setAttributeVec4( "quat", mujoco::quat2MjcfQuat( m_bodyRef->localQuat0() ) );

        /* Create joint-resource if applicable ****************************************************/
        auto _jointRef = _compoundBodyRef->joint();
        if ( _jointRef && _jointRef->adapter() )
        {
            // request joint-adapter to create mjcf resources for the required backend (all 
            // compound-bodies have a joint associated, even for root cases, either free or fixed)
            auto _jointAdapterRef = dynamic_cast< TMjcCompoundJointAdapter* >( _jointRef->adapter() );
            _jointAdapterRef->build();

            if ( _jointAdapterRef->mjcfResource() )
                m_mjcfXmlResource->children.push_back( _jointAdapterRef->mjcfResource() );
        }
        /******************************************************************************************/

        /* Create collision-resource if applicable ************************************************/
        auto _colliderRef = _compoundBodyRef->collision();
        if ( _colliderRef && _colliderRef->adapter() )
        {
            // request collider adapter to create mjcf resources for the required backend
            auto _colliderAdapterRef = dynamic_cast< TMjcCollisionAdapter* >( _colliderRef->adapter() );
            _colliderAdapterRef->build();

            if ( _colliderAdapterRef->mjcfResource() )
                m_mjcfXmlResource->children.push_back( _colliderAdapterRef->mjcfResource() );

            if ( _colliderAdapterRef->mjcfAssetResource() )
                m_mjcfXmlAssetResources->children.push_back( _colliderAdapterRef->mjcfAssetResource() );
        }
        /******************************************************************************************/
    }

    void TMjcCompoundBodyAdapter::reset()
    {
        if ( !m_bodyRef )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::reset() >>> adapter has no valid compound-body\
                               reference to use (it's nullptr)." );
            return;
        }

        if ( m_mjcBodyId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::reset() >>> adapter for body \"{0}\" hasn't been\
                               linked to backend, as body-id is -1 (it should have a valid identifier for\
                               this body in the backend simulation", m_bodyRef->name() );
            return;
        }

        if ( m_bodyRef->classType() != eBodyClassType::COMPOUND_BODY )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::reset() >>> this adapter is meant for compound-bodies,\
                               but we've been given a \"{0}\" for the body called \"{1}\". Use either TMjcSingleBodyAdapter\
                               or TMjcKinTreeBodyAdapter instead.", tysoc::toString( m_bodyRef->classType() ), m_bodyRef->name() );
            return;
        }
        auto _compoundBodyRef = dynamic_cast< TCompoundBody* >( m_bodyRef );
        auto _compoundRef = _compoundBodyRef->compound();
        assert( _compoundRef );

        if ( _compoundBodyRef->isRoot() )
        {
            auto _worldTf = _compoundBodyRef->compound()->tf0() * _compoundBodyRef->localTf0();
            if ( _compoundRef->dyntype() != eDynamicsType::STATIC )
                _updateWorldTransform_freeRoot( _worldTf );
            else
                _updateWorldTransform_fixedRoot( _worldTf );
        }
    }

    void TMjcCompoundBodyAdapter::preStep()
    {
        // nothing required here (world-pose is grabbed with getters below)
    }

    void TMjcCompoundBodyAdapter::postStep()
    {
        // nothing required here (world-pose is grabbed with getters below)
    }

    void TMjcCompoundBodyAdapter::setPosition( const TVec3& position )
    {
        // update the whole transform accordingly
        _updateTransformInBackend();
    }

    void TMjcCompoundBodyAdapter::setRotation( const TMat3& rotation )
    {
        // update the whole transform accordingly
        _updateTransformInBackend();
    }

    void TMjcCompoundBodyAdapter::setTransform( const TMat4& transform )
    {
        // update the whole transform accordingly
        _updateTransformInBackend();
    }

    void TMjcCompoundBodyAdapter::getPosition( TVec3& dstPosition )
    {
        if ( m_mjcBodyId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::getPosition() >>> body \"{0}\" not linked \
                               to any body in the simulation", m_bodyRef->name() );
            return;
        }

        dstPosition.x = m_mjcDataRef->xpos[3 * m_mjcBodyId + 0];
        dstPosition.y = m_mjcDataRef->xpos[3 * m_mjcBodyId + 1];
        dstPosition.z = m_mjcDataRef->xpos[3 * m_mjcBodyId + 2];
    }

    void TMjcCompoundBodyAdapter::getRotation( TMat3& dstRotation )
    {
        if ( m_mjcBodyId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::getRotation() >>> body \"{0}\" not linked \
                               to any body in the simulation", m_bodyRef->name() );
            return;
        }

        dstRotation = TMat3::fromQuaternion( { (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 1],    // q-x
                                               (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 2],    // q-y
                                               (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 3],    // q-z
                                               (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 0] } );// q-w
    }

    void TMjcCompoundBodyAdapter::getTransform( TMat4& dstTransform )
    {
        if ( m_mjcBodyId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::getTransform() >>> body \"{0}\" not linked \
                               to any body in the simulation", m_bodyRef->name() );
            return;
        }

        dstTransform.setPosition( { (TScalar) m_mjcDataRef->xpos[3 * m_mjcBodyId + 0],
                                    (TScalar) m_mjcDataRef->xpos[3 * m_mjcBodyId + 1],
                                    (TScalar) m_mjcDataRef->xpos[3 * m_mjcBodyId + 2] } );

        dstTransform.setRotation( { (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 1],
                                    (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 2], 
                                    (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 3], 
                                    (TScalar) m_mjcDataRef->xquat[4 * m_mjcBodyId + 0] } );
    }

    void TMjcCompoundBodyAdapter::setLocalPosition( const TVec3& position )
    {
        if ( m_mjcBodyId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::setLocalPosition() >>> body \"{0}\" not linked \
                               to any body in the simulation", m_bodyRef->name() );
            return;
        }

        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = position.x;
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = position.y;
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = position.z;
    }

    void TMjcCompoundBodyAdapter::setLocalRotation( const TMat3& rotation )
    {
        if ( m_mjcBodyId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::setLocalRotation() >>> body \"{0}\" not linked \
                               to any body in the simulation", m_bodyRef->name() );
            return;
        }

        auto _localQuat = TMat3::toQuaternion( rotation );
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = _localQuat.w;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = _localQuat.x;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = _localQuat.y;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = _localQuat.z;
    }

    void TMjcCompoundBodyAdapter::setLocalTransform( const TMat4& transform )
    {
        if ( m_mjcBodyId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::setLocalTransform() >>> body \"{0}\" not linked \
                               to any body in the simulation", m_bodyRef->name() );
            return;
        }

        auto _localPos = transform.getPosition();
        auto _localQuat = transform.getRotQuaternion();

        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = _localPos.x;
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = _localPos.y;
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = _localPos.z;

        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = _localQuat.w;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = _localQuat.x;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = _localQuat.y;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = _localQuat.z;
    }

    void TMjcCompoundBodyAdapter::getLocalPosition( TVec3& dstPosition )
    {
        // nothing do to here, local-position is computed from world-transforms in caller
    }

    void TMjcCompoundBodyAdapter::getLocalRotation( TMat3& dstRotation )
    {
        // nothing do to here, local-rotation is computed from world-transforms in caller
    }

    void TMjcCompoundBodyAdapter::getLocalTransform( TMat4& dstTransform )
    {
        // nothing do to here, local-transform is computed from world-transforms in caller
    }

    void TMjcCompoundBodyAdapter::onResourcesCreated()
    {
        // grab mjc-body id associated with the body in simulation
        m_mjcBodyId = mj_name2id( m_mjcModelRef, mjOBJ_BODY, m_bodyRef->name().c_str() );
        // upcast to the actual compound-body (we need those resources)
        auto _compoundBodyRef = dynamic_cast< TCompoundBody* >( m_bodyRef );

        // notify the collision-component that resources are ready
        auto _colliderRef = _compoundBodyRef->collision();
        if ( _colliderRef && _colliderRef->adapter() )
        {
            auto _colliderAdapterRef = dynamic_cast< TMjcCollisionAdapter* >( _colliderRef->adapter() );
            if ( _colliderAdapterRef )
            {
                _colliderAdapterRef->setMjcModelRef( m_mjcModelRef );
                _colliderAdapterRef->setMjcDataRef( m_mjcDataRef );
                _colliderAdapterRef->onResourcesCreated();
            }
        }

        // notify the joint-component that resources are ready
        auto _jointRef = _compoundBodyRef->joint();
        if ( _jointRef && _jointRef->adapter() )
        {
            auto _jointAdapterRef = dynamic_cast< TMjcCompoundJointAdapter* >( _jointRef->adapter() );
            if ( _jointAdapterRef )
            {
                _jointAdapterRef->setMjcModelRef( m_mjcModelRef );
                _jointAdapterRef->setMjcDataRef( m_mjcDataRef );
                _jointAdapterRef->onResourcesCreated();
            }
        }
    }

    void TMjcCompoundBodyAdapter::_updateTransformInBackend()
    {
        if ( m_mjcBodyId == -1 )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::_updateTransformInBackend() >>> body \"{0}\"\
                               is not linked to any body in the mujoco backend", m_bodyRef->name() );
            return;
        }

        auto _compoundBodyRef = dynamic_cast< TCompoundBody* >( m_bodyRef );
        assert( _compoundBodyRef );
        auto _compoundRef = _compoundBodyRef->compound();
        assert( _compoundRef );
        /* for root bodies, we either move using the available generalized coordinates (free root-body), 
           or using the body_pos|body_quat directly in the mjModel struct (fixed root-body) */
        if ( _compoundBodyRef->isRoot() )
        {
            // move the whole structure to the body's frame in world-space (world-tf was updated by caller)
            if ( _compoundRef->dyntype() != eDynamicsType::STATIC )
                _updateWorldTransform_freeRoot( _compoundBodyRef->tf() );
            else
                _updateWorldTransform_fixedRoot( _compoundBodyRef->tf() );
        }
        /* For non-root bodies, the relative transform w.r.t. parent body has changed, so we need to 
           update our local-transform (pos|quat) in the backend itself */
        else
        {
            _updateRelativeTransform( _compoundBodyRef->localTf() );
        }
    }

    void TMjcCompoundBodyAdapter::_updateWorldTransform_freeRoot( const TMat4& worldTransform )
    {
        // A free root-body has to use its own free-joint to position itself
        auto _compoundBodyRef = dynamic_cast< TCompoundBody* >( m_bodyRef );
        auto _jointRef = _compoundBodyRef->joint();
        if ( _jointRef->type() != eJointType::FREE )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundBodyAdapter::_updateWorldTransform_freeRoot() >>> body \
                               \"{0}\" has no free joint. Joint named \"{1}\" has type \"{2}\"", 
                               _compoundBodyRef->name(), _jointRef->name() );
            return;
        }

        auto _worldPos = worldTransform.getPosition();
        auto _worldQuat = worldTransform.getRotQuaternion();

        // construct buffer to set generalized coordinates (qvel dofs are set to zero to avoid speed accummulation)
        std::vector< TScalar > _qpos = { _worldPos.x, _worldPos.y, _worldPos.z,
                                         _worldQuat.w, _worldQuat.x, _worldQuat.y, _worldQuat.z };
        std::vector< TScalar > _qvel = { 0.0f, 0.0f, 0.0f,
                                         0.0f, 0.0f, 0.0f };
        _jointRef->setQpos( _qpos );
        _jointRef->setQvel( _qvel );
    }

    void TMjcCompoundBodyAdapter::_updateWorldTransform_fixedRoot( const TMat4& worldTransform )
    {
        auto _worldPos = worldTransform.getPosition();
        auto _worldQuat = worldTransform.getRotQuaternion();

        // A static root-body has to use the model body_pos itself to update its position
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = _worldPos.x;
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = _worldPos.y;
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = _worldPos.z;

        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = _worldQuat.w;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = _worldQuat.x;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = _worldQuat.y;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = _worldQuat.z;
    }

    void TMjcCompoundBodyAdapter::_updateRelativeTransform( const TMat4& localTransform )
    {
        auto _localPos = localTransform.getPosition();
        auto _localQuat = localTransform.getRotQuaternion();

        // Update local-transform information into the appropriate section in the mjModel
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 0] = _localPos.x;
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 1] = _localPos.y;
        m_mjcModelRef->body_pos[3 * m_mjcBodyId + 2] = _localPos.z;

        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 0] = _localQuat.w;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 1] = _localQuat.x;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 2] = _localQuat.y;
        m_mjcModelRef->body_quat[4 * m_mjcBodyId + 3] = _localQuat.z;
    }

    extern "C" TIBodyAdapter* simulation_createCompoundBodyAdapter( TCompoundBody* bodyRef )
    {
        return new TMjcCompoundBodyAdapter( bodyRef );
    }

}