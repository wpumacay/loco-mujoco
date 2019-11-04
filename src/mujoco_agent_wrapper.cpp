
#include <mujoco_agent_wrapper.h>

namespace tysoc {
namespace mujoco {

    /***********************************************************************************************
    *                                                                                              *
    *                                     Helper-Functions                                         *
    *                                                                                              *
    ***********************************************************************************************/

    std::string enumJointToMjcType( const eJointType& type )
    {
        if ( type == eJointType::FIXED ) return ""; // fixed joints are just skipped in mjcf format
        if ( type == eJointType::REVOLUTE ) return "hinge";
        if ( type == eJointType::PRISMATIC ) return "slide";
        if ( type == eJointType::SPHERICAL ) return "ball";
        if ( type == eJointType::PLANAR ) return ""; // planar joints are not supported, so skip for now
        if ( type == eJointType::FREE ) return "free";

        std::cout << "WARNING> invalid eJointType enum given" << std::endl;

        return "";
    }

    std::string enumShapeToMjcType( const eShapeType& type )
    {
        if ( type == eShapeType::BOX ) return "box";
        if ( type == eShapeType::PLANE ) return "plane";
        if ( type == eShapeType::SPHERE ) return "sphere";
        if ( type == eShapeType::CYLINDER ) return "cylinder";
        if ( type == eShapeType::CAPSULE ) return "capsule";
        if ( type == eShapeType::ELLIPSOID ) return "ellipsoid";
        if ( type == eShapeType::MESH ) return "mesh";
        if ( type == eShapeType::HFIELD ) return "hfield"; // @todo: move to mujoco_common

        std::cout << "WARNING> invalid eShapeType enum given" << std::endl;

        return "";
    }

    std::string enumActuatorToMjcType( const eActuatorType& type )
    {
        if ( type == eActuatorType::TORQUE ) return "motor";

        std::cout << "WARNING> unsupported type of actuator: " << tysoc::toString( type ) << std::endl;

        return "";
    }

    /***********************************************************************************************
    *                                                                                              *
    *                                    Mujoco Joint-Adapter                                      *
    *                                                                                              *
    ***********************************************************************************************/

    TMjcJointWrapper::TMjcJointWrapper( mjModel* mjcModelPtr,
                                        mjData* mjcDataPtr, 
                                        TKinTreeJoint* jointPtr )
    {
        m_kinTreeJointPtr = jointPtr;
        m_mjcModelPtr = mjcModelPtr;
        m_mjcDataPtr = mjcDataPtr;

        // grab some backend-info for this joint
        m_id = mj_name2id( m_mjcModelPtr, mjOBJ_JOINT, jointPtr->name.c_str() );
        if ( m_id == -1 && jointPtr->data.type != eJointType::FIXED )
        {
            std::cout << "ERROR> joint (" << jointPtr->name << ") not linked to joint" << std::endl;
            return;
        }

        m_nqpos     = jointPtr->data.nqpos;
        m_nqvel     = jointPtr->data.nqvel;
        m_qposAdr   = m_mjcModelPtr->jnt_qposadr[m_id];
        m_qvelAdr   = m_mjcModelPtr->jnt_dofadr[m_id];
    }

    void TMjcJointWrapper::setQpos( const std::vector< TScalar >& qpos )
    {
        for ( int i = 0; i < qpos.size(); i++ )
            m_mjcDataPtr->qpos[m_qposAdr + i] = qpos[i];
    }

    void TMjcJointWrapper::setQvel( const std::vector< TScalar >& qvel )
    {
        for ( int i = 0; i < qvel.size(); i++ )
            m_mjcDataPtr->qvel[m_qvelAdr + i] = qvel[i];
    }

    bool TMjcJointWrapper::isRootJoint()
    {
        if ( !m_kinTreeJointPtr->parentBodyPtr )
        {
            std::cout << "ERROR> this joint should have a parent body" << std::endl;
            return false;
        }

        if ( m_kinTreeJointPtr->parentBodyPtr->parentBodyPtr )
            return false;

        return true;
    }

    /***********************************************************************************************
    *                                                                                              *
    *                                 Mujoco Agent-Adapter                                         *
    *                                                                                              *
    ***********************************************************************************************/

    TMjcKinTreeAgentWrapper::TMjcKinTreeAgentWrapper( TAgent* agentPtr )
        : TAgentWrapper( agentPtr )
    {
        m_mjcModelPtr       = nullptr;
        m_mjcDataPtr        = nullptr;
        m_mjcfXmlResource   = nullptr;
        m_hasMadeSummary    = false;
    }

    TMjcKinTreeAgentWrapper::~TMjcKinTreeAgentWrapper()
    {
        // @wip: build functionality
        if ( m_mjcfXmlResource )
            delete m_mjcfXmlResource;

        m_mjcModelPtr   = nullptr;
        m_mjcDataPtr    = nullptr;
        m_mjcfXmlResource = nullptr;
    }

    void TMjcKinTreeAgentWrapper::build()
    {
        if ( !m_agentPtr )
            return;

        m_mjcfXmlResource = new mjcf::GenericElement( "mujoco" );
        m_mjcfXmlResource->setAttributeString( "model", name() );

        m_mjcfXmlAssetResources = new mjcf::GenericElement( "asset" );
        m_mjcfXmlAssetResources->setAttributeString( "model", name() );

        /* create a root element to place agent resources (geoms, bodies, joints, ...) */
        auto _worldBody = new mjcf::GenericElement( "worldbody" );
        m_mjcfXmlResource->children.push_back( _worldBody );
        
        // Collect bodies xml data into worldbody element
        _worldBody->children.push_back( _createMjcResourcesFromBodyNode( m_agentPtr->getRootBody() ) );

        // Create the default sensors (for joints and bodies)
        _createMjcSensorsFromKinTree();
        // Collect all actuators and replace the names accordingly
        _createMjcActuatorsFromKinTree();
        // Collect all contact eclusions
        _createMjcExclusionContactsFromKinTree();
        // Collect all assets data into the model element
        _createMjcAssetsFromKinTree();

        // Collect extra specifics depending of the type of data being parsed
        if ( m_agentPtr->format() == eModelFormat::MJCF )
            _configureFormatMjcf();
        else if ( m_agentPtr->format() == eModelFormat::URDF )
            _configureFormatUrdf();
        else if ( m_agentPtr->format() == eModelFormat::RLSIM )
            _configureFormatRlsim();
    }

    void TMjcKinTreeAgentWrapper::setMjcModel( mjModel* mjcModelPtr )
    {
        m_mjcModelPtr = mjcModelPtr;
    }

    void TMjcKinTreeAgentWrapper::setMjcData( mjData* mjcDataPtr )
    {
        m_mjcDataPtr = mjcDataPtr;
    }

    void TMjcKinTreeAgentWrapper::finishedCreatingResources()
    {
        if ( !m_agentPtr )
            return;

        // collect some low-level properties of the bodies (ids and more stuff)
        for ( auto _kinBody : m_agentPtr->bodies )
            _cacheBodyProperties( _kinBody );

        // collect some low-level properties of the joints (ids and more stuff)
        for ( auto _kinJoint : m_agentPtr->joints )
            _cacheJointProperties( _kinJoint );
    }

    void TMjcKinTreeAgentWrapper::_initializeInternal()
    {
        // @todo: remove this method from adapter specification
    }

    void TMjcKinTreeAgentWrapper::_resetInternal()
    {
        if ( !m_agentPtr )
            return;

        // set the qpos values set by the user (left in the abstract agent)
        for ( auto& _jointAdapter : m_jointWrappers )
        {
            // joint being wrapped
            auto _kinJoint = _jointAdapter.jointPtr();

            // buffer for the q-values
            std::vector< TScalar > _qposs;
            std::vector< TScalar > _qvels;

            if ( _jointAdapter.isRootJoint() )
            {
                if ( _kinJoint->data.type == eJointType::FREE )
                {
                    auto _position = m_agentPtr->getStartPosition();
                    auto _rotation = TMat3::toQuaternion( TMat3::fromEuler( m_agentPtr->getStartRotation() ) );

                    // use the start position ( x - y - z - qw - qx - qy - qz )
                    _qposs = { _position.x, _position.y, _position.z,
                               _rotation.w, _rotation.x, _rotation.y, _rotation.z };

                    // and no initial velocities
                    _qvels = { 0., 0., 0., 0., 0., 0. };
                }
                else if ( _kinJoint->data.type == eJointType::PRISMATIC )
                {
                    auto _position = m_agentPtr->getStartPosition();

                    if ( _kinJoint->data.axis == TVec3( 1., 0., 0. ) )
                    {
                        _qposs = { _position.x };
                        _qvels = { 0. };
                    }
                    else if ( _kinJoint->data.axis == TVec3( 0., 1., 0. ) )
                    {
                        _qposs = { _position.y };
                        _qvels = { 0. };
                    }
                    else if ( _kinJoint->data.axis == TVec3( 0., 0., 1. ) )
                    {
                        _qposs = { _position.z };
                        _qvels = { 0. };
                    }
                }
                else if ( _kinJoint->data.type == eJointType::REVOLUTE )
                {
                    auto _rotation = m_agentPtr->getStartRotation();

                    if ( _kinJoint->data.axis == TVec3( 1., 0., 0. ) )
                    {
                        _qposs = { _rotation.x };
                        _qvels = { 0. };
                    }
                    else if ( _kinJoint->data.axis == TVec3( 0., 1., 0. ) )
                    {
                        _qposs = { _rotation.y };
                        _qvels = { 0. };
                    }
                    else if ( _kinJoint->data.axis == TVec3( 0., 0., 1. ) )
                    {
                        _qposs = { _rotation.z };
                        _qvels = { 0. };
                    }
                }
            }
            else
            {
                if ( _kinJoint->data.type == eJointType::SPHERICAL )
                {
                    _qposs.push_back( _kinJoint->qpos0[3] );
                    _qposs.push_back( _kinJoint->qpos0[0] );
                    _qposs.push_back( _kinJoint->qpos0[1] );
                    _qposs.push_back( _kinJoint->qpos0[2] );
                }
                else
                {
                    // collect qpos from kintree-joint
                    for ( int j = 0; j < _kinJoint->data.nqpos; j++ )
                        _qposs.push_back( _kinJoint->qpos0[j] );
                }

                // set qvels to zeros
                for ( int j = 0; j < _kinJoint->data.nqvel; j++ )
                    _qvels.push_back( 0.0f );
            }

            // and set the qposs and qvels into the backend through the wrapper
            _jointAdapter.setQpos( _qposs );
            _jointAdapter.setQvel( _qvels );

        }

        // reset the internal high-level resources of the kintree
        m_agentPtr->reset();

        m_hasMadeSummary = false;
    }

    void TMjcKinTreeAgentWrapper::_preStepInternal()
    {
        if ( !m_agentPtr )
            return;

        //// for ( auto _actuator : m_agentPtr->actuators )
        ////     utils::setActuatorCtrl( m_mjcModelPtr, m_mjcDataPtr, _actuator->name, _actuator->ctrlValue );

        if ( !m_hasMadeSummary )
            _collectSummary();
    }

    void TMjcKinTreeAgentWrapper::_postStepInternal()
    {
        if ( !m_agentPtr )
            return;

        for ( auto _kinBody : m_agentPtr->bodies )
        {
            _kinBody->worldTransform.setPosition( utils::getBodyPosition( m_mjcModelPtr, 
                                                                          m_mjcDataPtr, 
                                                                          _kinBody->name ) );

            _kinBody->worldTransform.setRotation( utils::getBodyOrientation( m_mjcModelPtr, 
                                                                             m_mjcDataPtr, 
                                                                             _kinBody->name ) );
        }

        // @todo: bring back code from previous version used to handle sensor readings
    }

    mjcf::GenericElement* TMjcKinTreeAgentWrapper::_createMjcResourcesFromBodyNode( TKinTreeBody* kinBody )
    {
        /* create resources for the body */
        auto _bodyElmPtr = new mjcf::GenericElement( "body" );
        _bodyElmPtr->setAttributeString( "name", kinBody->name );
        if ( !kinBody->parentBodyPtr )
        {
            // root should use its worldTransform directly (transform root is defined by user, not model)
            _bodyElmPtr->setAttributeVec3( "pos", kinBody->worldTransform.getPosition() );
            auto _quat = TMat3::toQuaternion( kinBody->worldTransform.getRotation() );
            _bodyElmPtr->setAttributeVec4( "quat", { _quat.w, _quat.x, _quat.y, _quat.z } );
        }
        else
        {
            // non-root bodies use its relative transform to the parent body
            _bodyElmPtr->setAttributeVec3( "pos", kinBody->localTransformZero.getPosition() );
            auto _quat = TMat3::toQuaternion( kinBody->localTransformZero.getRotation() );
            _bodyElmPtr->setAttributeVec4( "quat", { _quat.w, _quat.x, _quat.y, _quat.z } );
        }

        /* create mjcf resource for joints, and add it to the body resource */
        for ( auto _kinJoint : kinBody->joints )
        {
            auto _jointElmPtr = _createMjcResourcesFromJointNode( _kinJoint );
            if ( _jointElmPtr )
                _bodyElmPtr->children.push_back( _jointElmPtr );
        }

        /* create mjcf resource from colliders, and add it to the body resource */
        for ( auto _kinCollision : kinBody->collisions )
        {
            auto _geomElmPtr = _createMjcResourcesFromCollisionNode( _kinCollision );
            if ( _geomElmPtr )
                _bodyElmPtr->children.push_back( _geomElmPtr );
        }

        /* create mjcf resource for inertial properties, only if valid inertia (non-zero mass) */
        if ( kinBody->inertialData.mass < TYSOC_FLOAT_EPSILON )
        {
            auto _inertiaElmPtr = _createMjcResourcesFromInertialNode( kinBody->inertialData );
            _bodyElmPtr->children.push_back( _inertiaElmPtr );
        }

        /* recurse to children */
        for ( auto _childBody : kinBody->children )
            _bodyElmPtr->children.push_back( _createMjcResourcesFromBodyNode( _childBody ) );

        return _bodyElmPtr;
    }

    mjcf::GenericElement* TMjcKinTreeAgentWrapper::_createMjcResourcesFromJointNode( TKinTreeJoint* kinJoint )
    {
        // for mujoco it it's like a non-existent joint in xml
        if ( kinJoint->data.type == eJointType::FIXED )
            return nullptr;

        if ( kinJoint->data.type == eJointType::PLANAR )
        {
            std::cout << "WARNING> joint with type: " << tysoc::toString( kinJoint->data.type ) << " "
                      << "not supported for mujoco backend" << std::endl;
            return nullptr;
        }

        auto _jointElmPtr = new mjcf::GenericElement( "joint" );

        _jointElmPtr->setAttributeString( "name", kinJoint->name );
        _jointElmPtr->setAttributeString( "type", mujoco::enumJointToMjcType( kinJoint->data.type ) );

        // free joints don't require any more information
        if ( kinJoint->data.type == eJointType::FREE )
            return _jointElmPtr;

        _jointElmPtr->setAttributeVec3( "pos", kinJoint->data.localTransform.getPosition() );
        _jointElmPtr->setAttributeVec3( "axis", kinJoint->data.localTransform.getRotation() * kinJoint->data.axis );
        _jointElmPtr->setAttributeString( "limited", ( kinJoint->data.limits.x < kinJoint->data.limits.y ) ? "true" : "false" );
        if ( kinJoint->data.limits.x < kinJoint->data.limits.y )
        {
            if ( kinJoint->data.type == eJointType::SPHERICAL )
                _jointElmPtr->setAttributeVec2( "range", { 0, rad2degrees( kinJoint->data.limits.y ) } );
            else
                _jointElmPtr->setAttributeVec2( "range", { rad2degrees( kinJoint->data.limits.x ), 
                                                           rad2degrees( kinJoint->data.limits.y ) } );
        }

        if ( kinJoint->data.stiffness != 0.0f )
            _jointElmPtr->setAttributeFloat( "stiffness", kinJoint->data.stiffness );
        if ( kinJoint->data.armature != 0.0f )
            _jointElmPtr->setAttributeFloat( "armature", kinJoint->data.armature );
        if ( kinJoint->data.damping != 0.0f )
            _jointElmPtr->setAttributeFloat( "damping", kinJoint->data.damping );
        if ( kinJoint->data.ref != 0.0f )
            _jointElmPtr->setAttributeFloat( "ref", kinJoint->data.ref );

        return _jointElmPtr;
    }

    mjcf::GenericElement* TMjcKinTreeAgentWrapper::_createMjcResourcesFromCollisionNode( TKinTreeCollision* kinCollision )
    {
        auto _geomElmPtr = new mjcf::GenericElement( "geom" );

        _geomElmPtr->setAttributeString( "name", kinCollision->name );
        _geomElmPtr->setAttributeString( "type", mujoco::enumShapeToMjcType( kinCollision->data.type ) );

        _geomElmPtr->setAttributeVec3( "pos", kinCollision->data.localTransform.getPosition() );
        auto _gquat = TMat3::toQuaternion( kinCollision->data.localTransform.getRotation() );
        _geomElmPtr->setAttributeVec4( "quat", { _gquat.w, _gquat.x, _gquat.y, _gquat.z } );

        if ( kinCollision->data.type != eShapeType::MESH && kinCollision->data.type != eShapeType::HFIELD )
            _geomElmPtr->setAttributeVec3( "size", _extractMjcSizeFromStandardSize( kinCollision->data ) );
        else if ( kinCollision->data.type == eShapeType::MESH )
            _geomElmPtr->setAttributeString( "mesh", tysoc::getFilenameNoExtensionFromFilePath( kinCollision->data.filename ) );

        if ( kinCollision->data.collisionGroup != -1 )
            _geomElmPtr->setAttributeInt( "contype", kinCollision->data.collisionGroup );

        if ( kinCollision->data.collisionMask != -1 )
            _geomElmPtr->setAttributeInt( "conaffinity", kinCollision->data.collisionMask );

        _geomElmPtr->setAttributeVec3( "friction", kinCollision->data.friction );
        _geomElmPtr->setAttributeFloat( "density", kinCollision->data.density );
        _geomElmPtr->setAttributeVec4( "rgba", TYSOC_DEFAULT_RGBA_COLOR );

        return _geomElmPtr;
    }

    mjcf::GenericElement* TMjcKinTreeAgentWrapper::_createMjcResourcesFromInertialNode( const TInertialData& inertia )
    {
        auto _inertiaElmPtr = new mjcf::GenericElement( "inertial" );
        _inertiaElmPtr->setAttributeFloat( "mass", inertia.mass );

        if ( inertia.ixx > TYSOC_FLOAT_EPSILON || inertia.iyy > TYSOC_FLOAT_EPSILON ||
             inertia.izz > TYSOC_FLOAT_EPSILON || inertia.ixy > TYSOC_FLOAT_EPSILON ||
             inertia.ixz > TYSOC_FLOAT_EPSILON || inertia.iyz > TYSOC_FLOAT_EPSILON )
        {
            if ( inertia.ixy < TYSOC_FLOAT_EPSILON &&
                 inertia.ixz < TYSOC_FLOAT_EPSILON &&
                 inertia.iyz < TYSOC_FLOAT_EPSILON )
            {
                // diagonal inertia matrix
                _inertiaElmPtr->setAttributeVec3( "diaginertia", 
                                                  { inertia.ixx, 
                                                    inertia.iyy, 
                                                    inertia.izz } );
            }
            else
            {
                // full inertia matrix
                _inertiaElmPtr->setAttributeArrayFloat( "fullinertia",
                                                        { 6, 
                                                          { inertia.ixx,
                                                            inertia.iyy,
                                                            inertia.izz,
                                                            inertia.ixy,
                                                            inertia.ixz,
                                                            inertia.iyz } } );
            }
        }

        _inertiaElmPtr->setAttributeVec3( "pos", inertia.localTransform.getPosition() );
        auto _iquat = TMat3::toQuaternion( inertia.localTransform.getRotation() );
        _inertiaElmPtr->setAttributeVec4( "quat", { _iquat.w, _iquat.x, _iquat.y, _iquat.z } );

        return _inertiaElmPtr;
    }


    void TMjcKinTreeAgentWrapper::_createMjcAssetsFromKinTree()
    {
        for ( auto _kinCollision : m_agentPtr->collisions )
        {
            if ( _kinCollision->data.type != eShapeType::MESH )
                continue;

            if ( _kinCollision->data.filename == "" )
                continue;

            auto _xmlAssetResource = new mjcf::GenericElement( "mesh" );
            _xmlAssetResource->setAttributeString( "name", tysoc::getFilenameNoExtensionFromFilePath( _kinCollision->data.filename ) );
            _xmlAssetResource->setAttributeString( "file", _kinCollision->data.filename );
            _xmlAssetResource->setAttributeVec3( "scale", _kinCollision->data.size );

            m_mjcfXmlAssetResources->children.push_back( _xmlAssetResource );
        }
    }

    void TMjcKinTreeAgentWrapper::_createMjcSensorsFromKinTree()
    {
        // @todo: bring back code from previous version, and accommodate new sensor types
    }

    void TMjcKinTreeAgentWrapper::_createMjcActuatorsFromKinTree()
    {
        // @todo: bring back code from previous version, and accommodate new actuator types
    }

    void TMjcKinTreeAgentWrapper::_createMjcExclusionContactsFromKinTree()
    {
        if ( !m_agentPtr )
            return;

        if ( m_agentPtr->exclusionContacts.size() < 1 )
            return;

        auto _exclusionContactsElm = new mjcf::GenericElement( "contact" );
        m_mjcfXmlResource->children.push_back( _exclusionContactsElm );

        for ( auto& _kinExclusionPair : m_agentPtr->exclusionContacts )
        {
            auto _exclusionContactPairElm = new mjcf::GenericElement( "exclude" );
            _exclusionContactPairElm->setAttributeString( "body1", _kinExclusionPair.first );
            _exclusionContactPairElm->setAttributeString( "body2", _kinExclusionPair.second );

            _exclusionContactsElm->children.push_back( _exclusionContactPairElm );
        }
    }

    void TMjcKinTreeAgentWrapper::_configureFormatMjcf()
    {
        // Collect all contacts in original model
        auto _srcContactsElmPtr = mjcf::findFirstChildByType( m_mjcfModelTemplatePtr, "contact" );
        if ( !_srcContactsElmPtr )
            return;

        // create the target element where we are going to place our contacts
        auto _targetContactsElmPtr = new mjcf::GenericElement( "contact" );
        m_mjcfXmlResource->children.push_back( _targetContactsElmPtr );

        // now place the original contacts inside the target contacts element
        for ( auto _srcContact : _srcContactsElmPtr->children )
            _targetContactsElmPtr->children.push_back( _srcContact );
    }

    void TMjcKinTreeAgentWrapper::_configureFormatUrdf()
    {
        if ( !m_agentPtr->getRootBody() )
            return;

        // Check if the root has a joint that fixes it to the world
        bool _isRootFixed = false;
        auto _rootJoints = m_agentPtr->getRootBody()->joints;
        for ( auto _rootJoint : _rootJoints )
        {
            _isRootFixed = ( _rootJoint->data.type == eJointType::FREE );
            if ( _isRootFixed )
                break;
        }

        // Create a free joint for the root body
        if ( !_isRootFixed )
        {
            // create the freejoint element
            auto _freeJointElmPtr = new mjcf::GenericElement( "joint" );
            // compute the appropiate unique-name
            auto _freeJointName = urdf::computeUrdfName( "joint", "free", name() );
            _freeJointElmPtr->setAttributeString( "name", _freeJointName );
            // set the type to free to give 6dof to the agent at its root body
            _freeJointElmPtr->setAttributeString( "type", "free" );
            // get the appropiate body to add this joint to
            auto _rootBodyElmPtr = mjcf::findFirstChildByType( m_mjcfXmlResource, "body" );
            // and add it to the rootbody
            _rootBodyElmPtr->children.push_back( _freeJointElmPtr );
            
            // create the kin-joint required for this free-joint
            auto _freeJoint = new TKinTreeJoint( eJointType::FREE );
            _freeJoint->name = _freeJointName;
            _freeJoint->parentBodyPtr = m_agentPtr->getRootBody();

            m_agentPtr->getRootBody()->joints.push_back( _freeJoint );
            m_agentPtr->joints.push_back( _freeJoint );
        }
    }

    void TMjcKinTreeAgentWrapper::_configureFormatRlsim()
    {
        // nothing to do for now
    }

    void TMjcKinTreeAgentWrapper::_cacheBodyProperties( TKinTreeBody* kinTreeBody )
    {
        // @todo: add functionality for adapters of bodies
    }

    void TMjcKinTreeAgentWrapper::_cacheJointProperties( TKinTreeJoint* kinTreeJoint )
    {
        m_jointWrappers.push_back( TMjcJointWrapper( m_mjcModelPtr, m_mjcDataPtr, kinTreeJoint ) );
    }

    TVec3 TMjcKinTreeAgentWrapper::_extractMjcSizeFromStandardSize( const TShapeData& data )
    {
        if ( data.type == eShapeType::PLANE ) return { data.size.x, data.size.y, data.size.z };
        if ( data.type == eShapeType::SPHERE ) return { data.size.x, data.size.y, data.size.z };
        if ( data.type == eShapeType::CAPSULE ) return { data.size.x, 0.5f * data.size.y, data.size.z };
        if ( data.type == eShapeType::CYLINDER ) return { data.size.x, 0.5f * data.size.y, data.size.z };
        if ( data.type == eShapeType::BOX ) return { 0.5f * data.size.x, 0.5f * data.size.y, 0.5f * data.size.z };
        if ( data.type == eShapeType::MESH ) return { data.size.x, data.size.y, data.size.z };
        if ( data.type == eShapeType::HFIELD ) return { data.size.x, data.size.y, data.size.z };

        return data.size;
    }

    void TMjcKinTreeAgentWrapper::_collectSummary()
    {
        // collect this kind of summary only once (per restart)
        m_hasMadeSummary = true;

        /* Generate summary information *******************************************/
        TGenericParams& _summary = m_agentPtr->summary();

        // collect inertia properties
        TScalar _totalMass = 0.0f;
        for ( auto _body : m_agentPtr->bodies )
        {
            auto _name = _body->name;
            auto _mass = utils::getBodyMass( m_mjcModelPtr, _name );

            _summary.set( "mass-" + _name, _mass );
            _totalMass += _mass;

            auto _inertiaDiag = utils::getBodyInertiaDiag( m_mjcModelPtr, _name );

            _summary.set( "inertia-" + _name, _inertiaDiag );
        }

        _summary.set( "total-mass", _totalMass );
        /**************************************************************************/
    }

    extern "C" TAgentWrapper* agent_createFromAbstract( TAgent* agentPtr )
    {
        return new TMjcKinTreeAgentWrapper( agentPtr );
    }

    extern "C" TAgentWrapper* agent_createFromFile( const std::string& name,
                                                    const std::string& filename )
    {
        return nullptr;
    }

    extern "C" TAgentWrapper* agent_createFromId( const std::string& name,
                                                  const std::string& format,
                                                  const std::string& id )
    {
        return nullptr;
    }

}}