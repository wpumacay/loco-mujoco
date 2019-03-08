
#include <mujoco_agent_wrapper.h>

namespace tysoc {
namespace mujoco {

    TMjcKinTreeAgentWrapper::TMjcKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                                      const std::string& workingDir )
        : TKinTreeAgentWrapper( kinTreeAgentPtr, workingDir )
    {
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;

        // create the mjcf resources element for this agent
        m_mjcfResourcesPtr = new mjcf::GenericElement( "mujoco" );
        m_mjcfResourcesPtr->setAttributeString( "model", name() );
        // and declare the reference to the target mjcf (where to place the resources)
        m_mjcfTargetResourcesPtr = NULL;

        // collect the required data
        _createMjcResourcesFromKinTree();
    }

    TMjcKinTreeAgentWrapper::~TMjcKinTreeAgentWrapper()
    {
        if ( m_mjcfResourcesPtr )
        {
            delete m_mjcfResourcesPtr;
            m_mjcfResourcesPtr = NULL;
        }

        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
        m_mjcfTargetResourcesPtr = NULL;
    }

    void TMjcKinTreeAgentWrapper::setMjcModel( mjModel* mjcModelPtr )
    {
        m_mjcModelPtr = mjcModelPtr;
    }

    void TMjcKinTreeAgentWrapper::setMjcData( mjData* mjcDataPtr )
    {
        m_mjcDataPtr = mjcDataPtr;
    }

    void TMjcKinTreeAgentWrapper::setMjcScene( mjvScene* mjcScenePtr )
    {
        m_mjcScenePtr = mjcScenePtr;
    }

    void TMjcKinTreeAgentWrapper::setMjcfTargetElm( mjcf::GenericElement* targetResourcesPtr )
    {
        m_mjcfTargetResourcesPtr = targetResourcesPtr;
    }

    void TMjcKinTreeAgentWrapper::_initializeInternal()
    {
        // Check if the caller (TMjcSimulation) set the target reference
        if ( !m_mjcfTargetResourcesPtr )
        {
            std::cout << "ERROR> mjc-sim object must pass a reference of the"
                      << " target resources to this agent" << std::endl;
            return;
        }

        // grab the mjcf resources to inject, namely the worlbody
        auto _worldBodyElmPtr = mjcf::findFirstChildByType( m_mjcfResourcesPtr, "worldbody" );
        // grab the actuators as well
        auto _actuatorsElmPtr = mjcf::findFirstChildByType( m_mjcfResourcesPtr, "actuator" );
        // and the assets
        auto _assetsElmPtr = mjcf::findFirstChildByType( m_mjcfResourcesPtr, "asset" );
        // and the sensors
        auto _sensorsElmPtr = mjcf::findFirstChildByType( m_mjcfResourcesPtr, "sensor" );
        // and the contacts
        auto _contactsElmPtr = mjcf::findFirstChildByType( m_mjcfResourcesPtr, "contact" );
        // then just add them to the children of the root
        if ( _assetsElmPtr )
        {
            // grab the assets in the target element
            auto _targetAssetsElmPtr    = mjcf::findFirstChildByType( m_mjcfTargetResourcesPtr, "asset" );
            auto _assetsInTarget        = _targetAssetsElmPtr->children;
            // grab the assets in our model element
            auto _assetsInModel = _assetsElmPtr->children;
            // create a set with the current elements in the assets list
            std::set< std::string > _currentAssets;
            for ( size_t i = 0; i < _assetsInTarget.size(); i++ )
            {
                _currentAssets.emplace( _assetsInTarget[i]->getAttributeString( "name" ) );
            }
            // and place our model assets if not already there
            for ( size_t i = 0; i < _assetsInModel.size(); i++ )
            {
                auto _assetElmName = _assetsInModel[i]->getAttributeString( "name" );
                if ( _currentAssets.find( _assetElmName ) == _currentAssets.end() )
                {
                    _targetAssetsElmPtr->children.push_back( _assetsInModel[i] );
                }
            }
        }
        if ( _contactsElmPtr )
            m_mjcfTargetResourcesPtr->children.push_back( _contactsElmPtr );
        if ( _worldBodyElmPtr )
            m_mjcfTargetResourcesPtr->children.push_back( _worldBodyElmPtr );
        if ( _actuatorsElmPtr )
            m_mjcfTargetResourcesPtr->children.push_back( _actuatorsElmPtr );
        if ( _sensorsElmPtr )
            m_mjcfTargetResourcesPtr->children.push_back( _sensorsElmPtr );
    }

    void TMjcKinTreeAgentWrapper::_resetInternal()
    {
        if ( m_kinTreeAgentPtr )
        {
            m_kinTreeAgentPtr->reset();
        }
    }

    void TMjcKinTreeAgentWrapper::_preStepInternal()
    {
        auto _kinActuators = m_kinTreeAgentPtr->getKinTreeActuators();

        for ( size_t i = 0; i < _kinActuators.size(); i++ )
        {
            utils::setActuatorCtrl( m_mjcModelPtr,
                                     m_mjcDataPtr,
                                     _kinActuators[i]->name,
                                     _kinActuators[i]->ctrlValue );
        }
    }

    void TMjcKinTreeAgentWrapper::_postStepInternal()
    {
        auto _kinBodies = m_kinTreeAgentPtr->getKinTreeBodies();
        for ( size_t i = 0; i < _kinBodies.size(); i++ )
        {
            // grab the position from the mujoco backend
            auto _pos = utils::getBodyPosition( m_mjcModelPtr,
                                                 m_mjcDataPtr,
                                                 _kinBodies[i]->name );
            // and the rotation as well
            float _rot[9];
            utils::getBodyOrientation( m_mjcModelPtr,
                                        m_mjcDataPtr,
                                        _kinBodies[i]->name, _rot );

            // convert the position/rotation data to our format
            TVec3 _position;
            TMat3 _rotation;

            _position.x = _pos.x;
            _position.y = _pos.y;
            _position.z = _pos.z;

            _rotation.buff[0] = _rot[0];
            _rotation.buff[1] = _rot[1];
            _rotation.buff[2] = _rot[2];
            _rotation.buff[3] = _rot[3];
            _rotation.buff[4] = _rot[4];
            _rotation.buff[5] = _rot[5];
            _rotation.buff[6] = _rot[6];
            _rotation.buff[7] = _rot[7];
            _rotation.buff[8] = _rot[8];

            // then set it to the body's worldtransform
            _kinBodies[i]->worldTransform.setPosition( _position );
            _kinBodies[i]->worldTransform.setRotation( _rotation );
        }

        // collect sensor readings
        auto _kinSensors = m_kinTreeAgentPtr->getKinTreeSensors();
        for ( size_t i = 0; i < _kinSensors.size(); i++ )
        {
            if ( _kinSensors[i]->type == "joint" )
            {
                auto _kinJointSensor = reinterpret_cast< agent::TKinTreeJointSensor* >( _kinSensors[i] );

                std::vector< float > _readings;
                // grab the reading from the jointpos sensor
                utils::getJointSensorReading( m_mjcModelPtr,
                                               m_mjcDataPtr,
                                               _kinJointSensor->name + std::string( "_jointpos" ),
                                               _readings );
                // and also the reading from the jointvel sensor
                utils::getJointSensorReading( m_mjcModelPtr,
                                               m_mjcDataPtr,
                                               _kinJointSensor->name + std::string( "_jointvel" ),
                                               _readings );
                // and store it into the sensor for later usage
                _kinJointSensor->theta       = _readings[0];
                _kinJointSensor->thetadot    = _readings[1];

                // std::cout << "theta: " << _readings[0] << std::endl;
                // std::cout << "thetadot: " << _readings[1] << std::endl;
            }
            else if ( _kinSensors[i]->type == "body" )
            {
                auto _kinBodySensor = reinterpret_cast< agent::TKinTreeBodySensor* >( _kinSensors[i] );

                std::vector< float > _readings;
                // grab the reading from the franelinvec sensor
                utils::getJointSensorReading( m_mjcModelPtr,
                                               m_mjcDataPtr,
                                               _kinBodySensor->name + std::string( "_framelinvel" ),
                                               _readings );
                // and also the reading from the framelinacc sensor
                utils::getJointSensorReading( m_mjcModelPtr,
                                               m_mjcDataPtr,
                                               _kinBodySensor->name + std::string( "_framelinacc" ),
                                               _readings );
                // and store it into the sensor for later usage
                _kinBodySensor->linVelocity     = { _readings[0], _readings[1], _readings[2] };
                _kinBodySensor->linAcceleration = { _readings[3], _readings[4], _readings[5] };

                // grab the forces and torques from mjData
                utils::getCOMForces( m_mjcModelPtr,
                                     m_mjcDataPtr,
                                     _kinBodySensor->bodyName,
                                     _kinBodySensor->comForces,
                                     _kinBodySensor->comTorques );
            }
        }
    }

    void TMjcKinTreeAgentWrapper::_createMjcResourcesFromKinTree()
    {
        if ( !m_mjcfResourcesPtr )
            return;

        if ( !m_kinTreeAgentPtr )
            return;

        auto _worldBody = new mjcf::GenericElement( "worldbody" );
        m_mjcfResourcesPtr->children.push_back( _worldBody );
        
        // Collect bodies xml data into worldbody element
        auto _rootBodyPtr = m_kinTreeAgentPtr->getRootBody();
        _createMjcResourcesFromBodyNode( _worldBody, _rootBodyPtr );

        // Collect all assets data into the model element
        _createMjcAssetsFromKinTree();
        // Create the default sensors (for joints and bodies)
        _createMjcSensorsFromKinTree();
        // Collect all actuators and replace the names accordingly
        _createMjcActuatorsFromKinTree();
        
        // Collect extra specifics depending of the type of data being parsed
        if ( m_kinTreeAgentPtr->getModelTemplateType() == agent::MODEL_TEMPLATE_TYPE_MJCF )
        {
            // Collect all contacts and replace the names accordingly
            if ( mjcf::findFirstChildByType( m_mjcfModelTemplatePtr, "contact" ) )
            {
                // create the target element where we are going to place our contacts
                auto _targetContactsElmPtr = new mjcf::GenericElement( "contact" );
                m_mjcfResourcesPtr->children.push_back( _targetContactsElmPtr );
                // and grab the contacts defined by our model template
                auto _srcContactsElmPtr = mjcf::findFirstChildByType( m_mjcfModelTemplatePtr, "contact" );

                // now place them inside the target contacts element
                auto _srcContacts = _srcContactsElmPtr->children;
                for ( size_t i = 0; i < _srcContacts.size(); i++ )
                {
                    _targetContactsElmPtr->children.push_back( _srcContacts[i] );
                }
            }
        }
        else if ( m_kinTreeAgentPtr->getModelTemplateType() == agent::MODEL_TEMPLATE_TYPE_URDF )
        {
            // Check if the root has a joint that fixes it to the world
            auto _rootJoints = _rootBodyPtr->childJoints;
            auto _isRootFixed = false;
            for ( size_t i = 0; i < _rootJoints.size(); i++ )
            {
                if ( _rootJoints[i]->type == "world" )
                {
                    _isRootFixed = true;
                    break;
                }
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
                auto _rootBodyElmPtr = mjcf::findFirstChildByType( m_mjcfResourcesPtr, "body" );
                // and add it to the rootbody
                _rootBodyElmPtr->children.push_back( _freeJointElmPtr );
                
            }
        }
        else if ( m_kinTreeAgentPtr->getModelTemplateType() == agent::MODEL_TEMPLATE_TYPE_RLSIM )
        {
            // @WIP: Add rlsim specific functionality here
        }
    }

    void TMjcKinTreeAgentWrapper::_createMjcResourcesFromBodyNode( mjcf::GenericElement* parentElmPtr,
                                                                   agent::TKinTreeBody* kinTreeBodyPtr )
    {
        auto _bodyElmPtr = new mjcf::GenericElement( "body" );
        _bodyElmPtr->setAttributeString( "name", kinTreeBodyPtr->name );
        if ( !kinTreeBodyPtr->parentBodyPtr )
        {
            // root should use its worldTransform directly
            _bodyElmPtr->setAttributeVec3( "pos", kinTreeBodyPtr->worldTransform.getPosition() );
        }
        else
        {
            // other bodies use its relative transform to the parent body
            _bodyElmPtr->setAttributeVec3( "pos", kinTreeBodyPtr->relTransform.getPosition() );
        }
        auto _quat = TMat3::toQuaternion( kinTreeBodyPtr->relTransform.getRotation() );
        _bodyElmPtr->setAttributeVec4( "quat", { _quat.w, _quat.x, _quat.y, _quat.z } );

        // add joints
        // if ( m_kinTreeAgentPtr->getModelTemplateType() ) @CONTINUE: traverse urdf format
        auto _joints = kinTreeBodyPtr->childJoints;
        for ( size_t i = 0; i < _joints.size(); i++ )
        {
            // check the joint type, standarize and check for support
            if ( _joints[i]->type == "continuous" ||
                 _joints[i]->type == "revolute" )
            {
                _joints[i]->type = "hinge";
            }
            else if ( _joints[i]->type == "prismatic" )
            {
                _joints[i]->type = "slide";
            }
            else if ( _joints[i]->type == "floating" )
            {
                _joints[i]->type = "free";
            }
            else if ( _joints[i]->type == "spherical" )
            {
                _joints[i]->type = "ball";
            }
            else if ( _joints[i]->type == "fixed" || _joints[i]->type == "world" )
            {
                // for mujoco it it's like a non-existent joint in xml
                continue;
            }
            else if ( _joints[i]->type == "planar" )
            {
                std::cout << "WARNING> joint with type: " << _joints[i]->type << " "
                          << "not supported for mujoco backend" << std::endl;
                continue;
            }

            auto _jointElmPtr = new mjcf::GenericElement( "joint" );
            _jointElmPtr->setAttributeString( "name", _joints[i]->name );
            _jointElmPtr->setAttributeString( "type", _joints[i]->type );

            if ( _joints[i]->type != "free" ) // free joints should not have any more elements
            {
                _jointElmPtr->setAttributeVec3( "pos", _joints[i]->relTransform.getPosition() );
                _jointElmPtr->setAttributeVec3( "axis", _joints[i]->relTransform.getRotation() * _joints[i]->axis );
                _jointElmPtr->setAttributeString( "limited", ( _joints[i]->limited ) ? "true" : "false" );
                if ( _joints[i]->limited )
                {
                    if ( _joints[i]->type == "ball" )
                        _jointElmPtr->setAttributeVec2( "range", { 0, _joints[i]->upperLimit } );
                    else
                        _jointElmPtr->setAttributeVec2( "range", { _joints[i]->lowerLimit, _joints[i]->upperLimit } );
                }
                // @GENERIC
                if ( _joints[i]->stiffness != 0.0f )
                    _jointElmPtr->setAttributeFloat( "stiffness", _joints[i]->stiffness );
                // @GENERIC
                if ( _joints[i]->armature != 0.0f )
                    _jointElmPtr->setAttributeFloat( "armature", _joints[i]->armature );
                // @GENERIC
                if ( _joints[i]->damping != 0.0f )
                    _jointElmPtr->setAttributeFloat( "damping", _joints[i]->damping );
            }

            _bodyElmPtr->children.push_back( _jointElmPtr );
        }

        // add geoms
        if ( kinTreeBodyPtr->childVisuals.size() > 0 )
        {
            auto _geoms = kinTreeBodyPtr->childVisuals;
            for ( size_t i = 0; i < _geoms.size(); i++ )
            {
                auto _geomElmPtr = new mjcf::GenericElement( "geom" );
                _geomElmPtr->setAttributeString( "name", _geoms[i]->name );
                _geomElmPtr->setAttributeString( "type", _geoms[i]->geometry.type );
                _geomElmPtr->setAttributeVec3( "pos", _geoms[i]->relTransform.getPosition() );
                auto _gquat = TMat3::toQuaternion( _geoms[i]->relTransform.getRotation() );
                _geomElmPtr->setAttributeVec4( "quat", { _gquat.w, _gquat.x, _gquat.y, _gquat.z } );
                _geomElmPtr->setAttributeVec3( "size", _extractMjcSizeFromStandardSize( _geoms[i]->geometry ) );

                if ( _geoms[i]->geometry.type == "mesh" )
                {
                    if ( _geoms[i]->geometry.meshId == "" )
                    {
                        _geomElmPtr->setAttributeString( "mesh", _geoms[i]->geometry.filename );
                    }
                    else
                    {
                        _geomElmPtr->setAttributeString( "mesh", _geoms[i]->geometry.meshId );
                    }
                }

                // @GENERIC
                if ( _geoms[i]->contype != -1 )
                    _geomElmPtr->setAttributeInt( "contype", _geoms[i]->contype );
                // @GENERIC
                if ( _geoms[i]->conaffinity != -1 )
                    _geomElmPtr->setAttributeInt( "conaffinity", _geoms[i]->conaffinity );
                // @GENERIC
                if ( _geoms[i]->condim != -1 )
                    _geomElmPtr->setAttributeInt( "condim", _geoms[i]->condim );
                // @GENERIC
                if ( _geoms[i]->group != -1 )
                    _geomElmPtr->setAttributeInt( "group", _geoms[i]->group );
                // @GENERIC
                if ( _geoms[i]->materialName != "" )
                    _geomElmPtr->setAttributeString( "material", _geoms[i]->materialName );
                if ( _geoms[i]->friction.ndim != 0 )
                    _geomElmPtr->setAttributeArrayFloat( "friction", _geoms[i]->friction );
                if ( _geoms[i]->density > 0 )
                    _geomElmPtr->setAttributeFloat( "density", _geoms[i]->density );
                // @GENERIC

                TVec4 _rgba = { _geoms[i]->material.diffuse.x,
                                _geoms[i]->material.diffuse.y,
                                _geoms[i]->material.diffuse.z, 1.0f };
                _geomElmPtr->setAttributeVec4( "rgba", _rgba );

                _bodyElmPtr->children.push_back( _geomElmPtr );
            }
        }
        else
        {
            auto _colls = kinTreeBodyPtr->childCollisions;
            for ( size_t i = 0; i < _colls.size(); i++ )
            {
                auto _geomElmPtr = new mjcf::GenericElement( "geom" );
                _geomElmPtr->setAttributeString( "name", _colls[i]->name );
                _geomElmPtr->setAttributeString( "type", _colls[i]->geometry.type );
                _geomElmPtr->setAttributeVec3( "pos", _colls[i]->relTransform.getPosition() );
                auto _gquat = TMat3::toQuaternion( _colls[i]->relTransform.getRotation() );
                _geomElmPtr->setAttributeVec4( "quat", { _gquat.w, _gquat.x, _gquat.y, _gquat.z } );
                _geomElmPtr->setAttributeVec3( "size", _extractMjcSizeFromStandardSize( _colls[i]->geometry ) );

                if ( _colls[i]->geometry.type == "mesh" )
                {
                    if ( _colls[i]->geometry.meshId == "" )
                    {
                        _geomElmPtr->setAttributeString( "mesh", _colls[i]->geometry.filename );
                    }
                    else
                    {
                        _geomElmPtr->setAttributeString( "mesh", _colls[i]->geometry.meshId );
                    }
                }

                // @GENERIC
                if ( _colls[i]->contype != -1 )
                    _geomElmPtr->setAttributeInt( "contype", _colls[i]->contype );
                // @GENERIC
                if ( _colls[i]->conaffinity != -1 )
                    _geomElmPtr->setAttributeInt( "conaffinity", _colls[i]->conaffinity );
                // @GENERIC
                if ( _colls[i]->condim != -1 )
                    _geomElmPtr->setAttributeInt( "condim", _colls[i]->condim );
                // @GENERIC
                if ( _colls[i]->group != -1 )
                    _geomElmPtr->setAttributeInt( "group", _colls[i]->group );

                _geomElmPtr->setAttributeVec4( "rgba", TYSOC_DEFAULT_RGBA_COLOR );

                _bodyElmPtr->children.push_back( _geomElmPtr );
            }
        }

        // add inertia element (only if not using default calculations, which is hint by NULL)
        if ( kinTreeBodyPtr->inertiaPtr )
        {
            auto _kinInertiaPtr = kinTreeBodyPtr->inertiaPtr;
            auto _inertiaElmPtr = new mjcf::GenericElement( "inertial" );
            _inertiaElmPtr->setAttributeFloat( "mass", _kinInertiaPtr->mass );
            _inertiaElmPtr->setAttributeVec3( "pos", { 0.0f, 0.0f, 0.0f} );
            auto _iquat = TMat3::toQuaternion( _kinInertiaPtr->relTransform.getRotation() );
            _inertiaElmPtr->setAttributeVec4( "quat", { 1.0f, 0.0f, 0.0f, 0.0f } );

            if ( _kinInertiaPtr->ixx > 0.000001 || 
                 _kinInertiaPtr->iyy > 0.000001 ||
                 _kinInertiaPtr->izz > 0.000001 ||
                 _kinInertiaPtr->ixy > 0.000001 ||
                 _kinInertiaPtr->ixz > 0.000001 ||
                 _kinInertiaPtr->iyz > 0.000001 )
            {
                if ( _kinInertiaPtr->ixy == 0.0 &&
                     _kinInertiaPtr->ixz == 0.0 &&
                     _kinInertiaPtr->iyz == 0.0 )
                {
                    // diagonal inertia matrix
                    _inertiaElmPtr->setAttributeVec3( "diaginertia", 
                                                      { _kinInertiaPtr->ixx, 
                                                        _kinInertiaPtr->iyy, 
                                                        _kinInertiaPtr->izz } );
                }
                else
                {
                    // full inertia matrix
                    _inertiaElmPtr->setAttributeArrayFloat( "fullinertia",
                                                            { 6, 
                                                              { _kinInertiaPtr->ixx,
                                                                _kinInertiaPtr->iyy,
                                                                _kinInertiaPtr->izz,
                                                                _kinInertiaPtr->ixy,
                                                                _kinInertiaPtr->ixz,
                                                                _kinInertiaPtr->iyz } } );
                }
            }

            _bodyElmPtr->children.push_back( _inertiaElmPtr );
        }

        // add sites
        // @WIP|@CHECK: Check if necessary


        // add this body element into the mjcf (in the correct place, given by the parent element)
        parentElmPtr->children.push_back( _bodyElmPtr );

        // and finally recursively add all bodies
        for ( size_t i = 0; i < kinTreeBodyPtr->childBodies.size(); i++ )
        {
            _createMjcResourcesFromBodyNode( _bodyElmPtr, kinTreeBodyPtr->childBodies[i] );
        }
    }

    void TMjcKinTreeAgentWrapper::_createMjcAssetsFromKinTree()
    {
        auto _meshAssets = m_kinTreeAgentPtr->getKinTreeMeshAssets();
        if ( _meshAssets.size() < 1 )
        {
            // No mesh assets to add to the model
            return;
        }

        auto _assetsElmPtr = new mjcf::GenericElement( "asset" );
        m_mjcfResourcesPtr->children.push_back( _assetsElmPtr );

        for ( size_t i = 0; i < _meshAssets.size(); i++ )
        {
            auto _meshAssetElmPtr = new mjcf::GenericElement( "mesh" );
            _meshAssetElmPtr->setAttributeString( "name", _meshAssets[i]->name );
            _meshAssetElmPtr->setAttributeString( "file", _meshAssets[i]->file );

            _assetsElmPtr->children.push_back( _meshAssetElmPtr );
        }
    }

    void TMjcKinTreeAgentWrapper::_createMjcSensorsFromKinTree()
    {
        auto _kinSensors = m_kinTreeAgentPtr->getKinTreeSensors();
        if ( _kinSensors.size() < 1 )
        {
            // no sensors to add to the model
            return;
        }

        auto _sensorsElmPtr = new mjcf::GenericElement( "sensor" );
        m_mjcfResourcesPtr->children.push_back( _sensorsElmPtr );

        for ( size_t i = 0; i < _kinSensors.size(); i++ )
        {
            if ( _kinSensors[i]->type == "joint" )
            {
                // cast to the sensor type
                auto _kinJointSensor = reinterpret_cast< agent::TKinTreeJointSensor* >( _kinSensors[i] );
                // grab the jointsensor name
                auto _sensorName = _kinJointSensor->name;
                // and the target joint name
                auto _targetJointName = _kinJointSensor->jointName;

                // create a jointpos sensor and a jointvel sensor
                auto _jointPosMjcSensorResource = new mjcf::GenericElement( "jointpos" );
                auto _jointVelMjcSensorResource = new mjcf::GenericElement( "jointvel" );

                // set the necessary properties (joint angular position sensor)
                auto _mjcJointPosSensorName = _sensorName + std::string( "_jointpos" );
                _jointPosMjcSensorResource->setAttributeString( "name", _mjcJointPosSensorName );
                _jointPosMjcSensorResource->setAttributeString( "joint", _targetJointName );

                // set the necessary properties (joint angular speed sensor)
                auto _mjcJointVelSensorName = _sensorName + std::string( "_jointvel" );
                _jointVelMjcSensorResource->setAttributeString( "name", _mjcJointVelSensorName );
                _jointVelMjcSensorResource->setAttributeString( "joint", _targetJointName );

                // add these to the sensor element
                _sensorsElmPtr->children.push_back( _jointPosMjcSensorResource );
                _sensorsElmPtr->children.push_back( _jointVelMjcSensorResource );
            }
            else if ( _kinSensors[i]->type == "body" )
            {
                // cast to the sensor type
                auto _kinBodySensor = reinterpret_cast< agent::TKinTreeBodySensor* >( _kinSensors[i] );
                // grab the jointsensor name
                auto _sensorName = _kinBodySensor->name;
                // and target body name
                auto _targetBodyName = _kinBodySensor->bodyName;

                // create a framelinvel sensor and a framelinacc sensor
                auto _bodyLinVelMjcSensorResource = new mjcf::GenericElement( "framelinvel" );
                auto _bodyLinAccMjcSensorResource = new mjcf::GenericElement( "framelinacc" );

                // set the necessary properties (body's linear velocity)
                auto _mjcBodyVelSensorName = _sensorName + std::string( "_framelinvel" );
                _bodyLinVelMjcSensorResource->setAttributeString( "name", _mjcBodyVelSensorName );
                _bodyLinVelMjcSensorResource->setAttributeString( "objtype", "body" );// @GENERALIZE
                _bodyLinVelMjcSensorResource->setAttributeString( "objname", _targetBodyName );

                // set the necessary properties (body's linear acceleration)
                auto _mjcBodyAccSensorName = _sensorName + std::string( "_framelinacc" );
                _bodyLinAccMjcSensorResource->setAttributeString( "name", _mjcBodyAccSensorName );
                _bodyLinAccMjcSensorResource->setAttributeString( "objtype", "body" );// @GENERALIZE
                _bodyLinAccMjcSensorResource->setAttributeString( "objname", _targetBodyName );                

                // add these to the sensor element
                _sensorsElmPtr->children.push_back( _bodyLinVelMjcSensorResource );
                _sensorsElmPtr->children.push_back( _bodyLinAccMjcSensorResource );
            }
        }
    }

    void TMjcKinTreeAgentWrapper::_createMjcActuatorsFromKinTree()
    {
        auto _kinActuators = m_kinTreeAgentPtr->getKinTreeActuators();
        if ( _kinActuators.size() < 1 )
        {
            // No actuators to add to the model
            return;
        }

        auto _actuatorsElmPtr = new mjcf::GenericElement( "actuator" );
        m_mjcfResourcesPtr->children.push_back( _actuatorsElmPtr );

        for ( size_t i = 0; i < _kinActuators.size(); i++ )
        {
            if ( _kinActuators[i]->jointPtr == NULL )
            {
                std::cout << "WARNING> badly defined actuator: " 
                          << _kinActuators[i]->name 
                          << std::endl;
                continue;
            }

            // Grab the joint actuator this actuator is using
            auto _kinJointPtr = _kinActuators[i]->jointPtr;

            // create the mjcf resource for this actuator
            auto _actuatorResource = new mjcf::GenericElement( _kinActuators[i]->type );
            // and set its properties
            _actuatorResource->setAttributeString( "name", _kinActuators[i]->name );
            _actuatorResource->setAttributeVec2( "ctrlrange", 
                                                 { _kinActuators[i]->minCtrl,
                                                   _kinActuators[i]->maxCtrl } );
            _actuatorResource->setAttributeString( "joint", _kinJointPtr->name );

            // @CHECK|@WIP : should change to variant with specific-params ...
            // according to the backend (mjc allows kv, kp, but bullet does not)

            // @GENERIC
            if ( _kinActuators[i]->gear.ndim > 0 )
                _actuatorResource->setAttributeArrayFloat( "gear", _kinActuators[i]->gear );
            // @GENERIC
            _actuatorResource->setAttributeString( "ctrllimited", ( _kinActuators[i]->clampCtrl ) ? "true" : "false" );
            // @GENERIC
            if ( _kinActuators[i]->type == "position" )
                _actuatorResource->setAttributeFloat( "kp", _kinActuators[i]->kp );
            else if ( _kinActuators[i]->type == "velocity" )
                _actuatorResource->setAttributeFloat( "kv", _kinActuators[i]->kv );

            _actuatorsElmPtr->children.push_back( _actuatorResource );
        }
    }

    TVec3 TMjcKinTreeAgentWrapper::_extractMjcSizeFromStandardSize( const TGeometry& geometry )
    {
        TVec3 _res;

        if ( geometry.type == "plane" )
        {
            _res = { geometry.size.x, 
                     geometry.size.y, 
                     geometry.size.z };
        }
        else if ( geometry.type == "sphere" )
        {
            _res = { geometry.size.x, 
                     geometry.size.y, 
                     geometry.size.z };
        }
        else if ( geometry.type == "capsule" ||
                  geometry.type == "cylinder" )
        {
            if ( geometry.usesFromto )
            {
                _res = { geometry.size.x, 
                         0.5f * geometry.size.y, 
                         geometry.size.z };
            }
            else
            {
                _res = { geometry.size.x, 
                         0.5f * geometry.size.y, 
                         geometry.size.z };
            }
        }
        else if ( geometry.type == "box" )
        {
            _res = { 0.5f * geometry.size.x, 
                     0.5f * geometry.size.y, 
                     0.5f * geometry.size.z };
        }


        return _res;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromAbstract( agent::TAgentKinTree* kinTreeAgentPtr,
                                                               const std::string& workingDir )
    {
        return new TMjcKinTreeAgentWrapper( kinTreeAgentPtr, workingDir );
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromFile( const std::string& name,
                                                           const std::string& filename,
                                                           const std::string& workingDir )
    {
        return NULL;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromId( const std::string& name,
                                                         const std::string& format,
                                                         const std::string& id,
                                                         const std::string& workingDir )
    {
        return NULL;
    }

}}