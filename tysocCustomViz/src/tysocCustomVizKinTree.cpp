
#include <tysocCustomVizKinTree.h>

#ifndef TYSOCMJC_RESOURCES_PATH
    #define TYSOCMJC_RESOURCES_PATH "../../res/xml"
#endif

namespace tysoc{
namespace viz{


    // @TODO: Change some of the visuals for actuators, sensors, etc. ...
    //        for special gizmos that give better feedback than primitives

    TCustomVizKinTree::TCustomVizKinTree( agent::TAgentKinTree* agentKinTreePtr,
                              engine::LScene* scenePtr )
    {
        m_scenePtr          = scenePtr;
        m_agentKinTreePtr   = agentKinTreePtr;

        _collectFromKinTree();
    }

    TCustomVizKinTree::~TCustomVizKinTree()
    {
        m_scenePtr          = NULL;
        m_agentKinTreePtr   = NULL;

        for ( size_t i = 0; i < m_vizBodies.size(); i++ )
        {
            m_vizBodies[i].meshPtr = NULL;
            m_vizBodies[i].bodyPtr = NULL;
        }
        m_vizBodies.clear();

        for ( size_t i = 0; i < m_vizJoints.size(); i++ )
        {
            m_vizJoints[i].meshPtr   = NULL;
            m_vizJoints[i].jointPtr  = NULL;
        }
        m_vizJoints.clear();

        for ( size_t i = 0; i < m_vizActuators.size(); i++ )
        {
            m_vizActuators[i].meshPtr        = NULL;
            m_vizActuators[i].actuatorPtr    = NULL;
        }
        m_vizActuators.clear();

        for ( size_t i = 0; i < m_vizSensors.size(); i++ )
        {
            m_vizSensors[i].meshPtr      = NULL;
            m_vizSensors[i].sensorPtr    = NULL;
        }
        m_vizSensors.clear();

        for ( size_t i = 0; i < m_vizVisuals.size(); i++ )
        {
            m_vizVisuals[i].meshPtr = NULL;
            m_vizVisuals[i].visualPtr = NULL;
        }
        m_vizVisuals.clear();

        for ( size_t i = 0; i < m_vizCollisions.size(); i++ )
        {
            m_vizCollisions[i].meshPtr      = NULL;
            m_vizCollisions[i].collisionPtr = NULL;
        }
        m_vizCollisions.clear();
    }

    void TCustomVizKinTree::_collectFromKinTree()
    {
        if ( !m_agentKinTreePtr )
        {
            std::cout << "WARNING> There is no kintree to extract info from" << std::endl;
            return;
        }

        // @TODO: Change render calls to debug objects (frames, etc) to a ...
        // separate simmple renderer, as it should not cast shadow, not anything

        _collectKinBodies();
        _collectKinJoints();
        _collectKinActuators();
        _collectKinSensors();
        _collectKinVisuals();
        _collectKinCollisions();
    }


    void TCustomVizKinTree::_collectKinBodies()
    {
        auto _bodies = m_agentKinTreePtr->getKinTreeBodies();

        for ( size_t i = 0; i < _bodies.size(); i++ )
        {
            TCustomVizKinBody _vizBody;
            // wrap the kinBody object
            _vizBody.bodyPtr = _bodies[i];
            // and create the appropiate mesh
            _vizBody.meshPtr = _createMesh( "sphere",
                                             VIZKINTREE_BODY_DEFAULT_SIZE,
                                             VIZKINTREE_BODY_DEFAULT_COLOR,
                                             VIZKINTREE_BODY_DEFAULT_COLOR,
                                             VIZKINTREE_BODY_DEFAULT_COLOR );
            // and create the axes
            _vizBody.axesPtr = engine::LMeshBuilder::createAxes( VIZKINTREE_AXES_DEFAULT_SIZE );
            // and add it to the scene
            m_scenePtr->addRenderable( _vizBody.axesPtr );
            // and add it to the bodies buffer
            m_vizBodies.push_back( _vizBody );

            _vizBody.axesPtr->debug = true;
        }
    }

    void TCustomVizKinTree::_collectKinJoints()
    {
        auto _joints = m_agentKinTreePtr->getKinTreeJoints();

        for ( size_t i = 0; i < _joints.size(); i++ )
        {
            TCustomVizKinJoint _vizJoint;
            // wrap the kinJoint object
            _vizJoint.jointPtr = _joints[i];
            // and create the appropiate mesh
            _vizJoint.meshPtr = _createMesh( "cylinder",
                                              VIZKINTREE_JOINT_DEFAULT_SIZE,
                                              VIZKINTREE_JOINT_DEFAULT_COLOR,
                                              VIZKINTREE_JOINT_DEFAULT_COLOR,
                                              VIZKINTREE_JOINT_DEFAULT_COLOR );
            // and create the axes
            _vizJoint.axesPtr = engine::LMeshBuilder::createAxes( VIZKINTREE_AXES_DEFAULT_SIZE );
            // and add it to the scene
            m_scenePtr->addRenderable( _vizJoint.axesPtr );
            // and add it to the joints buffer
            m_vizJoints.push_back( _vizJoint );

            _vizJoint.axesPtr->debug = true;
        }
    }

    void TCustomVizKinTree::_collectKinActuators()
    {
        auto _actuators = m_agentKinTreePtr->getKinTreeActuators();

        for ( size_t i = 0; i < _actuators.size(); i++ )
        {
            TCustomVizKinActuator _vizActuator;
            // wrap the kinActuator object
            _vizActuator.actuatorPtr = _actuators[i];
            // and create the appropiate mesh
            _vizActuator.meshPtr = _createMesh( "cylinder",
                                                VIZKINTREE_ACTUATOR_DEFAULT_SIZE,
                                                VIZKINTREE_ACTUATOR_DEFAULT_COLOR,
                                                VIZKINTREE_ACTUATOR_DEFAULT_COLOR,
                                                VIZKINTREE_ACTUATOR_DEFAULT_COLOR );
            // and create the axes
            _vizActuator.axesPtr = engine::LMeshBuilder::createAxes( VIZKINTREE_AXES_DEFAULT_SIZE );
            // and add it to the scene
            m_scenePtr->addRenderable( _vizActuator.axesPtr );
            // and add it to the actuators buffer
            m_vizActuators.push_back( _vizActuator );

            _vizActuator.axesPtr->debug = true;
        }
    }

    void TCustomVizKinTree::_collectKinSensors()
    {
        auto _sensors = m_agentKinTreePtr->getKinTreeSensors();

        for ( size_t i = 0; i < _sensors.size(); i++ )
        {
            TCustomVizKinSensor _vizSensor;
            // wrap the kinSensor object
            _vizSensor.sensorPtr = _sensors[i];
            // and create the appropiate mesh
            _vizSensor.meshPtr = _createMesh( "box",
                                              VIZKINTREE_SENSOR_DEFAULT_SIZE,
                                              VIZKINTREE_SENSOR_DEFAULT_COLOR,
                                              VIZKINTREE_SENSOR_DEFAULT_COLOR,
                                              VIZKINTREE_SENSOR_DEFAULT_COLOR );
            // and create the axes
            _vizSensor.axesPtr = engine::LMeshBuilder::createAxes( VIZKINTREE_AXES_DEFAULT_SIZE );
            // and add it to the scene
            m_scenePtr->addRenderable( _vizSensor.axesPtr );
            // and add it to the sensors buffer
            m_vizSensors.push_back( _vizSensor );

            _vizSensor.axesPtr->debug = true;
        }
    }

    void TCustomVizKinTree::_collectKinVisuals()
    {
        auto _visuals = m_agentKinTreePtr->getKinTreeVisuals();

        for ( size_t i = 0; i < _visuals.size(); i++ )
        {
            TCustomVizKinVisual _vizVisual;
            // wrap the kinVisual object
            _vizVisual.visualPtr = _visuals[i];
            // and create the appropiate mesh
            auto _geometry = _visuals[i]->geometry;
            _vizVisual.meshPtr = _createMesh( _geometry.type,
                                              _geometry.size,
                                              VIZKINTREE_VISUAL_DEFAULT_COLOR,
                                              VIZKINTREE_VISUAL_DEFAULT_COLOR,
                                              VIZKINTREE_VISUAL_DEFAULT_COLOR,
                                              _geometry.filename );
            // and create the axes
            _vizVisual.axesPtr = engine::LMeshBuilder::createAxes( VIZKINTREE_AXES_DEFAULT_SIZE );
            // and add it to the scene
            m_scenePtr->addRenderable( _vizVisual.axesPtr );
            // and add it to the visuals buffer
            m_vizVisuals.push_back( _vizVisual );

            // make axes ptr as debug @DIRTY
            _vizVisual.axesPtr->debug = true;
        }
    }

    void TCustomVizKinTree::_collectKinCollisions()
    {
        auto _collisions = m_agentKinTreePtr->getKinTreeCollisions();

        for ( size_t i = 0; i < _collisions.size(); i++ )
        {
            TCustomVizKinCollision _vizCollision;
            // wrap the kinCollision object
            _vizCollision.collisionPtr = _collisions[i];
            // and create the appropiate mesh
            auto _geometry = _collisions[i]->geometry;
            _vizCollision.meshPtr = _createMesh( _geometry.type,
                                                 _geometry.size,
                                                 VIZKINTREE_COLLISION_DEFAULT_COLOR,
                                                 VIZKINTREE_COLLISION_DEFAULT_COLOR,
                                                 VIZKINTREE_COLLISION_DEFAULT_COLOR,
                                                 _geometry.filename );
            _vizCollision.meshPtr->setWireframeMode( true );
            // and create the axes
            _vizCollision.axesPtr = engine::LMeshBuilder::createAxes( VIZKINTREE_AXES_DEFAULT_SIZE );
            // and add it to the scene
            m_scenePtr->addRenderable( _vizCollision.axesPtr );
            // and add it to the collisions buffer
            m_vizCollisions.push_back( _vizCollision );

            // @DIRTY
            _vizCollision.axesPtr->debug = true;
            _vizCollision.meshPtr->debug = true;
        }
    }


    engine::LIRenderable* TCustomVizKinTree::_createMesh( const std::string& type,
                                                    const TVec3& size,
                                                    const TVec3& cAmbient,
                                                    const TVec3& cDiffuse,
                                                    const TVec3& cSpecular,
                                                    const std::string& filename )
    {
        engine::LIRenderable* _renderable = NULL;

        if ( type == "box" )
        {
            _renderable = engine::LMeshBuilder::createBox( size.x,
                                                           size.y,
                                                           size.z );
        }
        else if ( type == "sphere" )
        {
            _renderable = engine::LMeshBuilder::createSphere( size.x );
        }
        else if ( type == "capsule" )
        {
            _renderable = engine::LMeshBuilder::createCapsule( size.x, size.y );
        }
        else if ( type == "cylinder" )
        {
            _renderable = engine::LMeshBuilder::createCylinder( size.x, size.y );
        }
        else if ( type == "mesh" )
        {
            auto _meshFilePath = std::string( TYSOCMJC_RESOURCES_PATH ) + std::string( "xml/" ) + filename;
            _renderable = engine::LMeshBuilder::createModelFromFile( _meshFilePath,
                                                                     "" );
            // std::cout << "mesh created: " << filename << std::endl;
        }

        if ( _renderable )
        {
            _renderable->getMaterial()->ambient     = { cAmbient.x, cAmbient.y, cAmbient.z };
            _renderable->getMaterial()->diffuse     = { cDiffuse.x, cDiffuse.y, cDiffuse.z };
            _renderable->getMaterial()->specular    = { cSpecular.x, cSpecular.y, cSpecular.z };

            m_scenePtr->addRenderable( _renderable );
        }
        else
        {
            std::cout << "WARNING> could not create mesh of type: " << type << std::endl;
            if ( type == "mesh" )
            {
                std::cout << "WARNING> filename of mesh: " << filename << std::endl;
            }
        }

        return _renderable;
    }

    void TCustomVizKinTree::update()
    {
        // update draw state
        for ( size_t i = 0; i < m_vizVisuals.size(); i++ )
        {
            m_vizVisuals[i].meshPtr->setWireframeMode( drawState.drawAsWireframe );
        }

        // update bodies
        for ( size_t i = 0; i < m_vizBodies.size(); i++ )
        {
            m_vizBodies[i].meshPtr->setVisibility( drawState.showBodies );
            m_vizBodies[i].axesPtr->setVisibility( drawState.drawFrameAxes &&
                                                    drawState.showBodies );
            _updateBody( m_vizBodies[i] );
        }

        // update joints
        for ( size_t i = 0; i < m_vizJoints.size(); i++ )
        {
            m_vizJoints[i].meshPtr->setVisibility( drawState.showJoints );
            m_vizJoints[i].axesPtr->setVisibility( drawState.drawFrameAxes &&
                                                    drawState.showJoints );
            _updateJoint( m_vizJoints[i] );
        }

        // update  sensors
        for ( size_t i = 0; i < m_vizSensors.size(); i++ )
        {
            m_vizSensors[i].meshPtr->setVisibility( drawState.showSensors );
            m_vizSensors[i].axesPtr->setVisibility( drawState.drawFrameAxes &&
                                                    drawState.showSensors );
            _updateSensor( m_vizSensors[i] );
        }

        // update visuals
        for ( size_t i = 0; i < m_vizVisuals.size(); i++ )
        {
            m_vizVisuals[i].meshPtr->setVisibility( drawState.showVisuals );
            m_vizVisuals[i].axesPtr->setVisibility( drawState.drawFrameAxes &&
                                                    drawState.showVisuals );
            _updateVisual( m_vizVisuals[i] );
        }

        // update actuator
        for ( size_t i = 0; i < m_vizActuators.size(); i++ )
        {
            m_vizActuators[i].meshPtr->setVisibility( drawState.showActuators );
            m_vizActuators[i].axesPtr->setVisibility( drawState.drawFrameAxes &&
                                                    drawState.showActuators );
            _updateActuator( m_vizActuators[i] );
        }

        // update collision
        for ( size_t i = 0; i < m_vizCollisions.size(); i++ )
        {
            m_vizCollisions[i].meshPtr->setVisibility( drawState.showCollisions );
            m_vizCollisions[i].axesPtr->setVisibility( drawState.drawFrameAxes &&
                                                    drawState.showCollisions );
            _updateCollision( m_vizCollisions[i] );
        }
    }

    void TCustomVizKinTree::_updateBody( TCustomVizKinBody& kinBody )
    {
        // extract body world transform
        TMat4 _worldTransform = kinBody.bodyPtr->worldTransform;
        TVec3 _worldPosition = _worldTransform.getPosition();
        TMat3 _worldRotation = _worldTransform.getRotation();

        // convert to engine datatypes
        engine::LVec3 _position = fromTVec3( _worldPosition );
        engine::LMat4 _rotation = fromTMat3( _worldRotation );

        // set the world transform of the corresponding renderables
        kinBody.meshPtr->pos        = _position;
        kinBody.meshPtr->rotation   = _rotation;

        kinBody.axesPtr->pos        = _position;
        kinBody.axesPtr->rotation   = _rotation;
    }

    void TCustomVizKinTree::_updateJoint( TCustomVizKinJoint& kinJoint )
    {
        // extract joint world transform
        TMat4 _worldTransform = kinJoint.jointPtr->worldTransform;
        TVec3 _worldPosition = _worldTransform.getPosition();
        TMat3 _worldRotation = _worldTransform.getRotation();

        // convert to engine datatypes
        engine::LVec3 _position = fromTVec3( _worldPosition );
        engine::LMat4 _rotation = fromTMat3( _worldRotation );

        // set the world transform of the corresponding renderables
        kinJoint.meshPtr->pos        = _position;
        kinJoint.meshPtr->rotation   = _rotation;

        kinJoint.axesPtr->pos        = _position;
        kinJoint.axesPtr->rotation   = _rotation;
    }

    void TCustomVizKinTree::_updateSensor( TCustomVizKinSensor& kinSensor )
    {
        // extract sensor world transform
        TMat4 _worldTransform = kinSensor.sensorPtr->worldTransform;
        TVec3 _worldPosition = _worldTransform.getPosition();
        TMat3 _worldRotation = _worldTransform.getRotation();

        // convert to engine datatypes
        engine::LVec3 _position = fromTVec3( _worldPosition );
        engine::LMat4 _rotation = fromTMat3( _worldRotation );

        // set the world transform of the corresponding renderables
        kinSensor.meshPtr->pos        = _position;
        kinSensor.meshPtr->rotation   = _rotation;

        kinSensor.axesPtr->pos        = _position;
        kinSensor.axesPtr->rotation   = _rotation;
    }

    void TCustomVizKinTree::_updateVisual( TCustomVizKinVisual& kinVisual )
    {
        // extract visual world transform
        TMat4 _worldTransform = kinVisual.visualPtr->geometry.worldTransform;
        TVec3 _worldPosition = _worldTransform.getPosition();
        TMat3 _worldRotation = _worldTransform.getRotation();

        // convert to engine datatypes
        engine::LVec3 _position = fromTVec3( _worldPosition );
        engine::LMat4 _rotation = fromTMat3( _worldRotation );

        // set the world transform of the corresponding renderables
        kinVisual.meshPtr->pos        = _position;
        kinVisual.meshPtr->rotation   = _rotation;

        kinVisual.axesPtr->pos        = _position;
        kinVisual.axesPtr->rotation   = _rotation;

        // and update the color as well
        if ( kinVisual.meshPtr->getType() != RENDERABLE_TYPE_MODEL )
        {
            kinVisual.meshPtr->getMaterial()->ambient.x = kinVisual.visualPtr->material.diffuse.x;
            kinVisual.meshPtr->getMaterial()->ambient.y = kinVisual.visualPtr->material.diffuse.y;
            kinVisual.meshPtr->getMaterial()->ambient.z = kinVisual.visualPtr->material.diffuse.z;

            kinVisual.meshPtr->getMaterial()->diffuse.x = kinVisual.visualPtr->material.diffuse.x;
            kinVisual.meshPtr->getMaterial()->diffuse.y = kinVisual.visualPtr->material.diffuse.y;
            kinVisual.meshPtr->getMaterial()->diffuse.z = kinVisual.visualPtr->material.diffuse.z;

            kinVisual.meshPtr->getMaterial()->specular.x = kinVisual.visualPtr->material.specular.x;
            kinVisual.meshPtr->getMaterial()->specular.y = kinVisual.visualPtr->material.specular.y;
            kinVisual.meshPtr->getMaterial()->specular.z = kinVisual.visualPtr->material.specular.z;
        }
        else
        {
            auto _children = reinterpret_cast< engine::LModel* >( kinVisual.meshPtr )->getMeshes();
            for ( size_t i = 0; i < _children.size(); i++ )
            {
                _children[i]->getMaterial()->ambient.x = kinVisual.visualPtr->material.diffuse.x;
                _children[i]->getMaterial()->ambient.y = kinVisual.visualPtr->material.diffuse.y;
                _children[i]->getMaterial()->ambient.z = kinVisual.visualPtr->material.diffuse.z;

                _children[i]->getMaterial()->diffuse.x = kinVisual.visualPtr->material.diffuse.x;
                _children[i]->getMaterial()->diffuse.y = kinVisual.visualPtr->material.diffuse.y;
                _children[i]->getMaterial()->diffuse.z = kinVisual.visualPtr->material.diffuse.z;

                _children[i]->getMaterial()->specular.x = kinVisual.visualPtr->material.specular.x;
                _children[i]->getMaterial()->specular.y = kinVisual.visualPtr->material.specular.y;
                _children[i]->getMaterial()->specular.z = kinVisual.visualPtr->material.specular.z;
            }
        }
    }

    void TCustomVizKinTree::_updateActuator( TCustomVizKinActuator& kinActuator )
    {
        // extract actuator world transform
        TMat4 _worldTransform = kinActuator.actuatorPtr->worldTransform;
        TVec3 _worldPosition = _worldTransform.getPosition();
        TMat3 _worldRotation = _worldTransform.getRotation();

        // convert to engine datatypes
        engine::LVec3 _position = fromTVec3( _worldPosition );
        engine::LMat4 _rotation = fromTMat3( _worldRotation );

        // set the world transform of the corresponding renderables
        kinActuator.meshPtr->pos        = _position;
        kinActuator.meshPtr->rotation   = _rotation;

        kinActuator.axesPtr->pos        = _position;
        kinActuator.axesPtr->rotation   = _rotation;
    }

    void TCustomVizKinTree::_updateCollision( TCustomVizKinCollision& kinCollision )
    {
        // extract collision world transform
        TMat4 _worldTransform = kinCollision.collisionPtr->geometry.worldTransform;
        TVec3 _worldPosition = _worldTransform.getPosition();
        TMat3 _worldRotation = _worldTransform.getRotation();

        // convert to engine datatypes
        engine::LVec3 _position = fromTVec3( _worldPosition );
        engine::LMat4 _rotation = fromTMat3( _worldRotation );

        // set the world transform of the corresponding renderables
        kinCollision.meshPtr->pos        = _position;
        kinCollision.meshPtr->rotation   = _rotation;

        kinCollision.axesPtr->pos        = _position;
        kinCollision.axesPtr->rotation   = _rotation;
    }

    agent::TAgentKinTree* TCustomVizKinTree::getKinTreePtr()
    {
        return m_agentKinTreePtr;
    }

}}