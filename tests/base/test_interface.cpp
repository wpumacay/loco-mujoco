
#include "test_interface.h"


namespace mujoco
{

    /***************************************************************************
    *                                                                          *
    *                               UTILITIES                                  *
    *                                                                          *
    ***************************************************************************/

    std::string mjtGeom2string( int type )
    {
        if ( type == mjGEOM_PLANE )
            return "plane";
        if ( type == mjGEOM_HFIELD )
            return "hfield";
        if ( type == mjGEOM_SPHERE )
            return "sphere";
        if ( type == mjGEOM_CAPSULE )
            return "capsule";
        if ( type == mjGEOM_ELLIPSOID )
            return "ellipsoid";
        if ( type == mjGEOM_CYLINDER )
            return "cylinder";
        if ( type == mjGEOM_BOX )
            return "box";
        if ( type == mjGEOM_MESH )
            return "mesh";

        return "undefined";
    }

    std::string mjtJoint2string( int type )
    {
        if ( type == mjJNT_FREE )
            return "free";
        if ( type == mjJNT_BALL )
            return "ball";
        if ( type == mjJNT_SLIDE )
            return "slide";
        if ( type == mjJNT_HINGE )
            return "hinge";

        return "undefined";
    }

    tysoc::TVec3 mjtNum2vec3( mjtNum* numPtr )
    {
        assert( numPtr != NULL );
        // Grab exactly 3 elements (so, can throw in 3-vecs, 4-vecs, 5-vecs, ...)
        return tysoc::TVec3( numPtr[0], numPtr[1], numPtr[2] );
    }

    tysoc::TVec4 mjtNum2vec4( mjtNum* numPtr )
    {
        assert( numPtr != NULL );
        // Grab exactly 4 elements (so, can throw in 4-vecs, 5-vecs, 6-vecs, ...)
        return tysoc::TVec4( numPtr[0], numPtr[1], numPtr[2], numPtr[3] );
    }

    tysoc::TVec4 mjtNumQuat2vec4( mjtNum* numPtr )
    {
        assert( numPtr != NULL );
        // Grab exactly 4 elements (so, can throw in 4-vecs, 5-vecs, 6-vecs, ...)
        return tysoc::TVec4( numPtr[1], numPtr[2], numPtr[3], numPtr[0] );
    }

    tysoc::TVec3 floatptr2vec3( float* floatPtr )
    {
        assert( floatPtr != NULL );
        // Grab exactly 3 elements (so, can throw in 3-vecs, 4-vecs, 5-vecs, ...)
        return tysoc::TVec3( floatPtr[0], floatPtr[1], floatPtr[2] );
    }

    tysoc::TVec4 floatptr2vec4( float* floatPtr )
    {
        assert( floatPtr != NULL );
        // Grab exactly 4 elements (so, can throw in 4-vecs, 5-vecs, 6-vecs, ...)
        return tysoc::TVec4( floatPtr[0], floatPtr[1], floatPtr[2], floatPtr[3] );
    }

    /***************************************************************************
    *                                                                          *
    *                            BODY-WRAPPER                                  *
    *                                                                          *
    ***************************************************************************/

    SimBody::SimBody( const std::string& bodyName,
                      mjModel* mjcModelPtr,
                      mjData* mjcDataPtr )
    {
        m_bodyName = bodyName;
        m_bodyId = -1;

        // default values of some body props (to be extracted later)
        m_jointsNum = -1;
        m_jointsAdr = -1;

        m_mjcModelPtr = mjcModelPtr;
        m_mjcDataPtr = mjcDataPtr;

        /******************** Grab information from mujoco ********************/

        m_bodyId = mj_name2id( m_mjcModelPtr, mjOBJ_BODY, m_bodyName.c_str() );
        if ( m_bodyId == -1 )
        {
            std::cout << "ERROR> can't grab id from body: " << m_bodyName << std::endl;
        }
        else
        {
            _grabGeometries();
            _grabJoints();
        }
    }

    SimBody::~SimBody()
    {
        m_mjcModelPtr = NULL;
        m_mjcDataPtr = NULL;

        m_geomsLocalTransforms.clear();
        m_geomsGraphics.clear();
        m_geomsNames.clear();
        m_geomsIds.clear();

        for ( size_t i = 0; i < m_simJoints.size(); i++ )
            delete m_simJoints[i];

        m_simJoints.clear();
        m_simJointsMap.clear();
    }

    void SimBody::_grabGeometries()
    {
        // Sanity check: make sure no other geometries were created
        assert( m_geomsGraphics.size() == 0 );
        assert( m_geomsLocalTransforms.size() == 0 );

        // Take the number of geometries in this body
        int _numGeoms = m_mjcModelPtr->body_geomnum[m_bodyId];
        int _startGeoms = m_mjcModelPtr->body_geomadr[m_bodyId];

        // std::cout << "LOG> _numGeoms: " << _numGeoms << std::endl;
        // std::cout << "LOG> _startGeoms: " << _startGeoms << std::endl;

        // Grab all geometries from the model
        for ( int i = 0; i < _numGeoms; i++ )
        {
            auto _geomId = _startGeoms + i;
            auto _geomName = std::string( "" );
            auto _geomNameChars = mj_id2name( m_mjcModelPtr, mjOBJ_GEOM, _geomId );
            if ( _geomNameChars )
                _geomName = std::string( _geomNameChars );
            m_geomsIds.push_back( _geomId );
            m_geomsNames.push_back( _geomName );

            auto _geomType = mjtGeom2string( m_mjcModelPtr->geom_type[_geomId] );
            auto _geomSize = mjtNum2vec3( m_mjcModelPtr->geom_size + 3 * _geomId );
            auto _geomColor = floatptr2vec4( m_mjcModelPtr->geom_rgba + 4 * _geomId );
            auto _geomGraphics = _buildGeomGraphics( _geomType, _geomSize, _geomColor );
            m_geomsGraphics.push_back( _geomGraphics );

            auto _geomLocalPos = mjtNum2vec3( m_mjcModelPtr->geom_pos + 3 * _geomId );
            auto _geomLocalQuat = mjtNumQuat2vec4( m_mjcModelPtr->geom_quat + 4 * _geomId );
            auto _geomLocalTransform = tysoc::TMat4( _geomLocalPos, _geomLocalQuat );
            m_geomsLocalTransforms.push_back( _geomLocalTransform );
        }
    }

    void SimBody::_grabJoints()
    {
        // print qpos and qvel from joints + dofs info
        m_jointsNum = m_mjcModelPtr->body_jntnum[m_bodyId];
        m_jointsAdr = m_mjcModelPtr->body_jntadr[m_bodyId];

        for ( int i = 0; i < m_jointsNum; i++ )
        {
            auto _joint = new SimJoint( m_jointsAdr + i,
                                        m_mjcModelPtr,
                                        m_mjcDataPtr );

            m_simJoints.push_back( _joint );
            m_simJointsMap[ _joint->name() ] = _joint;
        }
    }

    engine::LIRenderable* SimBody::_buildGeomGraphics( const std::string& type,
                                                       const tysoc::TVec3& size,
                                                       const tysoc::TVec4& color )
    {
        engine::LIRenderable* _renderable = NULL;

        if ( type == "plane" )
            _renderable = engine::LMeshBuilder::createPlane( 2.0 * size.x, 2.0 * size.y );
        else if ( type == "box" )
            _renderable = engine::LMeshBuilder::createBox( 2.0 * size.x, 2.0 * size.y, 2.0 * size.z );
        else if ( type == "sphere" )
            _renderable = engine::LMeshBuilder::createSphere( size.x );
        else if ( type == "capsule" )
            _renderable = engine::LMeshBuilder::createCapsule( size.x, 2.0 * size.y );
        else if ( type == "cylinder" )
            _renderable = engine::LMeshBuilder::createCylinder( size.x, 2.0 * size.y );

        if ( _renderable )
            _renderable->getMaterial()->setColor( { color.x, color.y, color.z } );

        return _renderable;
    }

    void SimBody::update()
    {
        assert( m_bodyId != -1 );

        // Grab the current position and orientation of the body
        m_bodyWorldPos = mjtNum2vec3( m_mjcDataPtr->xpos + 3 * m_bodyId );
        m_bodyWorldQuat = mjtNumQuat2vec4( m_mjcDataPtr->xquat + 4 * m_bodyId );
        m_bodyWorldTransform = tysoc::TMat4( m_bodyWorldPos, m_bodyWorldQuat );

        // Update the position of the geometries from the
        for ( size_t i = 0; i < m_geomsGraphics.size(); i++ )
        {
            if ( !m_geomsGraphics[i] )
                continue;

            auto _geomWorldTransform = m_bodyWorldTransform * m_geomsLocalTransforms[i];
            auto _geomWorldPos = _geomWorldTransform.getPosition();
            auto _geomWorldRot = _geomWorldTransform.getRotation();

            // grab position
            m_geomsGraphics[i]->pos = { _geomWorldPos.x, _geomWorldPos.y, _geomWorldPos.z };
            // grab rotation
            for ( size_t row = 0; row < 3; row++ )
                for ( size_t col = 0; col < 3; col++ )
                    m_geomsGraphics[i]->rotation.buff[row + 4 * col] = _geomWorldRot.buff[row + 3 * col];
        }

        // grab force-torque information of the body
        m_extComForce = { (float) m_mjcDataPtr->cfrc_ext[6 * m_bodyId + 0], 
                          (float) m_mjcDataPtr->cfrc_ext[6 * m_bodyId + 1], 
                          (float) m_mjcDataPtr->cfrc_ext[6 * m_bodyId + 2] };

        m_extComTorque = { (float) m_mjcDataPtr->cfrc_ext[6 * m_bodyId + 3], 
                           (float) m_mjcDataPtr->cfrc_ext[6 * m_bodyId + 4], 
                           (float) m_mjcDataPtr->cfrc_ext[6 * m_bodyId + 5] };
    }

    SimJoint* SimBody::getJointByName( const std::string& name )
    {
        if ( m_simJointsMap.find( name ) == m_simJointsMap.end() )
            return NULL;

        return m_simJointsMap[name];
    }

    void SimBody::print()
    {
        assert( m_bodyId != -1 );

        std::cout << "LOG> Body name: " << m_bodyName << std::endl;

        // print root body id and name
        int _rootId     = m_mjcModelPtr->body_rootid[m_bodyId];
        auto _rootName  = std::string( "" );
        if ( _rootId != -1 ) 
            _rootName = std::string( mj_id2name( m_mjcModelPtr, mjOBJ_BODY, _rootId ) );

        std::cout << "LOG> Root-body name: " << _rootName << std::endl;

        // print dof-information
        int _dofsNum = m_mjcModelPtr->body_dofnum[m_bodyId];
        int _dofsAdr = m_mjcModelPtr->body_dofadr[m_bodyId];

        std::cout << "LOG> num-dofs: " << _dofsNum << std::endl;
        std::cout << "LOG> dof-start-address: " << _dofsAdr << std::endl;

        // print qpos and qvel from joints + dofs info
        int _jntsNum = m_mjcModelPtr->body_jntnum[m_bodyId];
        int _jntsAdr = m_mjcModelPtr->body_jntadr[m_bodyId];

        std::cout << "LOG> num-jnts: " << _jntsNum << std::endl;
        std::cout << "LOG> jnts-start-address: " << _jntsAdr << std::endl;

        for ( size_t i = 0; i < m_simJoints.size(); i++ )
            m_simJoints[i]->print();
    }

    void SimBody::reset()
    {
        assert( m_bodyId != -1 );

        for ( size_t i = 0; i < m_simJoints.size(); i++ )
            m_simJoints[i]->reset();
    }

    /***************************************************************************
    *                                                                          *
    *                              JOINT WRAPPER                               *
    *                                                                          *
    ***************************************************************************/

    SimJoint::SimJoint( int jointId,
                        mjModel* mjcModelPtr,
                        mjData* mjcDataPtr )
    {
        m_jointId = jointId;
        m_jointName = "undefined";
        m_mjcModelPtr = mjcModelPtr;
        m_mjcDataPtr = mjcDataPtr;

        m_jointQposNum = -1;
        m_jointQposAdr = -1;
        m_jointQvelNum = -1;
        m_jointQvelAdr = -1;

        /******************** Grab information from mujoco ********************/

        if ( m_jointId == -1 )
        {
            std::cout << "ERROR> can't grab id from joint: " << m_jointName << std::endl;
            return;
        }

        auto _jointNameChars = mj_id2name( m_mjcModelPtr, mjOBJ_JOINT, m_jointId );
        if ( _jointNameChars ) { m_jointName = std::string( _jointNameChars ); }

        m_jointType         = m_mjcModelPtr->jnt_type[m_jointId];
        m_jointBodyParentId = m_mjcModelPtr->jnt_bodyid[m_jointId];
        m_jointQposAdr      = m_mjcModelPtr->jnt_qposadr[m_jointId];
        m_jointQvelAdr      = m_mjcModelPtr->jnt_dofadr[m_jointId];

        if ( m_jointType == mjJNT_FREE )
        {
            m_jointQposNum = 7;
            m_jointQvelNum = 6;
        }
        else if ( m_jointType == mjJNT_BALL )
        {
            m_jointQposNum = 4;
            m_jointQvelNum = 3;
        }
        else if ( m_jointType == mjJNT_SLIDE )
        {
            m_jointQposNum = m_jointQvelNum = 1;
        }
        else if ( m_jointType == mjJNT_HINGE )
        {
            m_jointQposNum = m_jointQvelNum = 1;
        }

        m_jointLocalPos = mjtNum2vec3( m_mjcModelPtr->jnt_pos + 3 * m_jointId );
        m_jointLocalTransform.setPosition( m_jointLocalPos );
    }

    SimJoint::~SimJoint()
    {
        m_mjcModelPtr = NULL;
        m_mjcDataPtr = NULL;
    }

    void SimJoint::update()
    {
        if ( m_jointId == -1 || m_jointBodyParentId == -1 )
            return;

        auto _parentBodyWorldPos = mjtNum2vec3( m_mjcDataPtr->xpos + 3 * m_jointBodyParentId );
        auto _parentBodyWorldQuat = mjtNumQuat2vec4( m_mjcDataPtr->xquat + 4 * m_jointBodyParentId );
        auto _parentBodyWorldTransform = tysoc::TMat4( _parentBodyWorldPos, _parentBodyWorldQuat );

        m_jointWorldTransform = _parentBodyWorldTransform * m_jointLocalTransform;
    }

    void SimJoint::print()
    {

        std::cout << "LOG> joint-type: " << mjtJoint2string( m_jointType ) << std::endl;

        std::cout << "LOG> qpos-num: " << m_jointQposNum << std::endl;
        std::cout << "LOG> qpos-address: " << m_jointQposAdr << std::endl;

        std::cout << "LOG> qvel(dof)-num: " << m_jointQvelNum << std::endl;
        std::cout << "LOG> qvel(dof)-address: " << m_jointQvelAdr << std::endl;

        for ( int q = 0; q < m_jointQposNum; q++ )
            std::cout << "LOG> qpos(" << q << ") = " << m_mjcDataPtr->qpos[m_jointQposAdr + q] << std::endl;

        for ( int v = 0; v < m_jointQvelNum; v++ )
            std::cout << "LOG> qvel(" << v << ") = " << m_mjcDataPtr->qvel[m_jointQvelAdr + v] << std::endl;
    }

    void SimJoint::reset()
    {
        for ( int i = 0; i < m_jointQposNum; i++ )
            m_mjcDataPtr->qpos[m_jointQposAdr + i] = m_mjcModelPtr->qpos0[m_jointQposAdr + i];

        for ( int i = 0; i < m_jointQvelNum; i++ )
            m_mjcDataPtr->qvel[m_jointQvelAdr + i] = 0.0;
    }

    void SimJoint::reset( const std::vector< mjtNum >& qpos )
    {
        for ( int  i = 0; i < m_jointQposNum; i++ )
            if ( i < qpos.size() )
                m_mjcDataPtr->qpos[m_jointQposAdr + i] = qpos[i];

        for ( int i = 0; i < m_jointQvelNum; i++ )
            m_mjcDataPtr->qvel[m_jointQvelAdr + i] = 0.0;
    }

    void SimJoint::setQpos( const std::vector< mjtNum >& qpos )
    {
        for ( int  i = 0; i < m_jointQposNum; i++ )
            if ( i < qpos.size() )
                m_mjcDataPtr->qpos[m_jointQposAdr + i] = qpos[i];
    }

    std::vector< mjtNum > SimJoint::getQpos()
    {
        std::vector< mjtNum > _qpos;

        for ( int  i = 0; i < m_jointQposNum; i++ )
            _qpos.push_back( m_mjcDataPtr->qpos[m_jointQposAdr + i] );

        return _qpos;
    }

    /***************************************************************************
    *                                                                          *
    *                             CONTACT WRAPPER                              *
    *                                                                          *
    ***************************************************************************/

    SimContact::SimContact( mjModel* mjcModelPtr,
                            mjData* mjcDataPtr )
    {
        m_active = false;
        m_graphicsContactPoint = engine::LMeshBuilder::createSphere( 0.05 );
        m_graphicsContactDirection = engine::LMeshBuilder::createArrow( 1.0, "x" );
        m_worldPos = tysoc::TVec3( 0., 0., 0. ); // Origin
        m_worldRot = tysoc::TMat3(); //Identity

        m_mjcModelPtr = mjcModelPtr;
        m_mjcDataPtr = mjcDataPtr;
    }

    SimContact::~SimContact()
    {
        delete m_graphicsContactPoint;
        delete m_graphicsContactDirection;

        m_graphicsContactPoint = NULL;
        m_graphicsContactDirection = NULL;
    }

    void SimContact::update( const mjContact& contactInfo )
    {
        // activate this contact again
        m_active = true;

        // grab position-rotation
        m_worldPos = { (float) contactInfo.pos[0], (float) contactInfo.pos[1], (float) contactInfo.pos[2] };
        m_worldRot = { (float) contactInfo.frame[0], (float) contactInfo.frame[3], (float) contactInfo.frame[6],
                       (float) contactInfo.frame[1], (float) contactInfo.frame[4], (float) contactInfo.frame[7],
                       (float) contactInfo.frame[2], (float) contactInfo.frame[5], (float) contactInfo.frame[8] };

        // update the renderables information
        m_graphicsContactPoint->setVisibility( true );
        m_graphicsContactDirection->setVisibility( true );

        m_graphicsContactPoint->pos = { m_worldPos.x, m_worldPos.y, m_worldPos.z };
        m_graphicsContactDirection->pos = { m_worldPos.x, m_worldPos.y, m_worldPos.z };
        for ( size_t row = 0; row < 3; row++ )
            for ( size_t col = 0; col < 3; col++ )
                m_graphicsContactDirection->rotation.buff[row + 4 * col] = m_worldRot.buff[row + 3 * col];

        // grab information of the geom-colliders that are in contact
        m_geomId1 = contactInfo.geom1;
        m_geomId2 = contactInfo.geom2;

        auto _geomName1Chars = mj_id2name( m_mjcModelPtr, mjOBJ_GEOM, m_geomId1 );
        if ( _geomName1Chars )
            m_geomName1 = std::string( _geomName1Chars );

        auto _geomName2Chars = mj_id2name( m_mjcModelPtr, mjOBJ_GEOM, m_geomId2 );
        if ( _geomName2Chars )
            m_geomName2 = std::string( _geomName2Chars );
    }

    void SimContact::reset()
    {
        m_active = true;
        m_graphicsContactPoint->setVisibility( false );
        m_graphicsContactDirection->setVisibility( false );
    }


    /***************************************************************************
    *                                                                          *
    *                              AGENT WRAPPER                               *
    *                                                                          *
    ***************************************************************************/

    SimAgent::SimAgent( int rootBodyId, 
                        const std::vector< SimBody* >& bodies,
                        mjModel* mjcModelPtr, 
                        mjData* mjcDataPtr )
    {
        m_rootBodyId = rootBodyId;
        m_rootBodyName = "";
        m_mjcModelPtr = mjcModelPtr;
        m_mjcDataPtr = mjcDataPtr;

        // grab the id of the root body
        auto _rootBodyNameChars = mj_id2name( m_mjcModelPtr, mjOBJ_BODY, m_rootBodyId );
        if ( _rootBodyNameChars )
            m_rootBodyName = std::string( _rootBodyNameChars );

        if ( m_rootBodyId != -1 )
            _collectBodies( bodies );
    }

    void SimAgent::_collectBodies( const std::vector< SimBody* >& bodies )
    {
        assert( m_bodies.size() == 0 );

        for ( size_t i = 0; i < bodies.size(); i++ )
        {
            if ( bodies[i]->id() == -1 )
                continue;

            int _rootId = m_mjcModelPtr->body_rootid[bodies[i]->id()];

            if ( _rootId == m_rootBodyId )
                m_bodies.push_back( bodies[i] );
        }

        for ( size_t i = 0; i < m_bodies.size(); i++ )
        {
            auto _joints = m_bodies[i]->joints();
            for ( size_t j = 0; j < _joints.size(); j++ )
                m_joints.push_back( _joints[j] );
        }
    }

    SimAgent::~SimAgent()
    {
        m_mjcModelPtr = NULL;
        m_mjcDataPtr = NULL;
        m_bodies.clear();
    }

    void SimAgent::update()
    {
        if ( m_rootBodyId == -1 )
            return;

        // Grab the current position and orientation of the root-body of the agent
        m_rootBodyWorldPos = mjtNum2vec3( m_mjcDataPtr->xpos + 3 * m_rootBodyId );
        m_rootBodyWorldQuat = mjtNumQuat2vec4( m_mjcDataPtr->xquat + 4 * m_rootBodyId );
        m_rootBodyWorldTransform.setPosition( m_rootBodyWorldPos );
        m_rootBodyWorldTransform.setRotation( m_rootBodyWorldQuat );
    }

    void SimAgent::reset()
    {
        for ( size_t i = 0; i < m_bodies.size(); i++ )
            m_bodies[i]->reset();
    }

    /***************************************************************************
    *                                                                          *
    *                             BASE-APPLICATION                             *
    *                                                                          *
    ***************************************************************************/

    ITestApplication::ITestApplication()
    {
        m_mjcModelPtr = NULL;
        m_mjcDataPtr = NULL;
        m_mjcScenePtr = NULL;
        m_mjcCameraPtr = NULL;
        m_mjcOptionPtr = NULL;
        m_mjcModelFile = "";
        m_isTerminated = false;
        m_isRunning = true;
        m_currentAgentIndx = -1;
        m_currentAgentName = "";
    }

    ITestApplication::~ITestApplication()
    {
        for ( size_t i = 0; i < m_simBodies.size(); i++ )
            delete m_simBodies[i];

        m_simBodies.clear();
        m_simBodiesMap.clear();

        m_simAgents.clear();

        m_simContacts.clear();

        // @TODO: Check glfw's master branch, as mujoco's glfw seems be older
        // @TODO: Check why are we linking against glfw3 from mjc libs
    #if defined(__APPLE__) || defined(_WIN32)
        delete m_graphicsApp;
    #endif
        m_graphicsApp = NULL;
        m_graphicsScene = NULL;

        mjv_freeScene( m_mjcScenePtr );
        mj_deleteData( m_mjcDataPtr );
        mj_deleteModel( m_mjcModelPtr );
        mj_deactivate();

        m_mjcCameraPtr = NULL;
        m_mjcOptionPtr = NULL;
        m_mjcScenePtr = NULL;
        m_mjcDataPtr = NULL;
        m_mjcModelPtr = NULL;
    }

    void ITestApplication::init()
    {
        _initScenario();
        _initGraphics();
        _initPhysics();
    }

    void ITestApplication::_initScenario()
    {
        // delegate to virtual method
        _initScenarioInternal();
    }

    void ITestApplication::_initPhysics()
    {
        assert( m_graphicsApp != NULL );
        assert( m_graphicsScene != NULL );

        // Activate using the appropriate licence file
        mj_activate( "/home/gregor/.mujoco/mjkey.txt" );

        // Create the model (by now the model-file must have been already set)
        m_mjcModelPtr = mj_loadXML( m_mjcModelFile.c_str(), NULL, m_mjcErrorMsg, 1000 );
        if ( !m_mjcModelPtr )
        {
            std::cout << "ERROR> could not create model: " << m_mjcModelFile << std::endl;
            std::cout << "ERROR> mujoco error message: " << m_mjcErrorMsg << std::endl;
            return;
        }

        // Create other mujoco-structures
        m_mjcDataPtr = mj_makeData( m_mjcModelPtr );
        m_mjcCameraPtr = new mjvCamera();
        m_mjcOptionPtr = new mjvOption();
        m_mjcScenePtr  = new mjvScene();
        mjv_defaultCamera( m_mjcCameraPtr );
        mjv_defaultOption( m_mjcOptionPtr );
        mjv_makeScene( m_mjcModelPtr, m_mjcScenePtr, 2000 );

        // grab all bodies and wrap them
        int _numBodies = m_mjcModelPtr->nbody;
        for ( size_t _bodyId = 0; _bodyId < _numBodies; _bodyId++ )
        {
            auto _bodyName = std::string( "" );
            auto _bodyNameChars = mj_id2name( m_mjcModelPtr, mjOBJ_BODY, _bodyId );
            if ( _bodyNameChars )
                _bodyName = std::string( _bodyNameChars );
            auto _bodyObj = new SimBody( _bodyName, m_mjcModelPtr, m_mjcDataPtr );

            m_simBodies.push_back( _bodyObj );
            m_simBodiesMap[ _bodyName ] = _bodyObj;

            // add renderables to the graphics scene
            auto _renderables = _bodyObj->geomsGraphics();
            for ( size_t i = 0; i < _renderables.size(); i++ )
                m_graphicsScene->addRenderable( _renderables[i] );
        }

        // grab all agents and wrap them
        for ( size_t _bodyId = 0; _bodyId < _numBodies; _bodyId++ )
        {
            int _rootBodyId = m_mjcModelPtr->body_rootid[_bodyId];

            // create agents only using root bodies
            if ( _rootBodyId != _bodyId )
                continue;

            // skip single-geom bodies of world bodies
            if ( m_simBodies[_bodyId]->name().find( "world" ) != std::string::npos )
                continue;

            m_simAgents.push_back( new SimAgent( _rootBodyId,
                                                 m_simBodies,
                                                 m_mjcModelPtr,
                                                 m_mjcDataPtr ) );
        }

        // show some overall information
        std::cout << "LOG> nq: " << m_mjcModelPtr->nq << std::endl;
        std::cout << "LOG> nv: " << m_mjcModelPtr->nv << std::endl;
        std::cout << "LOG> nbody: " << m_mjcModelPtr->nbody << std::endl;
        std::cout << "LOG> ngeom: " << m_mjcModelPtr->ngeom << std::endl;
        std::cout << "LOG> njnt: " << m_mjcModelPtr->njnt << std::endl;
    }

    void ITestApplication::_initGraphics()
    {
        m_graphicsApp = engine::LApp::GetInstance();
        m_graphicsScene = engine::LApp::GetInstance()->scene();

        auto _camera = new engine::LFpsCamera( "fps",
                                               { 1.0f, 2.0f, 1.0f },
                                               { -2.0f, -4.0f, -2.0f },
                                               engine::LICamera::UP_Z );

        auto _light = new engine::LLightDirectional( { 0.8, 0.8, 0.8 }, 
                                                     { 0.8, 0.8, 0.8 },
                                                     { 0.3, 0.3, 0.3 }, 
                                                     0, 
                                                     { 0.0, 0.0, -1.0 } );
        _light->setVirtualPosition( { 5.0, 0.0, 5.0 } );

        m_graphicsScene->addCamera( _camera );
        m_graphicsScene->addLight( _light );

        // Initialize UI resources
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& _io = ImGui::GetIO(); (void) _io;
    #ifdef __APPLE__
        ImGui_ImplOpenGL3_Init( "#version 150" );
    #else
        ImGui_ImplOpenGL3_Init( "#version 130" );
    #endif
        ImGui_ImplGlfw_InitForOpenGL( m_graphicsApp->window()->getGLFWwindow(), false );
        ImGui::StyleColorsDark();
    }

    void ITestApplication::reset()
    {
        // do some specific initialization
        _resetInternal();
    }

    void ITestApplication::step()
    {
        if ( m_isRunning && !m_isTerminated )
        {
            // Take a step in the simulator (MuJoCo)
            mjtNum _simstart = m_mjcDataPtr->time;
            while ( m_mjcDataPtr->time - _simstart < 1.0 / 60.0 )
                mj_step( m_mjcModelPtr, m_mjcDataPtr );
        }

        // collect contacts
        int _nContacts = m_mjcDataPtr->ncon;
        int _nContactsExtra = _nContacts - m_simContacts.size();
        // std::cout << "LOG> num-contacts: " << _nContacts << std::endl;
        // std::cout << "LOG> num-extra-contacts: " << _nContactsExtra << std::endl;

        // extend contacts pool in case more contacts are required
        for ( int i = 0; i < _nContactsExtra; i++ )
        {
            auto _simContact = new SimContact( m_mjcModelPtr, m_mjcDataPtr );
            m_graphicsScene->addRenderable( _simContact->graphicsContactDirection() );
            m_graphicsScene->addRenderable( _simContact->graphicsContactPoint() );

            m_simContacts.push_back( _simContact );
        }
        // reset all sim-contacts' information
        for ( int i = 0; i < m_simContacts.size(); i++ )
            m_simContacts[i]->reset();

        // assign contact information to our sim-contacts
        for ( int i = 0; i < _nContacts; i++ )
            m_simContacts[i]->update( m_mjcDataPtr->contact[i] );

        // Update all body-wrappers
        for ( size_t i = 0; i < m_simBodies.size(); i++ )
        {
            if ( !m_simBodies[i] )
                continue;

            m_simBodies[i]->update();
        }

        // Update all agent wrappers
        for ( size_t i = 0; i < m_simAgents.size(); i++ )
        {
            if ( !m_simAgents[i] )
                continue;

            m_simAgents[i]->update();
        }

        // do some custom step functionality
        _stepInternal();

        if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_SPACE ) )
        {
            m_graphicsScene->getCurrentCamera()->setActiveMode( false );
            m_graphicsApp->window()->enableCursor();
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_ENTER ) )
        {
            m_graphicsScene->getCurrentCamera()->setActiveMode( true );
            m_graphicsApp->window()->disableCursor();
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_ESCAPE ) )
        {
            m_isTerminated = true;
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_P ) )
        {
            togglePause();
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_Q ) )
        {
            for ( size_t i = 0; i < m_simBodies.size(); i++ )
            {
                if ( !m_simBodies[i] )
                    continue;

                m_simBodies[i]->print();
            }
        }
        else if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_R ) )
        {
            for ( size_t i = 0; i < m_simAgents.size(); i++ )
            {
                if ( !m_simAgents[i] )
                    continue;

                m_simAgents[i]->reset();
            }
        }

        m_graphicsApp->begin();
        m_graphicsApp->update();

        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 5.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } );
        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 0.0f, 5.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } );
        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 5.0f }, { 0.0f, 0.0f, 1.0f } );
        

        // render the UI
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        _renderUi();

        ImGui::Render();
        int _ww, _wh;
        glfwGetFramebufferSize( m_graphicsApp->window()->getGLFWwindow(), &_ww, &_wh );
        glViewport( 0, 0, _ww, _wh );
        ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );

        m_graphicsApp->end();
    }

    void ITestApplication::_renderUi()
    {
        // render ui related to agents
        _renderUiAgents();

        // Call some custom render functionality
        _renderUiInternal();
    }

    void ITestApplication::_renderUiAgents()
    {
        // Render selection menu
        ImGui::Begin( "Agents> Select an agent from here" );

        if ( ImGui::BeginCombo( "Agents", m_currentAgentName.c_str() ) )
        {
            for ( size_t i = 0; i < m_simAgents.size(); i++ )
            {
                auto _simAgent = m_simAgents[i];
                bool _isSelected = ( i == m_currentAgentIndx );
                auto _simAgentName = std::string( "agent-" ) + std::to_string( i );

                if ( ImGui::Selectable( _simAgentName.c_str(), _isSelected ) )
                {
                    m_currentAgentIndx = i;
                    m_currentAgentName = _simAgentName;
                }

                if ( _isSelected )
                    ImGui::SetItemDefaultFocus();
            }

            ImGui::EndCombo();
        }

        ImGui::End();

        // Render agent menu
        if ( m_currentAgentIndx != -1 )
        {
            ImGui::Begin( "Agent-playground" );

            auto _joints = m_simAgents[m_currentAgentIndx]->joints();
            for ( size_t j = 0; j < _joints.size(); j++ )
            {
                if ( _joints[j]->type() == mjJNT_HINGE )
                {
                    float _qpos = _joints[j]->getQpos()[0];
                    ImGui::SliderFloat( _joints[j]->name().c_str(),
                                        &_qpos, -3.1415f, 3.1415f );
                    _joints[j]->setQpos( { ( mjtNum )_qpos } );
                }
            }

            ImGui::End();
        }

        // Render bodies menu
        ImGui::Begin( "Bodies-info" );

        for ( size_t i = 0; i < m_simBodies.size(); i++ )
        {
            if ( !m_simBodies[i] )
                continue;

            auto _bodyName = m_simBodies[i]->name();
            auto _comForce = m_simBodies[i]->comForce();
            auto _comTorque = m_simBodies[i]->comTorque();

            ImGui::Text( "COM-force %s: (%.5f, %.5f, %.5f)", _bodyName.c_str(), _comForce.x, _comForce.y, _comForce.z );
            ImGui::Text( "COM-torque %s: (%.5f, %.5f, %.5f)", _bodyName.c_str(), _comTorque.x, _comTorque.y, _comTorque.z );
        }

        ImGui::End();
    }

    void ITestApplication::togglePause()
    {
        m_isRunning = ( m_isRunning ) ? false : true;
    }

    SimBody* ITestApplication::getBody( const std::string& name )
    {
        if ( m_simBodiesMap.find( name ) != m_simBodiesMap.end() )
            return m_simBodiesMap[name];

        return NULL;
    }

}