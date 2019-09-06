
#include <mujoco_simulation.h>



namespace tysoc {
namespace mujoco {

    //@HACK: checks that mujoco has been activated only once
    bool TMjcSimulation::HAS_ACTIVATED_MUJOCO = false;

    TMjcSimulation::TMjcSimulation( TScenario* scenarioPtr,
                                    const std::string& workingDir )
        : TISimulation( scenarioPtr, workingDir )
    {
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
        m_mjcCameraPtr  = NULL;
        m_mjcOptionPtr  = NULL;

        m_runtimeType = "mujoco";

        std::string _emptyModelPath;
        _emptyModelPath += m_workingDir;
        _emptyModelPath += "empty.xml";

        std::cout << "LOG> empty path: " << _emptyModelPath << std::endl;
        m_mjcfResourcesPtr = mjcf::loadGenericModel( _emptyModelPath );

        if ( !m_scenarioPtr )
            m_scenarioPtr = new TScenario();

        auto _bodies = m_scenarioPtr->getBodies();
        for ( size_t i = 0; i < _bodies.size(); i++ )
        {
            auto _bodyAdapter = new TMjcBodyAdapter( _bodies[i] );
            _bodies[i]->setAdapter( _bodyAdapter );

            m_bodyAdapters.push_back( _bodyAdapter );

            auto _collisions = _bodies[i]->collisions();
            for ( size_t j = 0; j < _collisions.size(); j++ )
            {
                auto _collisionAdapter = new TMjcCollisionAdapter( _collisions[j] );
                _collisions[j]->setAdapter( _collisionAdapter );

                m_collisionAdapters.push_back( _collisionAdapter );
            }
        }

        auto _agents = m_scenarioPtr->getAgents();
        for ( size_t q = 0; q < _agents.size(); q++ )
        {
            auto _agentWrapper = new TMjcKinTreeAgentWrapper( (agent::TAgent*) _agents[q],
                                                              m_workingDir );
            _agentWrapper->setMjcfTargetElm( m_mjcfResourcesPtr );

            m_agentWrappers.push_back( _agentWrapper );
        }

        auto _terraingens = m_scenarioPtr->getTerrainGenerators();
        for ( size_t q = 0; q < _terraingens.size(); q++ )
        {
            auto _terrainGenWrapper = new TMjcTerrainGenWrapper( _terraingens[q],
                                                                 m_workingDir );
            _terrainGenWrapper->setMjcfTargetElm( m_mjcfResourcesPtr );

            m_terrainGenWrappers.push_back( _terrainGenWrapper );
        }
    }

    TMjcSimulation::~TMjcSimulation()
    {
        if ( m_mjcfResourcesPtr )
        {
            delete m_mjcfResourcesPtr;
            m_mjcfResourcesPtr = NULL;
        }

        if ( m_mjcScenePtr )
        {
            mjv_freeScene( m_mjcScenePtr );
            m_mjcScenePtr = NULL;
        }

        if ( m_mjcDataPtr )
        {
            mj_deleteData( m_mjcDataPtr );
            m_mjcDataPtr = NULL;
        }

        if ( m_mjcModelPtr )
        {
            mj_deleteModel( m_mjcModelPtr );
            m_mjcModelPtr = NULL;
        }

        m_mjcOptionPtr = NULL;
        m_mjcCameraPtr = NULL;
    }

    bool TMjcSimulation::_initializeInternal()
    {

        /* Initialize wrappers (to create their internal structures) ***********/
        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
            m_terrainGenWrappers[q]->initialize();// Injects terrain resources into m_mjcfResourcesPtr

        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
            m_agentWrappers[q]->initialize();// Injects agent resources into m_mjcfResourcesPtr

        // Grab resources from single-bodies and inject them into the mjcf global resource
        for ( size_t i = 0; i < m_bodyAdapters.size(); i++ )
        {
            auto _worldBody4Body = new mjcf::GenericElement( "worldbody" );

            auto _bodyAdapter = reinterpret_cast< TMjcBodyAdapter* >( m_bodyAdapters[i] );
            _worldBody4Body->children.push_back( _bodyAdapter->mjcfResource() );

            auto _body = _bodyAdapter->body();
            auto _collisions = _body->collisions();

            for ( size_t j = 0; j < _collisions.size(); j++ )
            {
                auto _collisionAdapter = reinterpret_cast< TMjcCollisionAdapter* >
                                                    ( _collisions[j]->adapter() );

                _bodyAdapter->mjcfResource()->children.push_back( _collisionAdapter->mjcfResource() );
            }

            m_mjcfResourcesPtr->children.push_back( _worldBody4Body );
        }

        /* Inject resources into the workspace xml *****************************/
        std::string _workspaceModelPath;
        _workspaceModelPath += m_workingDir;
        _workspaceModelPath += "workspace.xml";
        std::cout << "LOG> working path: " << _workspaceModelPath << std::endl;
        mjcf::saveGenericModel( m_mjcfResourcesPtr, _workspaceModelPath );

        /* Initialize mujoco ***************************************************/
        if ( !TMjcSimulation::HAS_ACTIVATED_MUJOCO )
        {
            mj_activate( MUJOCO_LICENSE_FILE );
            TMjcSimulation::HAS_ACTIVATED_MUJOCO = true;
        }

        char _error[1000];
        m_mjcModelPtr = mj_loadXML( _workspaceModelPath.c_str(), NULL, _error, 1000 );
        if ( !m_mjcModelPtr )
        {
            std::cout << "ERROR> could not initialize mjcAPI" << std::endl;
            std::cout << "ERROR> " << _error << std::endl;
            return false;
        }

        m_mjcDataPtr = mj_makeData( m_mjcModelPtr );
        m_mjcCameraPtr = new mjvCamera();
        m_mjcOptionPtr = new mjvOption();
        m_mjcScenePtr  = new mjvScene();
        mjv_defaultCamera( m_mjcCameraPtr );
        mjv_defaultOption( m_mjcOptionPtr );

        mjv_makeScene( m_mjcModelPtr, m_mjcScenePtr, 2000 );

        /* Pass mjc structures references to the wrappers **********************/

        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
        {
            auto _mujocoTerrainGenWrapper = reinterpret_cast< TMjcTerrainGenWrapper* >
                                                ( m_terrainGenWrappers[q] );

            _mujocoTerrainGenWrapper->setMjcModel( m_mjcModelPtr );
            _mujocoTerrainGenWrapper->setMjcData( m_mjcDataPtr );
            _mujocoTerrainGenWrapper->setMjcScene( m_mjcScenePtr );
        }

        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
        {
            auto _mujocoAgentWrapper = reinterpret_cast< TMjcKinTreeAgentWrapper* >
                                            ( m_agentWrappers[q] );

            _mujocoAgentWrapper->setMjcModel( m_mjcModelPtr );
            _mujocoAgentWrapper->setMjcData( m_mjcDataPtr );
            _mujocoAgentWrapper->setMjcScene( m_mjcScenePtr );
            _mujocoAgentWrapper->finishedCreatingResources();
        }

        for ( size_t q = 0; q < m_bodyAdapters.size(); q++ )
        {
            auto _bodyAdapter = reinterpret_cast< TMjcBodyAdapter* >( m_bodyAdapters[q] );

            _bodyAdapter->setMjcModel( m_mjcModelPtr );
            _bodyAdapter->setMjcData( m_mjcDataPtr );
            _bodyAdapter->setMjcBodyId( mj_name2id( m_mjcModelPtr, mjOBJ_BODY, _bodyAdapter->body()->name().c_str() ) );
            _bodyAdapter->onResourcesCreated();
        }

        for ( size_t q = 0; q < m_collisionAdapters.size(); q++ )
        {
            auto _collisionAdapter = reinterpret_cast< TMjcCollisionAdapter* >( m_collisionAdapters[q] );

            _collisionAdapter->setMjcModel( m_mjcModelPtr );
            _collisionAdapter->setMjcData( m_mjcDataPtr );
            _collisionAdapter->setMjcGeomId( mj_name2id( m_mjcModelPtr, mjOBJ_GEOM, _collisionAdapter->collision()->name().c_str() ) );
            _collisionAdapter->onResourcesCreated();
        }

        std::cout << "total-nq: " << m_mjcModelPtr->nq << std::endl;
        std::cout << "total-nv: " << m_mjcModelPtr->nv << std::endl;
        std::cout << "total-nu: " << m_mjcModelPtr->nu << std::endl;
        std::cout << "total-nbody: " << m_mjcModelPtr->nbody << std::endl;
        std::cout << "total-njnt: " << m_mjcModelPtr->njnt << std::endl;
        std::cout << "total-ngeom: " << m_mjcModelPtr->ngeom << std::endl;
        std::cout << "total-nsensor: " << m_mjcModelPtr->nsensor << std::endl;

        return true;
    }

    void TMjcSimulation::_preStepInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }

    void TMjcSimulation::_simStepInternal()
    {
        mjtNum _simstart = m_mjcDataPtr->time;
        // int _steps = 0;
        while ( m_mjcDataPtr->time - _simstart < 1.0 / 60.0 )
        {
            // _steps++;
            mj_step( m_mjcModelPtr, m_mjcDataPtr );
        }

        // std::cout << "nsteps: " << _steps << std::endl;

        mjv_updateScene( m_mjcModelPtr, 
                         m_mjcDataPtr, 
                         m_mjcOptionPtr, 
                         NULL,
                         m_mjcCameraPtr,
                         mjCAT_ALL,
                         m_mjcScenePtr );
    }

    void TMjcSimulation::_postStepInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }
    
    void TMjcSimulation::_resetInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr,
                                                const std::string& workingDir )
    {
        std::cout << "INFO> creating mujoco simulation" << std::endl;
        return new TMjcSimulation( scenarioPtr, workingDir );
    }
}}