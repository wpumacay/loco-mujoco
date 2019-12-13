
#include <mujoco_simulation.h>

namespace tysoc {
namespace mujoco {

    /***************************************************************************
    *                                                                          *
    *                          TMjcContactManager Impl.                        *
    *                                                                          *
    ***************************************************************************/

    TMjcContactManager::TMjcContactManager( TScenario* scenario, 
                                            mjModel* mjcModel, 
                                            mjData* mjcData )
    {
        m_scenario = scenario;
        m_mjcModel = mjcModel;
        m_mjcData = mjcData;
    }

    TMjcContactManager::~TMjcContactManager()
    {
        m_scenario = nullptr;
        m_mjcModel = nullptr;
        m_mjcData = nullptr;
    }

    void TMjcContactManager::update()
    {
        /* clear current contacts */
        m_contacts.clear();

        /* start collecting from the backend all available contacts */
        int _nContacts = m_mjcData->ncon;
        for ( size_t i = 0; i < _nContacts; i++ )
        {
            auto _kinContact = TMjcContact();
            auto _contactInfo = m_mjcData->contact[i];

            _kinContact.position = { (float) _contactInfo.pos[0], 
                                     (float) _contactInfo.pos[1], 
                                     (float) _contactInfo.pos[2] };

            _kinContact.rotation = { (float) _contactInfo.frame[0], (float) _contactInfo.frame[3], (float) _contactInfo.frame[6],
                                     (float) _contactInfo.frame[1], (float) _contactInfo.frame[4], (float) _contactInfo.frame[7],
                                     (float) _contactInfo.frame[2], (float) _contactInfo.frame[5], (float) _contactInfo.frame[8] };

            _kinContact.transform.setPosition( _kinContact.position );
            _kinContact.transform.setRotation( _kinContact.rotation );

            int _geomId1 = _contactInfo.geom1;
            int _geomId2 = _contactInfo.geom2;

            if ( _geomId1 == -1 || _geomId2 == -1 )
                continue;

            _kinContact.collider1Name = mj_id2name( m_mjcModel, mjOBJ_GEOM, _geomId1 );
            _kinContact.collider2Name = mj_id2name( m_mjcModel, mjOBJ_GEOM, _geomId2 );

            m_contacts.push_back( _kinContact );
        }
    }

    void TMjcContactManager::reset()
    {
        m_contacts.clear();
    }

    /***************************************************************************
    *                                                                          *
    *                           TMjcSimulation-Impl.                           *
    *                                                                          *
    ***************************************************************************/

    //@HACK: checks that mujoco has been activated only once
    bool TMjcSimulation::HAS_ACTIVATED_MUJOCO = false;

    TMjcSimulation::TMjcSimulation( TScenario* scenarioPtr )
        : TISimulation( scenarioPtr )
    {
        m_mjcModelPtr   = nullptr;
        m_mjcDataPtr    = nullptr;

        m_runtimeType = "mujoco";

        // @todo: make empty.xml into a built-in string to parse
        std::string _emptyModelPath;
        _emptyModelPath += TYSOC_PATH_WORKING_DIR;
        _emptyModelPath += "empty.xml";

        TYSOC_CORE_TRACE( "Simulation-MuJoCo >>> looking for empty.xml at path: {0}", _emptyModelPath );
        m_mjcfResourcesPtr = mjcf::loadGenericModel( _emptyModelPath );

        if ( !m_scenarioPtr )
            m_scenarioPtr = new TScenario();

        auto _singleBodies = m_scenarioPtr->getSingleBodies();
        for ( auto _singleBody : _singleBodies )
        {
            auto _singleBodyAdapter = new TMjcBodyAdapter( _singleBody );
            _singleBody->setAdapter( _singleBodyAdapter );

            m_bodyAdapters.push_back( _singleBodyAdapter );

            auto _collision = _singleBody->collision();
            if ( _collision )
            {
                auto _collisionAdapter = new TMjcCollisionAdapter( _collision );
                _collision->setAdapter( _collisionAdapter );

                m_collisionAdapters.push_back( _collisionAdapter );
            }
        }

        auto _agents = m_scenarioPtr->getAgents();
        for ( auto _agent : _agents )
            m_agentWrappers.push_back( new TMjcKinTreeAgentWrapper( _agent ) );

        auto _terrainGens = m_scenarioPtr->getTerrainGenerators();
        for ( auto _terrainGen : _terrainGens )
        {
            auto _terrainGenAdapter = new TMjcTerrainGenWrapper( _terrainGen );
            _terrainGenAdapter->setMjcfTargetElm( m_mjcfResourcesPtr );

            m_terrainGenWrappers.push_back( _terrainGenAdapter );
        }
    }

    TMjcSimulation::~TMjcSimulation()
    {
        if ( m_mjcfResourcesPtr )
            delete m_mjcfResourcesPtr;

        if ( m_contactManager )
            delete m_contactManager;

        if ( m_mjcDataPtr )
            mj_deleteData( m_mjcDataPtr );

        if ( m_mjcModelPtr )
            mj_deleteModel( m_mjcModelPtr );

        m_mjcfResourcesPtr = nullptr;
        m_mjcDataPtr = nullptr;
        m_mjcModelPtr = nullptr;
    }

    bool TMjcSimulation::_initializeInternal()
    {
        /* collect all resources from the adapters */
        for ( auto _terrainGenAdapter : m_terrainGenWrappers )
            _terrainGenAdapter->initialize();// Injects terrain resources into m_mjcfResourcesPtr
        for ( auto _agentAdapter : m_agentWrappers )
            _collectResourcesFromAgentAdapter( dynamic_cast< TMjcKinTreeAgentWrapper* >( _agentAdapter ) );
        for ( auto _singleBodyAdapter : m_bodyAdapters )
            _collectResourcesFromBodyAdapter( dynamic_cast< TMjcBodyAdapter* >( _singleBodyAdapter ) );

        /* insert all collected mesh assets into the mjcf simulation resource */
        auto _assetsSimulationElmPtr = mjcf::findFirstChildByType( m_mjcfResourcesPtr, "asset" );
        std::set< std::string > _currentAssetsNames; // to keep assets without repetition
        for ( auto _meshAsset : m_mjcfMeshResources )
        {
            auto _meshAssetElmName = _meshAsset->getAttributeString( "name" );
            if ( _currentAssetsNames.find( _meshAssetElmName ) != _currentAssetsNames.end() )
                continue;

            _currentAssetsNames.emplace( _meshAssetElmName );
            _assetsSimulationElmPtr->children.push_back( _meshAsset );
        }

        /* Inject resources into the workspace xml *****************************/
        mjcf::saveGenericModel( m_mjcfResourcesPtr, "simulation.xml" );

        /* Initialize mujoco ***************************************************/
        if ( !TMjcSimulation::HAS_ACTIVATED_MUJOCO )
        {
            mj_activate( MUJOCO_LICENSE_FILE );
            TMjcSimulation::HAS_ACTIVATED_MUJOCO = true;
        }

        char _error[1000];
        m_mjcModelPtr = mj_loadXML( "simulation.xml", nullptr, _error, 1000 );
        if ( !m_mjcModelPtr )
        {
            std::cout << "ERROR> could not initialize mjcAPI" << std::endl;
            std::cout << "ERROR> " << _error << std::endl;
            return false;
        }

        m_mjcDataPtr = mj_makeData( m_mjcModelPtr );

        /* Pass mjc structures references to the wrappers **********************/

        for ( auto _terrainGenAdapter : m_terrainGenWrappers )
        {
            auto _mujocoTerrainGenAdapter = dynamic_cast< TMjcTerrainGenWrapper* >( _terrainGenAdapter );

            _mujocoTerrainGenAdapter->setMjcModel( m_mjcModelPtr );
            _mujocoTerrainGenAdapter->setMjcData( m_mjcDataPtr );
        }

        for ( auto _agentAdapter : m_agentWrappers )
        {
            auto _mujocoAgentAdapter = dynamic_cast< TMjcKinTreeAgentWrapper* >( _agentAdapter );

            _mujocoAgentAdapter->setMjcModel( m_mjcModelPtr );
            _mujocoAgentAdapter->setMjcData( m_mjcDataPtr );
            _mujocoAgentAdapter->finishedCreatingResources();
        }

        for ( auto _singleBodyAdapter : m_bodyAdapters )
        {
            auto _mujocoBodyAdapter = dynamic_cast< TMjcBodyAdapter* >( _singleBodyAdapter );

            _mujocoBodyAdapter->setMjcModel( m_mjcModelPtr );
            _mujocoBodyAdapter->setMjcData( m_mjcDataPtr );
            _mujocoBodyAdapter->setMjcBodyId( mj_name2id( m_mjcModelPtr, mjOBJ_BODY, _singleBodyAdapter->body()->name().c_str() ) );
            _mujocoBodyAdapter->onResourcesCreated();
        }

        for ( auto _collisionAdapter : m_collisionAdapters )
        {
            auto _mujocoCollisionAdapter = dynamic_cast< TMjcCollisionAdapter* >( _collisionAdapter );

            _mujocoCollisionAdapter->setMjcModel( m_mjcModelPtr );
            _mujocoCollisionAdapter->setMjcData( m_mjcDataPtr );
            _mujocoCollisionAdapter->setMjcGeomId( mj_name2id( m_mjcModelPtr, mjOBJ_GEOM, _collisionAdapter->collision()->name().c_str() ) );
            _mujocoCollisionAdapter->onResourcesCreated();
        }

        /* create contact manager */
        m_contactManager = new TMjcContactManager( m_scenarioPtr, m_mjcModelPtr, m_mjcDataPtr );

        TYSOC_CORE_TRACE( "Simulation-MuJoCo >>> total-nq: {0}", m_mjcModelPtr->nq );
        TYSOC_CORE_TRACE( "Simulation-MuJoCo >>> total-nv: {0}", m_mjcModelPtr->nv );
        TYSOC_CORE_TRACE( "Simulation-MuJoCo >>> total-nu: {0}", m_mjcModelPtr->nu );
        TYSOC_CORE_TRACE( "Simulation-MuJoCo >>> total-nbody: {0}", m_mjcModelPtr->nbody );
        TYSOC_CORE_TRACE( "Simulation-MuJoCo >>> total-njnt: {0}", m_mjcModelPtr->njnt );
        TYSOC_CORE_TRACE( "Simulation-MuJoCo >>> total-ngeom: {0}", m_mjcModelPtr->ngeom );
        TYSOC_CORE_TRACE( "Simulation-MuJoCo >>> total-nsensor: {0}", m_mjcModelPtr->nsensor );

        return true;
    }

    void TMjcSimulation::_collectResourcesFromBodyAdapter( TMjcBodyAdapter* bodyAdapter )
    {
        /* world-body element where to place the resources of the body */
        auto _worldBody4Body = new mjcf::GenericElement( "worldbody" );

        /* collect resources from the body adapter (bodies, geoms, joints, etc.) */
        _worldBody4Body->children.push_back( bodyAdapter->mjcfResource() );
        m_mjcfResourcesPtr->children.push_back( _worldBody4Body );

        /* collect mesh-assets from the body adapter (used by colliders) */
        auto _bodyXmlAssetResources = bodyAdapter->mjcfAssetResources();
        for ( auto _meshAsset : _bodyXmlAssetResources->children )
            m_mjcfMeshResources.push_back( _meshAsset );
    }

    void TMjcSimulation::_collectResourcesFromAgentAdapter( TMjcKinTreeAgentWrapper* agentAdapter )
    {
        auto _agentXmlResource = agentAdapter->mjcfResource();

        /* grab the mjcf resources required by this agent */
        auto _worldBodyInAgentElmPtr = mjcf::findFirstChildByType( _agentXmlResource, "worldbody" );
        auto _actuatorsInAgentElmPtr = mjcf::findFirstChildByType( _agentXmlResource, "actuator" );
        auto _sensorsInAgentElmPtr   = mjcf::findFirstChildByType( _agentXmlResource, "sensor" );
        auto _contactsInAgentElmPtr  = mjcf::findFirstChildByType( _agentXmlResource, "contact" );

        if ( _contactsInAgentElmPtr )
            m_mjcfResourcesPtr->children.push_back( _contactsInAgentElmPtr );

        if ( _worldBodyInAgentElmPtr )
            m_mjcfResourcesPtr->children.push_back( _worldBodyInAgentElmPtr );

        if ( _actuatorsInAgentElmPtr )
            m_mjcfResourcesPtr->children.push_back( _actuatorsInAgentElmPtr );

        if ( _sensorsInAgentElmPtr )
            m_mjcfResourcesPtr->children.push_back( _sensorsInAgentElmPtr );

        /* grab the mjcf mesh assets required by this agent's components */
        auto _agentXmlAssetResources = agentAdapter->mjcfAssetResources();
        for ( auto _meshAsset : _agentXmlAssetResources->children )
            m_mjcfMeshResources.push_back( _meshAsset );
    }

    void TMjcSimulation::_collectResourcesFromTerrainGenAdapter( TMjcTerrainGenWrapper* terrainAdapter )
    {
        // @wip
    }

    void TMjcSimulation::_preStepInternal()
    {
    #ifdef TYSOC_DEMO
        // @demo: testing sensor-camera view
        if ( m_visualizerPtr && DEMO_OPTIONS.useDemoCameraSensors )
        {
            auto _agents = m_scenarioPtr->getAgents();
            if ( _agents.size() > 0 )
            {
                auto _rootBody = _agents.front()->getRootBody();
                if ( _rootBody )
                    m_visualizerPtr->setSensorsView( _rootBody->worldTransform * TMat4::fromPositionAndRotation( { 0.25f, 0.0f, 0.0f }, TMat3() ) );
            }
        }

        // @demo: collect contacts here (should move to base)
        if ( m_contactManager && DEMO_OPTIONS.useDemoContactManager )
        {
            m_contactManager->update();

            if ( m_visualizerPtr )
            {
                auto _contacts = m_contactManager->contacts();
                for ( auto& _kinContact : _contacts )
                {
                    m_visualizerPtr->drawSolidCylinderX( 0.1, 0.025, _kinContact.transform, { 0.1f, 0.4f, 0.9f, 1.0f } );
                    m_visualizerPtr->drawSolidArrowX( 0.5, _kinContact.transform, { 0.4f, 0.9f, 0.1f, 1.0f } );
                }
            }
        }
    #endif
    }

    void TMjcSimulation::_simStepInternal()
    {
        const double _targetFps = 1.0 / 60.0;
        const mjtNum _simstart = m_mjcDataPtr->time;
        // int _steps = 0;
        while ( m_mjcDataPtr->time - _simstart < _targetFps )
        {
            // _steps++;
            mj_step( m_mjcModelPtr, m_mjcDataPtr );
        }
        // std::cout << "nsteps: " << _steps << std::endl;
    }

    void TMjcSimulation::_postStepInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }
    
    void TMjcSimulation::_resetInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr )
    {
        TYSOC_CORE_INFO( "Simulation-MuJoCo >>> creating simulation" );
        return new TMjcSimulation( scenarioPtr );
    }
}}