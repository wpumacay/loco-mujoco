
#include <mujoco_simulation.h>

namespace tysoc {
namespace mujoco {

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

        std::cout << "LOG> empty path: " << _emptyModelPath << std::endl;
        m_mjcfResourcesPtr = mjcf::loadGenericModel( _emptyModelPath );

        if ( !m_scenarioPtr )
            m_scenarioPtr = new TScenario();

        auto _bodies = m_scenarioPtr->getBodies();
        for ( auto _body : _bodies )
        {
            auto _bodyAdapter = new TMjcBodyAdapter( _body );
            _body->setAdapter( _bodyAdapter );

            m_bodyAdapters.push_back( _bodyAdapter );

            auto _collisions = _body->collisions();
            for ( auto _collision : _collisions )
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
        for ( auto _bodyAdapter : m_bodyAdapters )
            _collectResourcesFromBodyAdapter( dynamic_cast< TMjcBodyAdapter* >( _bodyAdapter ) );

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

        for ( auto _bodyAdapter : m_bodyAdapters )
        {
            auto _mujocoBodyAdapter = dynamic_cast< TMjcBodyAdapter* >( _bodyAdapter );

            _mujocoBodyAdapter->setMjcModel( m_mjcModelPtr );
            _mujocoBodyAdapter->setMjcData( m_mjcDataPtr );
            _mujocoBodyAdapter->setMjcBodyId( mj_name2id( m_mjcModelPtr, mjOBJ_BODY, _bodyAdapter->body()->name().c_str() ) );
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

        std::cout << "total-nq: " << m_mjcModelPtr->nq << std::endl;
        std::cout << "total-nv: " << m_mjcModelPtr->nv << std::endl;
        std::cout << "total-nu: " << m_mjcModelPtr->nu << std::endl;
        std::cout << "total-nbody: " << m_mjcModelPtr->nbody << std::endl;
        std::cout << "total-njnt: " << m_mjcModelPtr->njnt << std::endl;
        std::cout << "total-ngeom: " << m_mjcModelPtr->ngeom << std::endl;
        std::cout << "total-nsensor: " << m_mjcModelPtr->nsensor << std::endl;

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
        // @debug: testing sensor-camera view
        if ( m_visualizerPtr )
        {
            auto _agent = m_scenarioPtr->getAgents().front();
            if ( _agent )
            {
                auto _rootBody = _agent->getRootBody();
                if ( _rootBody )
                {
                    //// m_visualizerPtr->setSensorsView( _rootBody->worldTransform.getPosition() + TVec3( 0.0f, 0.25f, 0.0f ),
                    ////                                  _rootBody->worldTransform.getPosition() + TVec3( 0.0f, 1.25f, 0.0f ) );
                    m_visualizerPtr->setSensorsView( _rootBody->worldTransform * TMat4::fromPositionAndRotation( { 0.25f, 0.0f, 0.0f }, TMat3() ) );
                }
            }
        }
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
        std::cout << "INFO> creating mujoco simulation" << std::endl;
        return new TMjcSimulation( scenarioPtr );
    }
}}