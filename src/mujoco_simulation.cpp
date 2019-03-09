
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

        auto _agents = m_scenarioPtr->getAgentsByType( agent::AGENT_TYPE_KINTREE );
        for ( size_t q = 0; q < _agents.size(); q++ )
        {
            auto _agentWrapper = new TMjcKinTreeAgentWrapper( (agent::TAgentKinTree*) _agents[q],
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
        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
            m_agentWrappers[q]->initialize();// Injects agent resources into m_mjcfResourcesPtr

        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
            m_terrainGenWrappers[q]->initialize();// Injects terrain resources into m_mjcfResourcesPtr

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
        }

        std::cout << "nq: " << m_mjcModelPtr->nq << std::endl;
        std::cout << "nv: " << m_mjcModelPtr->nv << std::endl;
        std::cout << "nu: " << m_mjcModelPtr->nu << std::endl;
        std::cout << "nbody: " << m_mjcModelPtr->nbody << std::endl;
        std::cout << "njnt: " << m_mjcModelPtr->njnt << std::endl;
        std::cout << "ngeom: " << m_mjcModelPtr->ngeom << std::endl;
        std::cout << "nsensor: " << m_mjcModelPtr->nsensor << std::endl;

        return true;
    }

    void TMjcSimulation::_preStepInternal()
    {
        // collect terrain generaion info by letting ...
        // the terrain wrappers do the job
        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
        {
            m_terrainGenWrappers[q]->preStep();
        }

        // Collect actuator controls by letting ...
        // the kintree agent wrappers do the job
        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
        {
            m_agentWrappers[q]->preStep();
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
        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
        {
            m_agentWrappers[q]->postStep();
        }

        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
        {
            m_terrainGenWrappers[q]->postStep();
        }
    }
    
    void TMjcSimulation::_resetInternal()
    {
        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
        {
            m_agentWrappers[q]->reset();
        }

        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
        {
            m_terrainGenWrappers[q]->reset();
        }
    }

    std::map< std::string, std::vector<TScalar> > TMjcSimulation::_getVectorizedInfoInternal()
    {
        if ( !m_mjcModelPtr || !m_mjcDataPtr )
            return std::map< std::string, std::vector<TScalar> >();

        std::vector<TScalar> _qpos;
        for ( size_t i = 0; i < m_mjcModelPtr->nq; i++ )
            _qpos.push_back( (TScalar) m_mjcDataPtr->qpos[i] );

        std::vector<TScalar> _qvel;
        for ( size_t i = 0; i < m_mjcModelPtr->nv; i++ )
            _qvel.push_back( (TScalar) m_mjcDataPtr->qvel[i] );

        std::vector<TScalar> _comForcesExt;
        for ( size_t i = 0; i < m_mjcModelPtr->nbody; i++ )
        {
            _comForcesExt.push_back( (TScalar) m_mjcDataPtr->cfrc_ext[6 * i + 0] );
            _comForcesExt.push_back( (TScalar) m_mjcDataPtr->cfrc_ext[6 * i + 1] );
            _comForcesExt.push_back( (TScalar) m_mjcDataPtr->cfrc_ext[6 * i + 2] );
            _comForcesExt.push_back( (TScalar) m_mjcDataPtr->cfrc_ext[6 * i + 3] );
            _comForcesExt.push_back( (TScalar) m_mjcDataPtr->cfrc_ext[6 * i + 4] );
            _comForcesExt.push_back( (TScalar) m_mjcDataPtr->cfrc_ext[6 * i + 5] );
        }

        std::map< std::string, std::vector<float> > _data;

        _data["qpos"] = _qpos;
        _data["qvel"] = _qvel;
        _data["comForcesExt"] = _comForcesExt;

        return _data;
    }

    void* TMjcSimulation::_constructPayloadInternal( const std::string& type )
    {
        if ( type == "mjModel" )
            return (void*) m_mjcModelPtr;
        else if ( type == "mjData" )
            return (void*) m_mjcDataPtr;
        else if ( type == "mjvScene" )
            return (void*) m_mjcScenePtr;
        else if ( type == "mjvCamera" )
            return (void*) m_mjcCameraPtr;
        else if ( type == "mjvOption" )
            return (void*) m_mjcOptionPtr;

        return NULL;
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr,
                                                const std::string& workingDir )
    {
        std::cout << "INFO> creating mujoco simulation" << std::endl;
        return new TMjcSimulation( scenarioPtr, workingDir );
    }
}}