
#include <tysocMjc.h>



namespace tysoc {
namespace mujoco {

    bool TTysocMjcApi::HAS_ACTIVATED_MUJOCO = false;//@HACK: checks that mujoco has been activated only once

    TTysocMjcApi::TTysocMjcApi()
    {
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
        m_mjcCameraPtr  = NULL;
        m_mjcOptionPtr  = NULL;

        m_apiType = API_TYPE_MUJOCO;
    }


    TTysocMjcApi::~TTysocMjcApi()
    {
        // @TODO: Check if base gets called

        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            delete m_terrainGenWrappers[i];
        }
        m_terrainGenWrappers.clear();

        for ( size_t i = 0; i < m_kinTreeAgentWrappers.size(); i++ )
        {
            delete m_kinTreeAgentWrappers[i];
        }
        m_kinTreeAgentWrappers.clear();

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

        // std::cout << "INFO> Deactivated mujoco backend" << std::endl;
        // mj_deactivate();
    }

    void TTysocMjcApi::addKinTreeAgentWrapper( TMjcKinTreeAgentWrapper* agentKinTreeWrapperPtr )
    {
        m_kinTreeAgentWrappers.push_back( agentKinTreeWrapperPtr );
    }

    void TTysocMjcApi::addTerrainGenWrapper( TMjcTerrainGenWrapper* terrainGenWrapperPtr )
    {
        m_terrainGenWrappers.push_back( terrainGenWrapperPtr );
    }

    bool TTysocMjcApi::initializeMjcApi()
    {
        // Create mujoco resources and inject into xml *********************

        std::string _emptyModelPath( TYSOCMJC_RESOURCES_PATH );
        _emptyModelPath += "xml/empty.xml";

        auto _root = mjcf::loadGenericModel( _emptyModelPath );

        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            m_terrainGenWrappers[i]->injectMjcResources( _root );
        }

        for ( size_t i = 0; i < m_kinTreeAgentWrappers.size(); i++ )
        {
            std::cout << "INFO> Injecting agent resources: " << i << std::endl;
            m_kinTreeAgentWrappers[i]->injectMjcResources( _root );
        }

        std::string _workspaceModelPath( TYSOCMJC_RESOURCES_PATH );
        _workspaceModelPath += "xml/workspace.xml";
        mjcf::saveGenericModel( _root, _workspaceModelPath );

        // *****************************************************************

        // Initialize mujoco ***********************************************
        if ( !TTysocMjcApi::HAS_ACTIVATED_MUJOCO )
        {
            std::cout << "INFO> Trying to activate mujoco backend" << std::endl;
            mj_activate( MUJOCO_LICENSE_FILE );
            std::cout << "INFO> Successfully activated mujoco backend" << std::endl;

            TTysocMjcApi::HAS_ACTIVATED_MUJOCO = true;
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

        // *****************************************************************

        // Initialize all other functionality ******************************

        if ( !m_scenarioPtr )
        {
            // create a default scenario if none given
            m_scenarioPtr = new tysoc::TScenario();
        }

        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            m_terrainGenWrappers[i]->setMjcModel( m_mjcModelPtr );
            m_terrainGenWrappers[i]->setMjcData( m_mjcDataPtr );
            m_terrainGenWrappers[i]->setMjcScene( m_mjcScenePtr );

            if ( !m_scenarioPtr->hasTerrainGen( m_terrainGenWrappers[i]->name() ) )
                m_scenarioPtr->addTerrainGenerator( m_terrainGenWrappers[i]->terrainGenerator() );
        }

        for ( size_t i = 0; i < m_kinTreeAgentWrappers.size(); i++ )
        {
            m_kinTreeAgentWrappers[i]->setMjcModel( m_mjcModelPtr );
            m_kinTreeAgentWrappers[i]->setMjcData( m_mjcDataPtr );
            m_kinTreeAgentWrappers[i]->setMjcScene( m_mjcScenePtr );

            if ( !m_scenarioPtr->hasAgent( m_kinTreeAgentWrappers[i]->name() ) )
                m_scenarioPtr->addAgent( m_kinTreeAgentWrappers[i]->agent() );
        }

        // initialize all underlying base resources
        initialize();

        // Initialize wrappers
        for( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            m_terrainGenWrappers[i]->initialize();
        }

        return true;
    }

    void TTysocMjcApi::_preStep()
    {
        // collect terrain generaion info by letting ...
        // the terrain wrappers do the job
        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            m_terrainGenWrappers[ i ]->preStep();
        }

        // Collect actuator controls by letting ...
        // the kintree agent wrappers do the job
        for ( size_t i = 0; i < m_kinTreeAgentWrappers.size(); i++ )
        {
            m_kinTreeAgentWrappers[i]->preStep();
        }
    }

    void TTysocMjcApi::_updateStep()
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

    void TTysocMjcApi::_postStep()
    {
        for ( size_t i = 0; i < m_kinTreeAgentWrappers.size(); i++ )
        {
            m_kinTreeAgentWrappers[i]->postStep();
        }
    }
    
    // @CHANGE: this change is temporary, should change to a single ...
    // initialization mechanism. Creating wrappers and adding them is ...
    // not the way to go (even adding a wrapper does not add it to the scenatio)
    void TTysocMjcApi::_collectFromScenarioInternal()
    {
        auto _agents = m_scenarioPtr->getAgentsByType( agent::AGENT_TYPE_KINTREE );
        std::cout << "INFO> num agents: " << _agents.size() << std::endl;
        for ( size_t q = 0; q < _agents.size(); q++ )
        {
            auto _agentWrapper = new TMjcKinTreeAgentWrapper( (agent::TAgentKinTree*) _agents[q] );
            m_kinTreeAgentWrappers.push_back( _agentWrapper );
        }
    }

    void TTysocMjcApi::_resetInternal()
    {
        for ( size_t q = 0; q < m_kinTreeAgentWrappers.size(); q++ )
        {
            m_kinTreeAgentWrappers[q]->reset();
        }
    }

}}