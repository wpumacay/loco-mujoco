
#include <tysocMjc.h>



namespace tysocMjc
{


    TTysocMjcApi::TTysocMjcApi()
    {
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
        m_mjcCameraPtr  = NULL;
        m_mjcOptionPtr  = NULL;
    }


    TTysocMjcApi::~TTysocMjcApi()
    {
        // @TODO: Check if base gets called

        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            delete m_terrainGenWrappers[i];
        }
        m_terrainGenWrappers.clear();

        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
            delete it->second;
        }
        m_agentWrappers.clear();

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

        mj_deactivate();
    }

    void TTysocMjcApi::addAgentWrapper( TMjcAgentWrapper* agentWrapperPtr )
    {
        m_agentWrappers[ agentWrapperPtr->name() ] = agentWrapperPtr;
    }

    void TTysocMjcApi::addTerrainGenWrapper( TMjcTerrainGenWrapper* terrainGenWrapperPtr )
    {
        m_terrainGenWrappers.push_back( terrainGenWrapperPtr );
    }

    bool TTysocMjcApi::initializeMjcApi()
    {
        // Create mujoco resources and inject into xml *********************

        mjcf::Schema _schema;
        {
            std::string _schemaPath( TYSOCMJC_RESOURCES_PATH );
            _schemaPath += "xml/schema.xml";

            _schema.load( _schemaPath );
        }

        std::string _emptyModelPath( TYSOCMJC_RESOURCES_PATH );
        _emptyModelPath += "xml/empty.xml";

        auto _root = mjcf::loadGenericModel( &_schema, _emptyModelPath );

        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            m_terrainGenWrappers[i]->injectMjcResources( _root );
        }

        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
            it->second->injectMjcResources( _root );
        }

        std::string _workspaceModelPath( TYSOCMJC_RESOURCES_PATH );
        _workspaceModelPath += "xml/workspace.xml";
        mjcf::saveGenericModel( _root, _workspaceModelPath );

        // *****************************************************************

        // Initialize mujoco ***********************************************

        mj_activate( MUJOCO_LICENSE_FILE );

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
            m_terrainGenWrappers[i]->setMjcScene( m_mjcScenePtr );
            m_scenarioPtr->addTerrainGenerator( m_terrainGenWrappers[i]->terrainGenerator() );
        }

        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
            it->second->setMjcModel( m_mjcModelPtr );
            it->second->setMjcData( m_mjcDataPtr );
            it->second->setMjcScene( m_mjcScenePtr );

            m_scenarioPtr->addAgent( it->second->agent() );
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
        // the agent wrappers do the job
        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
            it->second->preStep();
        }
    }

    void TTysocMjcApi::_updateStep()
    {
        mjtNum _simstart = m_mjcDataPtr->time;
        //int _steps = 0;
        while ( m_mjcDataPtr->time - _simstart < 1.0 / 60.0 )
        {
            //_steps++;
            mj_step( m_mjcModelPtr, m_mjcDataPtr );
        }

        //std::cout << "nsteps: " << _steps << std::endl;

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
        // collect bodies and joints information
        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
            it->second->postStep();
        }
    }
}