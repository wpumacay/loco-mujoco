
#include <tysocCustomViz.h>


namespace tysoc {
namespace viz {


    TCustomVisualizer::TCustomVisualizer( TTysocCommonApi* tysocApiPtr )
        : TIVisualizer( tysocApiPtr )
    {
        m_glAppPtr      = NULL;
        m_glScenePtr    = NULL;
        m_uiContextPtr  = NULL;
    }

    TCustomVisualizer::~TCustomVisualizer()
    {
        // @TODO: Check if should delete meshes in scene or here
    }

    void TCustomVisualizer::_initializeInternal()
    {
        // set up rendering engine stuff
        _setupGlRenderingEngine();

        // Create visualization wrappers for the terrain generator
        auto _terrainGenerators = m_tysocApiPtr->getScenario()->getTerrainGenerators();

        for ( size_t i = 0; i < _terrainGenerators.size(); i++ )
        {
            _collectTerrainGenerator( _terrainGenerators[i] );
        }

        // Create visualization wrappers for the agents
        auto _agents = m_tysocApiPtr->getScenario()->getAgents();

        for ( size_t i = 0; i < _agents.size(); i++ )
        {
            if ( _agents[i]->type() == "kintree" )
            {
                _collectKinTreeAgent( ( agent::TAgentKinTree* ) _agents[i] );
            }
        }

        //// and finally create the UI
        // first the ui context
        m_uiContextPtr = new TCustomContextUI();
        m_uiContextPtr->isUiActive          = false;
        m_uiContextPtr->isBasicUiActive     = true;
        m_uiContextPtr->glfwWindowPtr       = m_glAppPtr->window()->getGLFWwindow();
        m_uiContextPtr->vizKinTreePtrs      = m_vizKinTreeWrappers;
        // and then the UI
        m_uiPtr = new TCustomUI( m_tysocApiPtr,
                                 m_uiContextPtr );
        m_uiPtr->initUI();
    }

    void TCustomVisualizer::_updateInternal()
    {
        // Update terrain visualization wrappers
        for ( size_t i = 0; i < m_vizTerrainGeneratorWrappers.size(); i++ )
        {
            m_vizTerrainGeneratorWrappers[i]->update();
        }

        // and also the agent visualization wrappers
        for ( size_t i = 0; i < m_vizKinTreeWrappers.size(); i++ )
        {
            m_vizKinTreeWrappers[i]->update();
        }

        // and the sensor readings (render them directly, seems like ...
        // wrapping them would be wasteful?)
        auto _scenarioPtr = m_tysocApiPtr->getScenario();
        auto _sensors = _scenarioPtr->getSensors();

        for ( size_t i = 0; i < _sensors.size(); i++ )
        {
            _renderSensorReading( _sensors[i] );
        }

        // and finally request to the rendering engine *****************

        m_glAppPtr->begin();
        m_glAppPtr->update();

        // render the UI
        _renderUIInternal();

        m_glAppPtr->end();

        // *************************************************************
    }

    void TCustomVisualizer::_renderUIInternal()
    {
        // @DIRTY: Change-Refactor this part

        if ( engine::InputSystem::isKeyDown( GLFW_KEY_SPACE ) )
        {
            // @CHECK: Should apply globally, as some cameras will not listen
            m_glScenePtr->getCurrentCamera()->setActiveMode( false );
            m_glAppPtr->window()->enableCursor();

            // @DIRTY: enable UI
            m_uiContextPtr->isUiActive = true;
        }
        else if ( engine::InputSystem::isKeyDown( GLFW_KEY_ENTER ) )
        {
            m_glScenePtr->getCurrentCamera()->setActiveMode( true );
            m_glAppPtr->window()->disableCursor();

            // @DIRTY: disable UI
            m_uiContextPtr->isUiActive = false;
        }

        m_uiContextPtr->vizKinTreePtrs  = m_vizKinTreeWrappers;

        m_uiPtr->renderUI();
    }

    bool TCustomVisualizer::_isActiveInternal()
    {
        return m_glAppPtr->isActive();
    }

    TIVizCamera* TCustomVisualizer::_createCameraInternal( const std::string& name,
                                                           const std::string& type,
                                                           const TVec3& pos,
                                                           const TMat3& rot )
    {
        // @TODO|@WIP
        return NULL;
    }

    void TCustomVisualizer::_changeToCameraInternal( TIVizCamera* cameraPtr )
    {
        // @TODO|@WIP
    }

    void TCustomVisualizer::_grabCameraFrameInternal( TIVizCamera* cameraPtr,
                                                      TIVizTexture& rgbTexture,
                                                      TIVizTexture& depthTexture )
    {
        // @TODO|@WIP
    }

    TIVizLight* TCustomVisualizer::_createLightInternal( const std::string& name,
                                                         const std::string& type,
                                                         const TVec3& pos )
    {
        // @TODO|@WIP
        return NULL;
    }

    void TCustomVisualizer::_setupGlRenderingEngine()
    {
        auto _app = m_glAppPtr = engine::LApp::GetInstance();
        auto _scene = m_glScenePtr = _app->scene();

        auto _camera = new engine::LFpsCamera( "fps",
                                               engine::LVec3( 1.0f, 2.0f, 1.0f ),
                                               engine::LVec3( -2.0f, -4.0f, -2.0f ),
                                               engine::LICamera::UP_Z );

        // auto _camera = new engine::LFixedCamera3d( "fixed",
        //                                            engine::LVec3( 4.0f, 8.0f, 4.0f ),
        //                                            engine::LVec3( 0.0f, 0.0f, 0.0f ),
        //                                            engine::LICamera::UP_Z );

        // make a sample light source
        auto _light = new engine::LLightDirectional( engine::LVec3( 0.8, 0.8, 0.8 ), 
                                                     engine::LVec3( 0.8, 0.8, 0.8 ),
                                                     engine::LVec3( 0.3, 0.3, 0.3 ), 
                                                     0, 
                                                     engine::LVec3( 0, 0, -1 ) );
        _light->setVirtualPosition( engine::LVec3( 5, 0, 5 ) );

        auto _skybox = new engine::LSkybox( "starfield" );

        // add these components to the scene
        _scene->addCamera( _camera );
        _scene->addLight( _light );
        //_scene->addSkybox( _skybox );
    }

    void TCustomVisualizer::_collectKinTreeAgent( agent::TAgentKinTree* kinTreeAgentPtr )
    {
        // create the kintree viz wrapper
        auto _vizKinTreeWrapper = new TCustomVizKinTree( kinTreeAgentPtr,
                                                   m_glScenePtr );
        // and add it to the buffer of kintree vizs
        m_vizKinTreeWrappers.push_back( _vizKinTreeWrapper );
    }

    void TCustomVisualizer::_collectTerrainGenerator( terrain::TITerrainGenerator* terrainGeneratorPtr )
    {
        // create the terrainGenrator viz wrapper
        auto _vizTerrainGeneratorWrapper = new TCustomVizTerrainGenerator( terrainGeneratorPtr,
                                                                     m_glScenePtr );
        // and add it to the buffer of terrainGenerator vizs
        m_vizTerrainGeneratorWrappers.push_back( _vizTerrainGeneratorWrapper );
    }

    void TCustomVisualizer::_renderSensorReading( sensor::TISensor* sensorPtr )
    {
        auto _measurement = sensorPtr->getSensorMeasurement();

        // @TODO: Move this part to a separate module (kind of ...
        // delegate the functionality of rendering specifics to
        // other submodules of the visualizer)

        if ( _measurement->type == "PathTerrainMeasurement" )
        {
            // draw profile from sensor reading
            auto _pathMeasurement = reinterpret_cast< sensor::TSectionsTerrainSensorMeasurement* >
                                        ( _measurement );

            std::vector< engine::LLine > _lines;
            for ( size_t i = 0; i < ( _pathMeasurement->profile.size() / 3 ); i++ )
            {
                engine::LLine _pline;


                if ( !_pathMeasurement->usesComplement )
                {
                    _pline.start.x = _pathMeasurement->profile[3 * i + 0];
                    _pline.start.y = _pathMeasurement->profile[3 * i + 1];
                    _pline.start.z = _pathMeasurement->agentProjection.z;
                    
                    _pline.end.x = _pathMeasurement->profile[3 * i + 0];
                    _pline.end.y = _pathMeasurement->profile[3 * i + 1];
                    _pline.end.z = _pline.start.z + _pathMeasurement->profile[3 * i + 2];
                }
                else
                {
                    _pline.start.x = _pathMeasurement->profile[3 * i + 0];
                    _pline.start.y = _pathMeasurement->profile[3 * i + 1];
                    _pline.start.z = _pathMeasurement->agentPosition.z;

                    _pline.end.x = _pathMeasurement->profile[3 * i + 0];
                    _pline.end.y = _pathMeasurement->profile[3 * i + 1];
                    _pline.end.z = _pline.start.z - _pathMeasurement->profile[3 * i + 2];
                }

                _lines.push_back( _pline );
            }

            engine::DebugSystem::drawLinesBatch( _lines );
        }
        else if ( _measurement->type == "AgentIntrinsicsMeasurement" )
        {
            auto _agentMeasurement = reinterpret_cast< sensor::TAgentIntrinsicsSensorMeasurement* >
                                        ( _measurement );

            std::vector< engine::LLine > _lines;
            for ( size_t i = 0; i < ( _agentMeasurement->bodiesRelativePosition.size() / 3 ); i++ )
            {
                engine::LLine _pline;

                _pline.start.x = _agentMeasurement->rootPosition.x;
                _pline.start.y = _agentMeasurement->rootPosition.y;
                _pline.start.z = _agentMeasurement->rootPosition.z;

                _pline.end.x = _pline.start.x + _agentMeasurement->bodiesRelativePosition[3 * i + 0];
                _pline.end.y = _pline.start.y + _agentMeasurement->bodiesRelativePosition[3 * i + 1];
                _pline.end.z = _pline.start.z + _agentMeasurement->bodiesRelativePosition[3 * i + 2];


                _lines.push_back( _pline );
            }

            engine::DebugSystem::drawLinesBatch( _lines, { 1.0f, 0.1f, 0.0f } );
        }
    }

}}