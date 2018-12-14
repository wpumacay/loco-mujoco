
#include <tysocViz.h>



namespace tysoc {
namespace viz {

    TVisualizer::TVisualizer( tysoc::TTysocCommonApi* api )
    {
        m_tysocApiPtr = api;

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
        _scene->addSkybox( _skybox );

        // Initialize UI
        m_uiContext.glfwWindowPtr   = m_glAppPtr->window()->getGLFWwindow();
        tysoc::ui::initUI( m_uiContext );
    }

    TVisualizer::~TVisualizer()
    {
        // @TODO: Check if should delete meshes in scene or here
    }

    void TVisualizer::initialize()
    {
        // Collect geometries from the terrain gens
        // @CHECK: this actually change, but
        auto _terrainGenMaps = m_tysocApiPtr->getScenario()->getTerrainGenerators();

        for ( size_t i = 0; i < _terrainGenMaps.size(); i++ )
        {
            _collectTerrainGenResources( _terrainGenMaps[i] );
        }

        // collect all kintree type agents
        auto _IAgents = m_tysocApiPtr->getScenario()->getAgents();

        for ( size_t i = 0; i < _IAgents.size(); i++ )
        {
            if ( _IAgents[i]->type() == "kintree" )
            {
                _collectKinTreeAgent( (tysoc::agent::TAgentKinTree*) _IAgents[i] );
            }
        }
    }

    void TVisualizer::_collectKinTreeAgent( tysoc::agent::TAgentKinTree* kinTreeAgentPtr )
    {
        // create the kintree viz wrapper
        auto _vizKinTreeWrapper = new tysoc::viz::TVizKinTree( kinTreeAgentPtr,
                                                               m_glScenePtr );
        // and add it to the buffer of kintree vizs
        m_vizKinTreeWrappers.push_back( _vizKinTreeWrapper );
    }

    void TVisualizer::_collectTerrainGenResources( tysoc::terrain::TITerrainGenerator* terrainGenPtr )
    {
        auto _geometries = terrainGenPtr->getPrimitives();
        for ( size_t i = 0; i < _geometries.size(); i++ )
        {
            _cacheTerrainGeometry( _geometries[i] );
        }
    }

    void TVisualizer::_resizeMesh( engine::LMesh* meshPtr, 
                                   tysoc::terrain::TTerrainPrimitive* terrainGeomPtr )
    {
        if ( terrainGeomPtr->geomType == "plane" )
        {
            meshPtr->scale.x = 0.5f * terrainGeomPtr->size.x;
            meshPtr->scale.y = 0.5f * terrainGeomPtr->size.y;
        }
        else if ( terrainGeomPtr->geomType == "sphere" )
        {
            meshPtr->scale.x = terrainGeomPtr->size.x;
        }
        else if ( terrainGeomPtr->geomType == "capsule" )
        {
            meshPtr->scale.x = terrainGeomPtr->size.x;
            meshPtr->scale.y = terrainGeomPtr->size.x;
            meshPtr->scale.z = 0.5f * terrainGeomPtr->size.y;
        }
        else if ( terrainGeomPtr->geomType == "cylinder" )
        {
            meshPtr->scale.x = terrainGeomPtr->size.x;
            meshPtr->scale.y = terrainGeomPtr->size.x;
            meshPtr->scale.z = 0.5f * terrainGeomPtr->size.y;
        }
        else if ( terrainGeomPtr->geomType == "box" )
        {
            meshPtr->scale.x = 0.5f * terrainGeomPtr->size.x;
            meshPtr->scale.y = 0.5f * terrainGeomPtr->size.y;
            meshPtr->scale.z = 0.5f * terrainGeomPtr->size.z;
        }
    }

    void TVisualizer::_setColor( engine::LMesh* meshPtr, float* color )
    {
        auto _material = meshPtr->getMaterial();

        _material->ambient.x = color[0];
        _material->ambient.y = color[1];
        _material->ambient.z = color[2];

        _material->diffuse.x = color[0];
        _material->diffuse.y = color[1];
        _material->diffuse.z = color[2];

        _material->specular.x = color[0];
        _material->specular.y = color[1];
        _material->specular.z = color[2];
    }

    void TVisualizer::_updateSensor( tysoc::sensor::TISensor* sensorPtr )
    {
        auto _measurement = sensorPtr->getSensorMeasurement();

        // @TODO: Move this part to a separate module (kind of ...
        // delegate the functionality of rendering specifics to
        // other submodules of the visualizer)

        if ( _measurement->type == "PathTerrainMeasurement" )
        {
            // draw profile from sensor reading
            auto _pathMeasurement = reinterpret_cast< tysoc::sensor::TSectionsTerrainSensorMeasurement* >
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
            auto _agentMeasurement = reinterpret_cast< tysoc::sensor::TAgentIntrinsicsSensorMeasurement* >
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

    void TVisualizer::_cacheTerrainGeometry( tysoc::terrain::TTerrainPrimitive* terrainGeomPtr )
    {
        engine::LMesh* _glMesh = NULL;

        if ( terrainGeomPtr->geomType == "plane" )
        {
            _glMesh = engine::LMeshBuilder::createPlane( 2.0f, 2.0f );

        }
        else if ( terrainGeomPtr->geomType == "sphere" )
        {
            _glMesh = engine::LMeshBuilder::createSphere( 1.0f );
        }
        else if ( terrainGeomPtr->geomType == "capsule" )
        {
            _glMesh = engine::LMeshBuilder::createCapsule( 1.0f, 2.0f );
        }
        else if ( terrainGeomPtr->geomType == "cylinder" )
        {
            _glMesh = engine::LMeshBuilder::createCylinder( 1.0f, 2.0f );
        }
        else if ( terrainGeomPtr->geomType == "box" )
        {
            _glMesh = engine::LMeshBuilder::createBox( 2.0f,
                                                       2.0f,
                                                       2.0f );
        }
        else
        {
            std::cout << "WARNING> Could not create terrain geometry : " << terrainGeomPtr->type << std::endl;
            _glMesh = NULL;
        }

        if ( _glMesh )
        {
            _resizeMesh( _glMesh, terrainGeomPtr );
            // ok, we can create the wrapper now
            auto _meshWrapper = new TVizTerrainMeshWrapper();
            _meshWrapper->glMesh = _glMesh;
            _meshWrapper->geometry = terrainGeomPtr;
            // add mesh to glengine's scene
            m_glScenePtr->addRenderable( _meshWrapper->glMesh );
            // add it to the wrappers list for later usage
            m_terrainMeshWrappers.push_back( _meshWrapper );
        }

    }

    void TVisualizer::_renderUI()
    {
        // @DIRTY: Change-Refactor this part

        if ( engine::InputSystem::isKeyDown( GLFW_KEY_SPACE ) )
        {
            // @CHECK: Should apply globally, as some cameras will not listen
            m_glScenePtr->getCurrentCamera()->setActiveMode( false );
            m_glAppPtr->window()->enableCursor();

            // @DIRTY: enable UI
            m_uiContext.isUiActive = true;
        }
        else if ( engine::InputSystem::isKeyDown( GLFW_KEY_ENTER ) )
        {
            m_glScenePtr->getCurrentCamera()->setActiveMode( true );
            m_glAppPtr->window()->disableCursor();

            // @DIRTY: enable UI
            m_uiContext.isUiActive = false;
        }

        m_uiContext.vizKinTreePtrs  = m_vizKinTreeWrappers;

        tysoc::ui::renderUI( m_uiContext );
    }

    void TVisualizer::update()
    {
        // @CHECK: I'm assuming all geometries are known at initialization.
        //      1. procedural terrain gnerators intiialize to a fixed number of objects and reuse
        //      2. static terrain generators already have the right amoung of objects to be used
        //  the only issue seems to be the one related to dynamic objects spawning for other tasks.
        //  this could have a workaround by using a fixed number of objects and reusing, but still ...
        //  given the tasks and what other developers want to achieve, it might be necessary to use something ...
        // different, like if an enemy spawns given the tasks conditions, then we might to need these functionality
        // The naive way is easy enough, but requires O(n2) checks between these wrappers and the underlying objects ...
        // so for now we will avoid using it, and see if necessary later when we deal with these scenarios
        // One way could be to make this class not cache resources, but directly linking meshes as needed to the ...
        // underlying objects (if they are requested for usage, like the terrain primitives with its inUse flag)

        // Update cached primitives
        for ( size_t i = 0; i < m_terrainMeshWrappers.size(); i++ )
        {
            _updateTerrainWrapper( m_terrainMeshWrappers[i] );
        }
        for ( size_t i = 0; i < m_vizKinTreeWrappers.size(); i++ )
        {
            _updateVizKinTree( m_vizKinTreeWrappers[i] );
        }

        // @CHECK: For now I will just create a pool for the terrain objects, and reuse it
        // The idea is to first disconnect everything and make the connections every frame as ...
        // the underlying logic requests it

        // for ( size_t i = 0; i < m_terrainMeshWrappers.size(); i++ )
        // {
        //     m_terrainMeshWrappers[i]->geometry = NULL;
        // }

        // render sensors readings
        auto _scenarioPtr = m_tysocApiPtr->getScenario();
        auto _sensors = _scenarioPtr->getSensors();

        for ( size_t i = 0; i < _sensors.size(); i++ )
        {
            _updateSensor( _sensors[i] );
        }

        m_glAppPtr->begin();
        m_glAppPtr->update();

        // render the UI
        _renderUI();

        m_glAppPtr->end();
    }

    bool TVisualizer::isActive()
    {
        return engine::LApp::GetInstance()->isActive();
    }

    void TVisualizer::_updateTerrainWrapper( TVizTerrainMeshWrapper* terrainWrapperPtr )
    {
        _resizeMesh( terrainWrapperPtr->glMesh, terrainWrapperPtr->geometry );

        float _color[3] = { terrainWrapperPtr->geometry->color.r,
                            terrainWrapperPtr->geometry->color.g,
                            terrainWrapperPtr->geometry->color.b };
        _setColor( terrainWrapperPtr->glMesh, _color );

        terrainWrapperPtr->glMesh->setVisibility( terrainWrapperPtr->geometry->inUse );

        terrainWrapperPtr->glMesh->pos.x = terrainWrapperPtr->geometry->pos.x;
        terrainWrapperPtr->glMesh->pos.y = terrainWrapperPtr->geometry->pos.y;
        terrainWrapperPtr->glMesh->pos.z = terrainWrapperPtr->geometry->pos.z;
        
        terrainWrapperPtr->glMesh->rotation.set( 0, 0, terrainWrapperPtr->geometry->rotmat[0] );
        terrainWrapperPtr->glMesh->rotation.set( 0, 1, terrainWrapperPtr->geometry->rotmat[3] );
        terrainWrapperPtr->glMesh->rotation.set( 0, 2, terrainWrapperPtr->geometry->rotmat[6] );
        terrainWrapperPtr->glMesh->rotation.set( 1, 0, terrainWrapperPtr->geometry->rotmat[1] );
        terrainWrapperPtr->glMesh->rotation.set( 1, 1, terrainWrapperPtr->geometry->rotmat[4] );
        terrainWrapperPtr->glMesh->rotation.set( 1, 2, terrainWrapperPtr->geometry->rotmat[7] );
        terrainWrapperPtr->glMesh->rotation.set( 2, 0, terrainWrapperPtr->geometry->rotmat[2] );
        terrainWrapperPtr->glMesh->rotation.set( 2, 1, terrainWrapperPtr->geometry->rotmat[5] );
        terrainWrapperPtr->glMesh->rotation.set( 2, 2, terrainWrapperPtr->geometry->rotmat[8] );
    }

    void TVisualizer::_updateVizKinTree( tysoc::viz::TVizKinTree* vizKinTreePtr )
    {
        vizKinTreePtr->update();
    }

}}