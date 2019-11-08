
#include <chrono>
#include <runtime.h>
#include <model_loader.h>
#include <mujoco_config.h>

static std::string MODEL_FORMAT = "rlsim";
static std::string MODEL_NAME = "dog3d";

static std::string TYSOC_MJCF_TEMPLATES     = std::string( TYSOC_PATH_MJCF_TEMPLATES );
static std::string TYSOC_URDF_TEMPLATES     = std::string( TYSOC_PATH_URDF_TEMPLATES );
static std::string TYSOC_RLSIM_TEMPLATES    = std::string( TYSOC_PATH_RLSIM_TEMPLATES );

bool USE_HFIELD = true;

tysoc::TBody* createHfield( const std::string& name, const tysoc::TVec3& position )
{
    const int nxSamples = 100;
    const int nySamples = 100;
    const float xExtent = 10.0f;
    const float yExtent = 10.0f;

    float _maxHeight = 0.0f;
    std::vector< float > _heightData;
    for ( size_t i = 0; i < nxSamples; i++ )
    {
        for ( size_t j = 0; j < nySamples; j++ )
        {
            float _x = xExtent * ( ( (float) i ) / nxSamples - 0.5f );
            float _y = yExtent * ( ( (float) j ) / nySamples - 0.5f );

            //// float _z = 10.0f * ( _x * _x + _y * _y ) / ( xExtent * xExtent + yExtent * yExtent );

            float _u = _x * 2.0f;
            float _v = _y * 2.0f;
            float _z = 0.5f * std::cos( std::sqrt( ( _u * _u + _v * _v ) ) );

            _heightData.push_back( _z );

            // book keeping: save the max-height for later normalization
            _maxHeight = std::max( _z, _maxHeight );
        }
    }

    if ( _maxHeight > 0.0f )
    {
        for ( size_t i = 0; i < _heightData.size(); i++ )
            _heightData[i] = _heightData[i] / _maxHeight;
    }

    tysoc::TCollisionData _collisionData;
    _collisionData.type = tysoc::eShapeType::HFIELD;
    _collisionData.size = { xExtent, yExtent, _maxHeight };
    _collisionData.hdata.nWidthSamples = nxSamples;
    _collisionData.hdata.nDepthSamples = nySamples;
    _collisionData.hdata.heightData = _heightData;

    tysoc::TVisualData _visualData;
    _visualData.type = tysoc::eShapeType::HFIELD;
    _visualData.size = { xExtent, yExtent, _maxHeight };
    _visualData.texture = "built_in_chessboard";
    _visualData.hdata.nWidthSamples = nxSamples;
    _visualData.hdata.nDepthSamples = nySamples;
    _visualData.hdata.heightData = _heightData;

    _visualData.ambient     = { 0.2f, 0.3f, 0.4f };
    _visualData.diffuse     = { 0.2f, 0.3f, 0.4f };
    _visualData.specular    = { 0.2f, 0.3f, 0.4f };
    _visualData.shininess   = 50.0f;

    tysoc::TBodyData _bodyData;
    _bodyData.dyntype = tysoc::eDynamicsType::STATIC;
    _bodyData.collisions.push_back( _collisionData );
    _bodyData.visuals.push_back( _visualData );

    return new tysoc::TBody( name, _bodyData, position, tysoc::TMat3() );;
}

tysoc::TAgent* createAgent( const std::string& format,
                                   const std::string& modelName,
                                   const std::string& agentName,
                                   const tysoc::TVec3& position,
                                   const tysoc::TVec3& rotation = tysoc::TVec3() )
{
    auto _modelLoader = tysoc::TModelLoader::Create();

    if ( format == "urdf" )
    {
        auto _modelData = _modelLoader->getUrdfModel( modelName );

        return new tysoc::TAgent( _modelData, agentName, position, rotation );
    }
    else if ( format == "rlsim" )
    {
        auto _modelData = _modelLoader->getRlsimModel( modelName );
        
        return new tysoc::TAgent( _modelData, agentName, position, rotation );
    }
    else if ( format == "mjcf" )
    {
        auto _modelData = _modelLoader->getMjcfModel( modelName );
        
        return new tysoc::TAgent( _modelData, agentName, position, rotation );
    }

    std::cout << "ERROR> format: " << format << " not supported" << std::endl;
    return NULL;
}

int main( int argc, const char** argv )
{
    if ( argc > 2 )
    {
        try
        {
            MODEL_FORMAT = std::string( argv[1] );
            MODEL_NAME = std::string( argv[2] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> should pass FORMAT(mjcf|urdf|rlsim) and MODEL_NAME(see templates)" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }

        if ( argc > 3 )
            USE_HFIELD = ( std::string( argv[3] ) == "hfield" );
    }

    /* ***************************************************************************/
    auto _scenario = new tysoc::TScenario();

    auto _agent0 = createAgent( MODEL_FORMAT, MODEL_NAME, "agent0", { 0.0f, 0.0f, 2.5f } );
    auto _agent1 = createAgent( MODEL_FORMAT, MODEL_NAME, "agent1", { 2.0f, 0.0f, 1.5f } );
    auto _agent2 = createAgent( MODEL_FORMAT, MODEL_NAME, "agent2", { -2.0f, 0.0f, 1.5f } );
    auto _agent3 = createAgent( MODEL_FORMAT, MODEL_NAME, "agent3", { 0.0f, 2.0f, 1.5f } );
    auto _agent4 = createAgent( MODEL_FORMAT, MODEL_NAME, "agent4", { 0.0f, -2.0f, 1.5f } );

    if ( !_agent0 || !_agent1 || !_agent2 || !_agent3 )
    {
        std::cout << "ERROR> (format|model): " 
                  << MODEL_FORMAT << "|" << MODEL_NAME 
                  << " not found" << std::endl;
        return 1;
    }

    if ( USE_HFIELD )
    {
        auto _hfield = createHfield( "terrain_0", { 0.0f, 0.0f, 0.0f } );
        _scenario->addBody( _hfield );
    }
    else
    {
        auto _terrainGenStatic = new tysoc::TStaticTerrainGenerator( "terrainGen0" );
        _terrainGenStatic->createPrimitive( "plane", 
                                            { 10.0f, 10.0f, 0.2f }, 
                                            { 0.0f, 0.0f, 0.0f },
                                            tysoc::TMat3(),
                                            { 0.2f, 0.3f, 0.4f },
                                            "built_in_chessboard" );
        _scenario->addTerrainGenerator( _terrainGenStatic );
    }

    _scenario->addAgent( _agent0 );
    _scenario->addAgent( _agent1 );
    //// _scenario->addAgent( _agent2 );
    //// _scenario->addAgent( _agent3 );
    //// _scenario->addAgent( _agent4 );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO, 
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    _simulation->togglePause();

    while ( _visualizer->isActive() )
    {
        //// auto _start = std::chrono::high_resolution_clock::now();

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_P ) )
            _simulation->togglePause();

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_ESCAPE ) )
            break;

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_R ) )
            _simulation->reset();

        _simulation->step();

        _visualizer->update();

        //// auto _duration = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::high_resolution_clock::now() - _start );
        //// std::cout << "step-time: " << _duration.count() << " ||| fps: " << ( 1000.0 / _duration.count() ) << std::endl;
    }

    _runtime->destroyVisualizer();
    _runtime->destroySimulation();
    _visualizer = NULL;
    _simulation = NULL;

    return 0;
}
