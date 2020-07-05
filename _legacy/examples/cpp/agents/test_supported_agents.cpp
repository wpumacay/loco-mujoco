
#include <chrono>
#include <runtime.h>
#include <model_loader.h>
#include <mujoco_config.h>

bool USE_HFIELD = false;

tysoc::TSingleBody* createHfield( const std::string& name, const tysoc::TVec3& position )
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
            float _z = 0.2f * std::cos( std::sqrt( ( _u * _u + _v * _v ) ) );

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
    _collisionData.type = tysoc::eShapeType::HEIGHTFIELD;
    _collisionData.size = { xExtent, yExtent, _maxHeight };
    _collisionData.hdata.nWidthSamples = nxSamples;
    _collisionData.hdata.nDepthSamples = nySamples;
    _collisionData.hdata.heightData = _heightData;

    tysoc::TVisualData _visualData;
    _visualData.type = tysoc::eShapeType::HEIGHTFIELD;
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
    _bodyData.collision = _collisionData;
    _bodyData.visual = _visualData;

    return new tysoc::TSingleBody( name, _bodyData, position, tysoc::TMat3() );;
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

int main()
{
    /* ***************************************************************************/
    auto _scenario = new tysoc::TScenario();

    auto _agent0 = createAgent( "mjcf", "ant"       , "agent0", { -4.0f, -2.0f, 2.0f } );
    auto _agent1 = createAgent( "mjcf", "walker"    , "agent1", { -2.0f, -2.0f, 2.0f } );
    auto _agent2 = createAgent( "mjcf", "hopper"    , "agent2", {  0.0f, -2.0f, 2.0f } );
    auto _agent3 = createAgent( "mjcf", "cheetah"   , "agent3", {  2.0f, -2.0f, 2.0f } );
    auto _agent4 = createAgent( "mjcf", "humanoid"  , "agent4", {  4.0f, -2.0f, 2.0f } );

    auto _agent5 = createAgent( "urdf", "laikago"   , "agent6", { -0.5f,  0.0f, 2.0f } );
    auto _agent6 = createAgent( "urdf", "dogbot"   , "agent5", { 0.5f,  0.0f, 2.0f } );

    // auto _agent7 = createAgent( "rlsim", "dog3d"        , "agent7" , { -2.0f, 0.0f, 2.0f } );
    // auto _agent8 = createAgent( "rlsim", "raptor3d"     , "agent8" , { -1.0f, 0.0f, 2.0f } );
    // auto _agent9 = createAgent( "rlsim", "goat3d"       , "agent9" , {  0.0f, 0.0f, 2.0f } );
    auto _agent10 = createAgent( "rlsim", "biped3d"     , "agent10", { -0.5f, 0.0f, 2.0f } );
    auto _agent11 = createAgent( "rlsim", "humanoid3d"  , "agent11", {  0.5f, 0.0f, 2.0f } );

    if ( USE_HFIELD )
    {
        auto _hfield = createHfield( "terrain_0", { 0.0f, 0.0f, 0.0f } );
        _scenario->addSingleBody( _hfield );
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

    // _scenario->addAgent( _agent0 );
    // _scenario->addAgent( _agent1 );
    // _scenario->addAgent( _agent2 );
    // _scenario->addAgent( _agent3 );
    // _scenario->addAgent( _agent4 );

    // _scenario->addAgent( _agent5 );
    // _scenario->addAgent( _agent6 );

    // _scenario->addAgent( _agent7 );
    // _scenario->addAgent( _agent8 );
    // _scenario->addAgent( _agent9 );
    _scenario->addAgent( _agent10 );
    _scenario->addAgent( _agent11 );

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
