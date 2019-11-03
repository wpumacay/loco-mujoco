
#include <runtime.h>
#include <model_loader.h>
#include <mujoco_config.h>
#include <random>

std::default_random_engine              g_randomGenerator;
std::uniform_real_distribution<double>  g_randomUniformDistribution = std::uniform_real_distribution<double>( -2.0, 2.0 );

#define NUM_BOXES       1
#define NUM_SPHERES     1
#define NUM_CYLINDERS   1
#define NUM_CAPSULES    1
#define NUM_MESHES      1

tysoc::TBody* createHfield( const std::string& name, const tysoc::TVec3& position )
{
    const int nxSamples = 50;
    const int nySamples = 50;
    const float xExtent = 5.0f;
    const float yExtent = 5.0f;

    float _maxHeight = 0.0f;
    std::vector< float > _heightData;
    for ( size_t i = 0; i < nxSamples; i++ )
    {
        for ( size_t j = 0; j < nySamples; j++ )
        {
            float _x = xExtent * ( ( (float) i ) / nxSamples - 0.5f );
            float _y = yExtent * ( ( (float) j ) / nySamples - 0.5f );

            float _z = 10.0f * ( _x * _x + _y * _y ) / ( xExtent * xExtent + yExtent * yExtent );

            // float _u = _x * 2.0f;
            // float _v = _y * 2.0f;
            // float _z = std::cos( std::sqrt( ( _u * _u + _v * _v ) ) );

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

tysoc::TBody* createSimpleBody( const std::string& name, const std::string& type )
{
    tysoc::TCollisionData _collisionData;
    tysoc::TVisualData _visualData;
    tysoc::TBodyData _bodyData;

    if ( type == "box" )
    {
        _collisionData.type = tysoc::eShapeType::BOX;
        _collisionData.size = { 0.2, 0.2, 0.4 };

        _visualData.type = tysoc::eShapeType::BOX;
        _visualData.size = { 0.2, 0.2, 0.4 };
    }
    else if ( type == "sphere" )
    {
        _collisionData.type = tysoc::eShapeType::SPHERE;
        _collisionData.size = { 0.1, 0.1, 0.1 };

        _visualData.type = tysoc::eShapeType::SPHERE;
        _visualData.size = { 0.1, 0.1, 0.1 };
    }
    else if ( type == "cylinder" )
    {
        _collisionData.type = tysoc::eShapeType::CYLINDER;
        _collisionData.size = { 0.1, 0.2, 0.1 };

        _visualData.type = tysoc::eShapeType::CYLINDER;
        _visualData.size = { 0.1, 0.2, 0.1 };
    }
    else if ( type == "capsule" )
    {
        _collisionData.type = tysoc::eShapeType::CAPSULE;
        _collisionData.size = { 0.1, 0.2, 0.1 };

        _visualData.type = tysoc::eShapeType::CAPSULE;
        _visualData.size = { 0.1, 0.2, 0.1 };
    }
    else if ( type == "mesh" )
    {
        _collisionData.type = tysoc::eShapeType::MESH;
        _collisionData.size = { 0.2, 0.2, 0.2 };
        _collisionData.filename = std::string( TYSOC_PATH_MESHES_DIR ) + "monkey.stl";

        _visualData.type = tysoc::eShapeType::MESH;
        _visualData.size = { 0.2, 0.2, 0.2 };
        _visualData.filename = std::string( TYSOC_PATH_MESHES_DIR ) + "monkey.stl";
    }
    else 
    {
        return NULL;
    }

    _collisionData.density = 1000.0;
    _collisionData.friction = { 1.0, 0.005, 0.0001 };
    _collisionData.collisionGroup = 1;
    _collisionData.collisionMask = 1;

    _visualData.ambient = { 0.7, 0.5, 0.3 };
    _visualData.diffuse = { 0.7, 0.5, 0.3 };
    _visualData.specular = { 0.7, 0.5, 0.3 };
    _visualData.shininess = 50.0f;

    _bodyData.dyntype = tysoc::eDynamicsType::DYNAMIC;
    _bodyData.collisions.push_back( _collisionData );
    _bodyData.visuals.push_back( _visualData );

    // choose a random position
    tysoc::TVec3 _position;
    _position.x = g_randomUniformDistribution( g_randomGenerator );
    _position.y = g_randomUniformDistribution( g_randomGenerator );
    _position.z = 3.0f;

    // choose a random orientation
    tysoc::TVec3 _rotation;
    // _rotation.x = TYSOC_PI / 4.0f;
    // _rotation.y = TYSOC_PI / 4.0f;
    // _rotation.z = TYSOC_PI / 4.0f;

    // create the abstract body
    auto _bodyPtr = new tysoc::TBody( name, 
                                      _bodyData, 
                                      _position, 
                                      tysoc::TMat3::fromEuler( _rotation ) );

    return _bodyPtr;
}

int main()
{
    auto _scenario = new tysoc::TScenario();

    auto _hfield = createHfield( "terrain_0", { 0.0f, 0.0f, 0.0f } );
    _scenario->addBody( _hfield );

    for ( size_t i = 0; i < NUM_BOXES; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "box_" ) + std::to_string( i ), 
                                              "box" ) );
    }

    for ( size_t i = 0; i < NUM_SPHERES; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "sphere_" ) + std::to_string( i ), 
                                              "sphere" ) );
    }

    for ( size_t i = 0; i < NUM_CYLINDERS; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "cylinder_" ) + std::to_string( i ), 
                                              "cylinder" ) );
    }

    for ( size_t i = 0; i < NUM_CAPSULES; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "capsule_" ) + std::to_string( i ), 
                                              "capsule" ) );
    }

    for ( size_t i = 0; i < NUM_MESHES; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "mesh_" ) + std::to_string( i ), 
                                              "mesh" ) );
    }

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO, 
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    _simulation->togglePause();

    while ( _visualizer->isActive() )
    {
        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_P ) )
            _simulation->togglePause();

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_ESCAPE ) )
            break;

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_R ) )
            _simulation->reset();

        _simulation->step();

        _visualizer->update();
    }

    _runtime->destroyVisualizer();
    _runtime->destroySimulation();
    _visualizer = NULL;
    _simulation = NULL;

    return 0;
}
