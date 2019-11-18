
#include <chrono>
#include <runtime.h>
#include <model_loader.h>
#include <mujoco_config.h>
#include <random>

std::default_random_engine              g_randomGenerator;
std::uniform_real_distribution<double>  g_randomUniformDistribution = std::uniform_real_distribution<double>( -2.0, 2.0 );

#define NUM_BOXES 5
#define NUM_SPHERES 5
#define NUM_CYLINDERS 5
#define NUM_CAPSULES 5
#define NUM_MESHES 5

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
        return nullptr;
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
    _position.z = 3.0;

    // choose a random orientation
    tysoc::TVec3 _rotation;
    _rotation.x = TYSOC_PI * g_randomUniformDistribution( g_randomGenerator ) / 4.;
    _rotation.y = TYSOC_PI * g_randomUniformDistribution( g_randomGenerator ) / 4.;
    _rotation.z = TYSOC_PI * g_randomUniformDistribution( g_randomGenerator ) / 4.;

    // create the abstract body
    auto _bodyPtr = new tysoc::TBody( name, 
                                      _bodyData, 
                                      _position, 
                                      tysoc::TMat3::fromEuler( _rotation ) );

    return _bodyPtr;
}

int main()
{
    auto _terrainGenStatic = new tysoc::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "plane", 
                                        { 10.0f, 10.0f, 0.2f }, 
                                        { 0.0f, 0.0f, 0.0f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f },
                                        "built_in_chessboard" );

    auto _scenario = new tysoc::TScenario();
    _scenario->addTerrainGenerator( _terrainGenStatic );

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
        auto _start = std::chrono::high_resolution_clock::now();

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_P ) )
            _simulation->togglePause();
        else if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_R ) )
            _simulation->reset();
        else if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_ESCAPE ) )
            break;

        _simulation->step();

        _visualizer->update();

        auto _duration = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::high_resolution_clock::now() - _start );
        std::cout << "step-time: " << _duration.count() << " ||| fps: " << ( 1000.0 / _duration.count() ) << std::endl;
    }

    _runtime->destroyVisualizer();
    _runtime->destroySimulation();
    _visualizer = nullptr;
    _simulation = nullptr;

    return 0;
}
