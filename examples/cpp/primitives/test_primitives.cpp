
#include <runtime.h>
#include <model_loader.h>
#include <mujoco_config.h>
#include <random>

std::default_random_engine              g_randomGenerator;
std::uniform_real_distribution<double>  g_randomUniformDistribution = std::uniform_real_distribution<double>( -2.0, 2.0 );

tysoc::TBody* createSimpleBody( const std::string& name, const std::string& type )
{
    tysoc::TCollisionData _collisionData;
    tysoc::TVisualData _visualData;
    tysoc::TBodyData _bodyData;

    if ( type == "box" )
    {
        _collisionData.type = tysoc::eShapeType::BOX;
        _collisionData.size = { 0.2, 0.2, 0.2 };

        _visualData.type = tysoc::eShapeType::BOX;
        _visualData.size = { 0.2, 0.2, 0.2 };

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
    _bodyData.hasInertia = false;
    _bodyData.collisions.push_back( _collisionData );
    _bodyData.visuals.push_back( _visualData );

    // create the abstract body
    auto _bodyPtr = new tysoc::TBody( name, _bodyData );

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

    _bodyPtr->setPosition( _position );
    _bodyPtr->setRotation( tysoc::TMat3::fromEuler( _rotation ) );

    return _bodyPtr;
}

int main()
{
    auto _terrainGenStatic = new tysoc::terrain::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "box", 
                                        { 10.0f, 10.0f, 0.2f }, 
                                        { 0.0f, 0.0f, -0.1f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f },
                                        "chessboard" );

    auto _scenario = new tysoc::TScenario();
    _scenario->addTerrainGenerator( _terrainGenStatic );

    for ( size_t i = 0; i < 15; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "box_" ) + std::to_string( i ), 
                                              "box" ) );
    }

    for ( size_t i = 0; i < 15; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "sphere_" ) + std::to_string( i ), 
                                              "sphere" ) );
    }

    for ( size_t i = 0; i < 15; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "cylinder_" ) + std::to_string( i ), 
                                              "cylinder" ) );
    }

    for ( size_t i = 0; i < 15; i++ )
    {
        _scenario->addBody( createSimpleBody( std::string( "capsule_" ) + std::to_string( i ), 
                                              "capsule" ) );
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
