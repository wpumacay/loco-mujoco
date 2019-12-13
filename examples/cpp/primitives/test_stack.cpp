
#include <chrono>
#include <runtime.h>
#include <model_loader.h>
#include <mujoco_config.h>
#include <random>

const size_t STACK_SIZE = 6;
const float SPACING = 0.1f;
const tysoc::TVec3 BODY_SIZE = { 0.1f, 0.2f, 0.3f };
const tysoc::TVec3 EXTENTS = { (STACK_SIZE - 1) * (SPACING + BODY_SIZE.x),
                               (STACK_SIZE - 1) * (SPACING + BODY_SIZE.y),
                               (STACK_SIZE - 1) * (SPACING + BODY_SIZE.z) };

std::default_random_engine                      g_randomGenerator;
std::uniform_real_distribution<TScalar>  g_randomUniformDistribution = std::uniform_real_distribution<TScalar>( 0.5, 1.0 );

// ISSUE: Pure vertical case seems to be a corner case that breaks the collision detector and solver
const tysoc::TVec3 NOISE_POSITION = { 0.0f, 0.0f, 0.0f };
const tysoc::TVec3 NOISE_ROTATION = { 0.0f, 0.0f, 0.0f };

tysoc::TSingleBody* createSingleBody( const std::string& name, 
                                      const tysoc::eShapeType& type,
                                      const tysoc::TVec3& size,
                                      const tysoc::TVec3& position,
                                      const tysoc::TMat3& rotation,
                                      const tysoc::TVec3& color,
                                      const tysoc::eDynamicsType& dyntype = tysoc::eDynamicsType::DYNAMIC );

int main()
{
    auto _scenario = new tysoc::TScenario();

    auto _plane = createSingleBody( "plane_0", 
                                    tysoc::eShapeType::BOX, 
                                    { 10.0f, 10.0f, 0.5f }, 
                                    { 0.0f, 0.0f, -0.25f }, 
                                    tysoc::TMat3(), 
                                    { 0.3f, 0.5f, 0.7f },
                                    tysoc::eDynamicsType::STATIC );
    _scenario->addSingleBody( _plane );

    for ( size_t i = 0; i < STACK_SIZE; i++ )
    {
        for ( size_t j = 0; j < STACK_SIZE; j++ )
        {
            for ( size_t k = 0; k < STACK_SIZE; k++ )
            {
                const std::string _bodyName = std::string( "body_" ) + std::to_string( i ) + "_" + std::to_string( j ) + "_" + std::to_string( k );
                const tysoc::eShapeType _bodyShape = tysoc::eShapeType::CAPSULE;
                const float _posX = i * ( SPACING + BODY_SIZE.x ) - 0.5f * EXTENTS.x;
                const float _posY = j * ( SPACING + BODY_SIZE.y ) - 0.5f * EXTENTS.y;
                const float _posZ = k * ( SPACING + BODY_SIZE.z ) - 0.5f * EXTENTS.z + 3.0f;
                const float _factor = g_randomUniformDistribution( g_randomGenerator );
                const tysoc::TVec3 _position = { _posX + _factor * NOISE_POSITION.x, _posY + _factor * NOISE_POSITION.y, _posZ + _factor * NOISE_POSITION.z };
                const tysoc::TMat3 _rotation = tysoc::TMat3::fromEuler( { _factor * NOISE_ROTATION.x, _factor * NOISE_ROTATION.y, _factor * NOISE_ROTATION.z } );
                _scenario->addSingleBody( createSingleBody( _bodyName,
                                                            _bodyShape, 
                                                            BODY_SIZE, 
                                                            _position,
                                                            _rotation, // it seems extreme case of all vertical blows up the simulation
                                                            { 0.7f, 0.5f, 0.3f } ) );
            }
        }
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
        TYSOC_TRACE( "step-time: {0} ||| fps: {1}", _duration.count(), 1000.0 / _duration.count() );
    }

    _runtime->destroyVisualizer();
    // _runtime->destroySimulation();
    _visualizer = nullptr;
    _simulation = nullptr;

    return 0;
}

tysoc::TSingleBody* createSingleBody( const std::string& name,
                                      const tysoc::eShapeType& type,
                                      const tysoc::TVec3& size,
                                      const tysoc::TVec3& position,
                                      const tysoc::TMat3& rotation,
                                      const tysoc::TVec3& color,
                                      const tysoc::eDynamicsType& dyntype )
{
    tysoc::TCollisionData _collisionData;
    tysoc::TVisualData _visualData;
    tysoc::TBodyData _bodyData;

    _collisionData.type = type;
    _collisionData.size = size;
    _collisionData.density = 1000.0;
    _collisionData.friction = { 1.0, 0.005, 0.0001 };
    _collisionData.collisionGroup = 1;
    _collisionData.collisionMask = 1;

    _visualData.type = type;
    _visualData.size = size;
    _visualData.ambient = color;
    _visualData.diffuse = color;
    _visualData.specular = color;
    _visualData.shininess = 50.0f;

    _bodyData.dyntype = dyntype;
    _bodyData.collision = _collisionData;
    _bodyData.visual = _visualData;

    return new tysoc::TSingleBody( name, _bodyData, position, rotation );
}
