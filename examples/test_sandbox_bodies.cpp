
// Includes from core functionality
#include <runtime.h>
// Includes from mujoco functionality
#include <mujoco_common.h>

int main()
{

    auto _bbox = new tysoc::sandbox::TBody();
    _bbox->name = "bbox";
    _bbox->type = "box";
    _bbox->size = { 0.2, 0.2, 0.4 };
    _bbox->worldTransform.setPosition( { 1.0, 1.0, 1.0 } );
    _bbox->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _bsphere = new tysoc::sandbox::TBody();
    _bsphere->name = "bsphere";
    _bsphere->type = "sphere";
    _bsphere->size = { 0.2, 0.2, 0.2 };
    _bsphere->worldTransform.setPosition( { 1.0, -1.0, 1.0 } );
    _bsphere->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _bcapsule = new tysoc::sandbox::TBody();
    _bcapsule->name = "bcapsule";
    _bcapsule->type = "capsule";
    _bcapsule->size = { 0.1, 0.4, 0.0 };
    _bcapsule->worldTransform.setPosition( { -1.0, 1.0, 1.0 } );
    _bcapsule->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _bcylinder = new tysoc::sandbox::TBody();
    _bcylinder->name = "bcylinder";
    _bcylinder->type = "cylinder";
    _bcylinder->size = { 0.1, 0.4, 0.0 };
    _bcylinder->worldTransform.setPosition( { -1.0, -1.0, 1.0 } );
    _bcylinder->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _terrainGenStatic = new tysoc::terrain::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "plane", 
                                        { 10.0f, 10.0f, 0.1f }, 
                                        { 0.0f, 0.0f, 0.0f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f },
                                        "chessboard" );

    auto _scenario = new tysoc::TScenario();
    _scenario->addBody( _bbox );
    _scenario->addBody( _bsphere );
    _scenario->addBody( _bcapsule );
    _scenario->addBody( _bcylinder );
    _scenario->addTerrainGenerator( _terrainGenStatic );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO,
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    while ( _visualizer->isActive() )
    {
        _simulation->step();

        _visualizer->update();
    }

    return 0;
}