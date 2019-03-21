
// Includes from core functionality
#include <runtime.h>
// Includes from mujoco functionality
#include <mujoco_config.h>

int main()
{

    auto _bbox1 = new tysoc::sandbox::TBody();
    _bbox1->name = "bbox1";
    _bbox1->type = "box";
    _bbox1->size = { 0.2, 0.2, 1.0 };
    _bbox1->worldTransform.setPosition( { 1.0, 1.0, 3.0 } );
    _bbox1->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _jhinge1 = new tysoc::sandbox::TJoint();
    _jhinge1->name = "jhinge1";
    _jhinge1->type = "hinge";
    _jhinge1->axis = { 1, 0, 0 };
    _jhinge1->limits = { -180, 180 };
    _jhinge1->parentBodyPtr = _bbox1;
    _bbox1->joints.push_back( _jhinge1 );
    _jhinge1->relTransform.setPosition( { 0.0, 0.0, 0.5 } );

    auto _bbox2 = new tysoc::sandbox::TBody();
    _bbox2->name = "bbox2";
    _bbox2->type = "box";
    _bbox2->size = { 0.2, 0.2, 1.0 };
    _bbox2->color = { 0.75, 0.5, 0.25 };
    _bbox2->parentBodyPtr = _bbox1;
    _bbox1->bodies.push_back( _bbox2 );
    _bbox2->relTransform.setPosition( { 0.0, 0.0, -0.75 } );

    auto _jhinge2 = new tysoc::sandbox::TJoint();
    _jhinge2->name = "jhinge2";
    _jhinge2->type = "hinge";
    _jhinge2->axis = { 1, 0, 0 };
    _jhinge2->limits = { -180, 180 };
    _jhinge2->parentBodyPtr = _bbox2;
    _bbox2->joints.push_back( _jhinge2 );
    _jhinge2->relTransform.setPosition( { 0.0, 0.0, 0.5 } );

    auto _terrainGenStatic = new tysoc::terrain::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "plane", 
                                        { 10.0f, 10.0f, 0.1f }, 
                                        { 0.0f, 0.0f, 0.0f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f },
                                        "chessboard" );

    auto _scenario = new tysoc::TScenario();
    _scenario->addBody( _bbox1 );
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