
// Includes from core functionality
#include <runtime.h>
// Includes from mujoco functionality
#include <mujoco_config.h>

int main()
{

    auto _bbox = new tysoc::sandbox::TBody();
    _bbox->name = "bbox";
    _bbox->type = "box";
    _bbox->size = { 0.2, 0.2, 1.0 };
    _bbox->worldTransform.setPosition( { 1.0, 1.0, 1.0 } );

    auto _jhinge = new tysoc::sandbox::TJoint();
    _jhinge->name = "jhinge";
    _jhinge->type = "hinge";
    _jhinge->axis = { 1, 0, 0 };
    _jhinge->limits = { -180, 180 };
    _jhinge->parentBodyPtr = _bbox;

    _bbox->joints.push_back( _jhinge );

    auto _terrainGenStatic = new tysoc::terrain::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "plane", 
                                        { 10.0f, 10.0f, 0.1f }, 
                                        { 0.0f, 0.0f, 0.0f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f },
                                        "chessboard" );

    auto _scenario = new tysoc::TScenario();
    _scenario->addBody( _bbox );
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