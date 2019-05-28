
// Includes from core functionality
#include <runtime.h>
// Includes from mujoco functionality
#include <mujoco_config.h>

int main()
{

    auto _bbase = new tysoc::sandbox::TBody();
    _bbase->name = "bbase";
    _bbase->type = "sphere";
    _bbase->mass = 1.0;
    _bbase->size = { 0.021, 0.021, 0.021 };
    _bbase->worldTransform.setPosition( { 0.0, 0.0, 2.5 } );
    _bbase->worldTransform.setRotation( tysoc::TVec3( 0.001, 0.001, 0.001 ) );
    _bbase->vel = { 0.1, 0.1, 0.1 };

    auto _jhinge1 = new tysoc::sandbox::TJoint();
    _jhinge1->name = "jhinge1";
    _jhinge1->type = "hinge";
    _jhinge1->axis = { 0, 1, 0 };
    // _jhinge1->limits = { -0.5 * TYSOC_PI, 0.5 * TYSOC_PI };
    _jhinge1->parentBodyPtr = _bbase;
     _bbase->joints.push_back( _jhinge1 );

    auto _bpole = new tysoc::sandbox::TBody();
    _bpole->name = "bpole";
    _bpole->type = "capsule";
    _bpole->mass = 1.0;
    _bpole->size = { 0.02, 0.5, 0.0 };
    _bpole->color = { 0.75, 0.5, 0.25 };
    _bpole->parentBodyPtr = _bbase;
    _bbase->bodies.push_back( _bpole );
    _bpole->relTransform.setPosition( { 0.0, 0.0, 0.25 } );
    _bpole->vel = { 0.1, 0.1, 0.1 };

    auto _bplane = new tysoc::sandbox::TBody();
    _bplane->name = "bplane";
    _bplane->type = "plane";
    _bplane->size = { 10.0, 10.0, 0.1 };
    _bplane->color = { 0.3, 0.3, 0.3 };
    _bplane->worldTransform.setPosition( { 0, 0, 0 } );

    auto _scenario = new tysoc::TScenario();
    _scenario->addBody( _bbase );
    _scenario->addBody( _bplane );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO,
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    bool _running = false;

    while ( _visualizer->isActive() )
    {
        if ( _visualizer->checkSingleKeyPress( 15 ) )
            _running = ( _running ) ? false : true;

        if ( _running )
            _simulation->step();

        _visualizer->update();
    }

    return 0;
}