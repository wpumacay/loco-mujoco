
// Includes from core functionality
#include <runtime.h>
#include <model_loader.h>
// Includes from mujoco functionality
#include <mujoco_common.h>

static std::string VISUALIZER_TYPE = "custom";// custom visualizer using cat1 engine

int main( int argc, const char** argv )
{
    if ( argc > 1 )
    {
        try
        {
            VISUALIZER_TYPE = std::string( argv[1] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> should pass which visualizer to use" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }
    }

    auto _modelLoader = tysoc::TModelLoader::Create();
    auto _modelData = _modelLoader->getMjcfModel( "humanoid" );
    auto _agent = tysoc::agent::createKinTreeAgent( "agent0", { 0.0f, 0.0f, 1.0f }, _modelData );

    auto _scenario = new tysoc::TScenario();
    _scenario->addAgent( _agent );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO, 
                                         tysoc::config::rendering::MJCVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _simulation->scenario() );
    _visualizer->initialize();

    while ( _visualizer->isActive() )
    {
        _simulation->step();

        _visualizer->update();
    }

    return 0;
}