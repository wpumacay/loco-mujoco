
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
    auto _modelData = _modelLoader->getMjcfModel( "ant" );
    auto _agent = tysoc::agent::createKinTreeAgent( "agent0", { 0.0f, 0.0f, 1.0f }, _modelData );
    auto _sensor = new tysoc::sensor::TAgentIntrinsicsSensor( "ss0int", _agent );

    auto _scenario = new tysoc::TScenario();
    _scenario->addAgent( _agent );
    _scenario->addSensor( _sensor );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO, 
                                         tysoc::config::rendering::MJCVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    while ( _visualizer->isActive() )
    {
        _simulation->step();

        _visualizer->update();

//        auto _measurement = ( tysoc::sensor::TAgentIntrinsicsSensorMeasurement* ) _sensor->getSensorMeasurement();
//        auto _forces = _measurement->comForces;
//        auto _torques = _measurement->comTorques;
//
//        std::cout << "numforces: " << _forces.size() << std::endl;
//        std::cout << "numtorques: " << _torques.size() << std::endl;
//
//        for ( size_t i = 0; i < _forces.size(); i++ )
//        {
//            std::cout << "force(" << i << "): " << tysoc::TVec3::toString( _forces[i] ) << std::endl;
//        }
//
//        for ( size_t i = 0; i < _torques.size(); i++ )
//        {
//            std::cout << "torques(" << i << "): " << tysoc::TVec3::toString( _torques[i] ) << std::endl;
//        }


        auto _simVectDataDict = _simulation->getVectorizedInfo();
        auto _comForcesExt = _simVectDataDict["comForcesExt"];
        std::cout << "comForcesExt: [ ";

        for ( size_t i = 0; i < _comForcesExt.size(); i++ )
            std::cout << _comForcesExt[i] << " ";

        std::cout << "]" << std::endl;

    }

    return 0;
}