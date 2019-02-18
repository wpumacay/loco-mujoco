
#include <runtime.h>

#ifndef TYSOC_DLLIBS_PATH
    #define TYSOC_DLLIBS_PATH "../" // pointing to the build/ directory
#endif

#ifndef TYSOCMJC_RESOURCES_PATH
    #define TYSOCMJC_RESOURCES_PATH "../../res/"
#endif

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

    auto _scenario = new tysoc::TScenario();

    auto _modelpath = std::string( TYSOCMJC_RESOURCES_PATH ) + std::string( "templates/mjcf/humanoid.xml" );
    auto _modeldata = tysoc::mjcf::loadGenericModel( _modelpath );
    auto _agent = tysoc::agent::createKinTreeAgent( "agent0", { 0.0f, 0.0f, 0.0f }, _modeldata );

    _scenario->addAgent( _agent );

    auto _simLibPath = std::string( TYSOC_DLLIBS_PATH ) + std::string( "libtysocMujoco.so");
    auto _vizLibPath = std::string( TYSOC_DLLIBS_PATH ) + 
                            ( ( VISUALIZER_TYPE == "mujoco") ? std::string( "tysocMujocoViz/libtysocMujocoViz.so" ) :
                                                               std::string( "tysocCustomViz/libtysocCustomViz.so" ) );

    auto _runtime = new tysoc::TRuntime( _simLibPath, _vizLibPath );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer();
    _visualizer->setScenario( _simulation->scenario() );
    _visualizer->initialize();

    while ( _visualizer->isActive() )
    {
        _simulation->step();

        _visualizer->update();
    }

    return 0;
}