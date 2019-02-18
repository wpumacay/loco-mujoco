
#include <runtime.h>

#ifndef TYSOCCORE_DLLIBS_PATH
    #define TYSOCCORE_DLLIBS_PATH "../" // pointing to the build/ directory
#endif


int main()
{
    auto _simLibPath = std::string( TYSOCCORE_DLLIBS_PATH ) + std::string( "libtysocMujoco.so");
    auto _vizLibPath = std::string( TYSOCCORE_DLLIBS_PATH ) + std::string( "tysocCustomViz/libtysocCustomViz.so" );

    auto _runtime = new tysoc::TRuntime( _simLibPath, _vizLibPath );

    auto _simulation = _runtime->createSimulation();
    auto _visualizer = _runtime->createVisualizer();
    _visualizer->setScenario( _simulation->scenario() );

    _simulation->initialize();
    _visualizer->initialize();

    while ( _visualizer->isActive() )
    {
        _visualizer->update();
    }

    return 0;
}