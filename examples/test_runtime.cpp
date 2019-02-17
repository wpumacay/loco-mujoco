
#include <runtime.h>

#ifndef TYSOCCORE_DLLIBS_PATH
    #define TYSOCCORE_DLLIBS_PATH "../" // pointing to the build/ directory
#endif


int main()
{
    auto _runtime = new tysoc::TRuntime( std::string( TYSOCCORE_DLLIBS_PATH ) + 
                                         std::string( "libtysocMujoco.so") );

    auto _simulation = _runtime->createSimulation();

    return 0;
}