
#include <mujoco.h>
#include <iostream>
#include <string>

char g_error[1000];

mjModel* g_model;
mjData* g_data;

#ifndef MUJOCO_RESOURCES_PATH
    #define MUJOCO_RESOURCES_PATH "../../res/"
#endif

int main()
{
    mj_activate( "/home/gregor/.mujoco/mjkey.txt" );

    std::string _modelPath( MUJOCO_RESOURCES_PATH );
    _modelPath += "mjxml/basic.xml";

    std::cout << "MUJOCO_RESOURCES_PATH: " << MUJOCO_RESOURCES_PATH << std::endl;

    g_model = mj_loadXML( _modelPath.c_str(), NULL, g_error, 1000 );
    if ( !g_model )
    {
        std::cout << "ERROR> " << std::string( g_error ) << std::endl;
        return 1;
    }

    g_data = mj_makeData( g_model );

    while ( g_data->time < 10 )
    {
        // std::cout << "g_data->time : " << g_data->time << std::endl;
        mj_step( g_model, g_data );
    }

    mj_deleteData( g_data );
    mj_deleteModel( g_model );
    mj_deactivate();

    return 0;
}