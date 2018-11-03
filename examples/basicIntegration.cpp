
#include <mujoco.h>
#include <iostream>
#include <string>

#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>

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

    auto _app = engine::LApp::GetInstance();
    auto _scene = _app->scene();
    
    // make a sample mesh just for testing
    auto _mesh = engine::LMeshBuilder::createBox( 0.5f, 0.5f, 0.5f );
    // make a sample camera
    auto _camera = new engine::LFpsCamera( "fixed",
                                           engine::LVec3( 1.0f, 2.0f, -1.0f ),
                                           engine::LVec3( -2.0f, -4.0f, -2.0f ),
                                           engine::LICamera::UP_Z );

    // make a sample light source
    auto _light = new engine::LLightDirectional( engine::LVec3( 0.2, 0.2, 0.2 ), engine::LVec3( 0.8, 0.8, 0.8 ),
                                                 engine::LVec3( 0.05, 0.05, 0.05 ), 0, engine::LVec3( -1, -1, 0 ) );

    // add these components to the scene
    _scene->addCamera( _camera );
    _scene->addLight( _light );
    _scene->addRenderable( _mesh );

    while( _app->isActive() )
    {
        mjtNum simstart = g_data->time;
        while( g_data->time - simstart < 1.0 / 60.0 )
            mj_step( g_model, g_data );

        std::cout << "g_data->time : " << g_data->time << std::endl;
        mj_step( g_model, g_data );

        _app->update();
    }


    mj_deleteData( g_data );
    mj_deleteModel( g_model );
    mj_deactivate();

    return 0;
}