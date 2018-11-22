
/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

/*
* This sample is based on the basic.cpp sample from mujoco. I've ...
* just added some extra functionality on top, and is found by the @ADD tag
*/

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <iostream>


#include <tysocMjc.h>

mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

tysocMjc::TTysocMjcApi* g_tysocApi = NULL;

static int NUM_AGENTS = 5;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData( g_tysocApi->getMjcModel(), g_tysocApi->getMjcData() );
        mj_forward( g_tysocApi->getMjcModel(), g_tysocApi->getMjcData() );
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera( g_tysocApi->getMjcModel(), 
                    action, 
                    dx/height, 
                    dy/height, 
                    g_tysocApi->getMjcScene(), 
                    g_tysocApi->getMjcCamera() );
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera( g_tysocApi->getMjcModel(), 
                    mjMOUSE_ZOOM, 0, -0.05*yoffset, 
                    g_tysocApi->getMjcScene(), 
                    g_tysocApi->getMjcCamera() );
}


// main function
int main( int argc, const char** argv ) /* @ADD: removed main params */
{
    if ( argc > 1 )
    {
        try
        {
            NUM_AGENTS = std::stoi( argv[1] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> Should pass an int for numagents" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }
    }

    /* ***************************************************************************/
    g_tysocApi = new tysocMjc::TTysocMjcApi();

    auto _factory = new tysocMjc::TMjcFactory();

    // auto _agent1 = _factory->createAgent( "agent1",
    //                                       "walker",
    //                                       0.0f, 0.0f, 1.3f );
    // auto _agent2 = _factory->createAgent( "agent2",
    //                                       "walker",
    //                                       0.0f, 3.0f, 1.3f );
    // auto _terrain1 = _factory->createTerrainGen( "terrain1",
    //                                              "procedural" );
    // auto _scenario = new tysoc::TScenario();

    // g_tysocApi->setScenario( _scenario );
    // g_tysocApi->addAgentWrapper( _agent1 );
    // g_tysocApi->addAgentWrapper( _agent2 );
    // g_tysocApi->addTerrainGenWrapper( _terrain1 );

    tysocMjc::TGenericParams _terrainParams;
    _terrainParams.set( "sineProfileAmplitude", 2.0f );
    _terrainParams.set( "sineProfilePeriod", 10.0f );
    _terrainParams.set( "sineProfilePhase", 1.57f );
    _terrainParams.set( "profileDeltaX", 0.5f );
    _terrainParams.set( "profileDepth", 1.0f );
    _terrainParams.set( "profileTickness", 0.01f );


    for ( size_t i = 0; i < NUM_AGENTS; i++ )
    {
        // create agent wrapper
        auto _agent = _factory->createAgent( std::string( "walker_" ) + std::to_string( i ),
                                             "walker",
                                             1.0f, i * 2.5f, 4.5f );

        // create agent wrapper
        mjcf::Vec3 _startPosition = { 0.0f, i * 2.5f, 0.0f };
        _terrainParams.set( "startPosition", _startPosition );
        auto _terrain = _factory->createTerrainGen( std::string( "terrain_proc" ) + std::to_string( i ),
                                                    "procedural", _terrainParams );
        
        auto _terrainGen        = _terrain->terrainGenerator();
        auto _terrainGenInfo    = _terrainGen->generatorInfo();
        _terrainGenInfo->trackingpoint.x = 0.0f;
        _terrainGenInfo->trackingpoint.y = i * 2.5f;
        _terrainGenInfo->trackingpoint.z = 0.0f;

        g_tysocApi->addAgentWrapper( _agent );
        g_tysocApi->addTerrainGenWrapper( _terrain );
    }

    auto _scenario = new tysoc::TScenario();
    g_tysocApi->setScenario( _scenario );

    if ( !g_tysocApi->initializeMjcApi() )
    {
        std::cout << "There was an error initializing the MjcApi" << std::endl;
        return 1;
    }

    /* ***************************************************************************/

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize rendering context
    mjr_defaultContext(&con);
    mjr_makeContext( g_tysocApi->getMjcModel(), &con, mjFONTSCALE_150 );

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // update api
        g_tysocApi->step();

        auto _terrainGens = g_tysocApi->getTerrainGenerators();
        for ( size_t i = 0; i < _terrainGens.size(); i++ )
        {
            auto _genInfoPtr = _terrainGens[i]->generatorInfo();
            _genInfoPtr->trackingpoint.x += 0.025f;
        }

        for ( size_t i = 0; i < NUM_AGENTS; i++ )
        {
            auto _agentName = std::string( "walker_" ) + std::to_string( i );
            auto _actuatorName = std::string( "mjcact_" ) + _agentName + std::string( "_right_hip" );
            g_tysocApi->setAgentAction( _agentName, _actuatorName, std::cos( g_tysocApi->getMjcData()->time ) );
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // render
        mjr_render(viewport, g_tysocApi->getMjcScene(), &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjr_freeContext(&con);

    delete g_tysocApi;

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
