    
#include <cmath>
#include <mujoco.h>
#include <glfw3.h>

#include "mjcf/mjcf_api.h"
#include "mjcint/mjcint_api.h"

#include "pendulum.h"

#ifndef MUJOCO_RESOURCES_PATH
    #define MUJOCO_RESOURCES_PATH "../../res/"
#endif

#define PENDULUM_NUM_LINKS 2
#define PENDULUM_LINK_LEGTH 0.5f
#define PENDULUM_MODEL_FILE "double_pendulum.xml"


struct MjContext
{
    mjModel*    model;
    mjData*     data;

    // visualization
    mjvCamera   camera;
    mjvOption   options;
    mjvScene    scene;
    mjrContext  gpuContext;
};

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

MjContext g_MujocoContext;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData( g_MujocoContext.model, g_MujocoContext.data );
        mj_forward( g_MujocoContext.model, g_MujocoContext.data );
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
    mjv_moveCamera( g_MujocoContext.model, 
                    action, dx/height, dy/height, 
                    &g_MujocoContext.scene, 
                    &g_MujocoContext.camera);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera( g_MujocoContext.model,
                    mjMOUSE_ZOOM, 0, -0.05*yoffset, 
                    &g_MujocoContext.scene, 
                    &g_MujocoContext.camera );
}

int main()
{
    auto _pendulum = pendulum::create( PENDULUM_NUM_LINKS );
    std::string _savefile( MUJOCO_RESOURCES_PATH );
    _savefile += PENDULUM_MODEL_FILE;
    mjcf::saveGenericModel( _pendulum, _savefile );

    mj_activate( "/home/gregor/.mujoco/mjkey.txt" );

    char _error[1000];
    g_MujocoContext.model = mj_loadXML( _savefile.c_str(), NULL, _error, 1000 );
    if ( !g_MujocoContext.model )
    {
        std::cout << "ERROR> " << std::string( _error ) << std::endl;
        return 1;
    }

    g_MujocoContext.data = mj_makeData( g_MujocoContext.model );

    // init GLFW
    if( !glfwInit() )
        mju_error( "Could not initialize GLFW" );

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&g_MujocoContext.camera);
    mjv_defaultOption(&g_MujocoContext.options);
    mjv_defaultScene(&g_MujocoContext.scene);
    mjr_defaultContext(&g_MujocoContext.gpuContext);

    // create scene and context
    mjv_makeScene( g_MujocoContext.model, &g_MujocoContext.scene, 2000);
    mjr_makeContext( g_MujocoContext.model, &g_MujocoContext.gpuContext, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        float _z = PENDULUM_LINK_LEGTH * PENDULUM_NUM_LINKS + 0.5f;
        float _y = 0.0;
        float _x = std::sin( g_MujocoContext.data->time / 2.0f );

        mjcint::setBodyPosition( g_MujocoContext.model, "base", { _x, _y, _z } );

        mjcint::setActuatorCtrl( g_MujocoContext.model, g_MujocoContext.data, 
                                 "torque_0", 2.0f * std::sin( 5.0f * g_MujocoContext.data->time ) );

        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = g_MujocoContext.data->time;
        while( g_MujocoContext.data->time - simstart < 1.0/60.0 )
            mj_step(g_MujocoContext.model, g_MujocoContext.data);

        // auto _pos = getBodyPosition( "base" );

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene( g_MujocoContext.model, 
                         g_MujocoContext.data, 
                         &g_MujocoContext.options, 
                         NULL, &g_MujocoContext.camera, 
                         mjCAT_ALL, &g_MujocoContext.scene);
        mjr_render(viewport, &g_MujocoContext.scene, &g_MujocoContext.gpuContext);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&g_MujocoContext.scene);
    mjr_freeContext(&g_MujocoContext.gpuContext);

    // free MuJoCo model and data, deactivate
    mj_deleteData(g_MujocoContext.data);
    mj_deleteModel(g_MujocoContext.model);
    mj_deactivate();

    return 0;
}
