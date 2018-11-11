    
#include <mujoco.h>

#include <cmath>

#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>

#include "mjcf/mjcf_api.h"


#ifndef MUJOCO_RESOURCES_PATH
    #define MUJOCO_RESOURCES_PATH "../../res/"
#endif

#define PENDULUM_NUM_LINKS 5
#define PENDULUM_LINK_LEGTH 0.5f
#define PENDULUM_BASE_LINK_NAME "link_"
#define PENDULUM_BASE_JOINT_NAME "joint_"
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

mjcf::GenericElement* createPendulum( int numLinks );
mjcf::GenericElement* _createPendulumLinks();
mjcf::GenericElement* _createLink( mjcf::GenericElement* parent, int linkIndx = 0 );

void setBodyPosition( const std::string& name, const mjcf::Vec3& pos );
mjcf::Vec3 getBodyPosition( const std::string& name );

void setActuatorCtrl( const std::string& name, float val );

int main()
{
    auto _pendulum = createPendulum( 2 );
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

        setBodyPosition( "base", { _x, _y, _z } );

        setActuatorCtrl( "torque_0", 2.0f * std::sin( 5.0f * g_MujocoContext.data->time ) );

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

mjcf::GenericElement* createPendulum( int numLinks )
{
    auto _root = new mjcf::GenericElement( "mujoco" );

    // attach some general info
    {
        //includes
        auto _incVisual     = new mjcf::GenericElement( "include" );
        auto _incSkybox     = new mjcf::GenericElement( "include" );
        auto _incMaterials  = new mjcf::GenericElement( "include" );

        _incVisual->setAttributeString( "file", "visual.xml" );
        _incSkybox->setAttributeString( "file", "skybox.xml" );
        _incMaterials->setAttributeString( "file", "materials.xml" );

        _root->children.push_back( _incVisual );
        _root->children.push_back( _incSkybox );
        _root->children.push_back( _incMaterials );

        // options
        auto _options = new mjcf::GenericElement( "option" );
        _options->setAttributeFloat( "timestep", 0.02 );
        {
            auto _flagEnergy = new mjcf::GenericElement( "flag" );
            _flagEnergy->setAttributeString( "energy", "enable" );
            _options->children.push_back( _flagEnergy );
        }

        _root->children.push_back( _options );
        
    }

    auto _worldBody = mjcf::_createWorldBody();

    // attach some visual aspects, base plane and general info
    {
        // floor
        auto _floor = mjcf::_createBody( "floor" );
        auto _floorGeom = mjcf::_createGeometry( "floor", "plane", { 3, { 3.0, 3.0, 0.2 } }, 0.0f );
        _floorGeom->setAttributeString( "material", "grid" );
        _floor->children.push_back( _floorGeom );
        _worldBody->children.push_back( _floor );

        // light
        auto _light = new mjcf::GenericElement( "light" );
        _light->setAttributeString( "name", "light" );
        _light->setAttributeVec3( "pos", { 0, 0, PENDULUM_LINK_LEGTH * PENDULUM_NUM_LINKS + 2.0f } );
        _worldBody->children.push_back( _light );

        // camera
        auto _fixedCamera = new mjcf::GenericElement( "camera" );
        _fixedCamera->setAttributeString( "name", "fixed" );
        _fixedCamera->setAttributeVec3( "pos", { 0, -1.5, 2 } );
        _fixedCamera->setAttributeArrayFloat( "xyaxes", { 6, { 1, 0, 0, 0, 1, 1 } } );
        _worldBody->children.push_back( _fixedCamera );
    }

    // create the pendulum
    {
        auto _pendulumBase = _createPendulumLinks();
        _worldBody->children.push_back( _pendulumBase );
    }
    
    // create the actuators
    {
        auto _actuators = new mjcf::GenericElement( "actuator" );

        for ( size_t i = 0; i < PENDULUM_NUM_LINKS; i++ )
        {
            auto _actuator = new mjcf::GenericElement( "motor" );
            _actuator->setAttributeString( "name", std::string( "torque_" ) + std::to_string( i ) );
            _actuator->setAttributeString( "joint", std::string( PENDULUM_BASE_JOINT_NAME ) + std::to_string( i ) );
            _actuator->setAttributeFloat( "gear", 1.0 );
            _actuator->setAttributeArrayFloat( "ctrlrange", { 2, { -2, 2 } } );
            _actuator->setAttributeString( "ctrllimited", "true" );

            _actuators->children.push_back( _actuator );
        }

        _root->children.push_back( _actuators );
    }

    // Attach world body
    _root->children.push_back( _worldBody );

    return _root;
}

mjcf::GenericElement* _createLink( mjcf::GenericElement* parent, int linkIndx )
{
    auto _link = mjcf::_createBody( std::string( PENDULUM_BASE_LINK_NAME ) + std::to_string( linkIndx ),
                                    { 0, 0, ( ( linkIndx == 0 ) ? 0.0f :-PENDULUM_LINK_LEGTH ) } );
    {
        auto _linkGeom = mjcf::_createGeometry( std::string( PENDULUM_BASE_LINK_NAME ) + std::to_string( linkIndx ),
                                                "capsule",
                                                { 1, { 0.02 } },
                                                { 6, { 0, 0, 0, 0, 0, -PENDULUM_LINK_LEGTH } },
                                                0.01f );
        _linkGeom->setAttributeString( "material", "self" );
        _link->children.push_back( _linkGeom );

        auto _linkJoint = new mjcf::GenericElement( "joint" );
        _linkJoint->setAttributeString( "name", std::string( PENDULUM_BASE_JOINT_NAME ) + std::to_string( linkIndx ) );
        _linkJoint->setAttributeString( "type", "hinge" );
        _linkJoint->setAttributeVec3( "axis", { 0, 1, 0 } );
        _linkJoint->setAttributeFloat( "damping", 0.1 );

        _link->children.push_back( _linkJoint );
    }

    parent->children.push_back( _link );

    return _link;
}

mjcf::GenericElement* _createPendulumLinks()
{
    // make the base body
    auto _baseLink = mjcf::_createBody( "base", { 0, 0, PENDULUM_NUM_LINKS * PENDULUM_LINK_LEGTH + 0.5f } );
    // add some geometry to this element
    {
        auto _baseGeom = mjcf::_createGeometry( "base", "cylinder", 
                                                { 1, { 0.025 } },
                                                { 6, { 0.0, -0.1, 0.0, 0.0, 0.1, 0.0 } },
                                                0.0f );
        _baseGeom->setAttributeString( "material", "decoration" );
        _baseLink->children.push_back( _baseGeom );
    }

    // make all other links
    auto _currentLink = _baseLink;
    
    for ( size_t i = 0; i < PENDULUM_NUM_LINKS; i++ )
    {
        _currentLink = _createLink( _currentLink, i );
    }

    auto _effector = mjcf::_createGeometry( "endmass", "sphere", 
                                            { 1, { 0.05f } }, 1.0f, 
                                            { 0, 0, -PENDULUM_LINK_LEGTH } );
    _currentLink->children.push_back( _effector );

    return _baseLink;
}



mjcf::Vec3 getBodyPosition( const std::string& name )
{
    mjcf::Vec3 _res;
    
    auto _id = mj_name2id( g_MujocoContext.model, mjOBJ_BODY, name.c_str() );
    std::cout << "id: " << _id << std::endl;

    if ( _id != -1 )
    {
        _res.x = g_MujocoContext.data->xpos[ 3 * _id + 0 ];
        _res.y = g_MujocoContext.data->xpos[ 3 * _id + 1 ];
        _res.z = g_MujocoContext.data->xpos[ 3 * _id + 2 ];
    }
    else
    {
        std::cout << "body: " << name << " not found" << std::endl;
    }

    return _res;
}

void setBodyPosition( const std::string& name, const mjcf::Vec3& pos )
{
    auto _id = mj_name2id( g_MujocoContext.model, mjOBJ_BODY, name.c_str() );

    if ( _id != -1 )
    {
        g_MujocoContext.model->body_pos[ 3 * _id + 0 ] = pos.x;
        g_MujocoContext.model->body_pos[ 3 * _id + 1 ] = pos.y;
        g_MujocoContext.model->body_pos[ 3 * _id + 2 ] = pos.z;

        // g_MujocoContext.data->xipos[ 3 * _id + 0 ] = pos.x;
        // g_MujocoContext.data->xipos[ 3 * _id + 1 ] = pos.y;
        // g_MujocoContext.data->xipos[ 3 * _id + 2 ] = pos.z;
    }
}

void setActuatorCtrl( const std::string& name, float val )
{
    auto _id = mj_name2id( g_MujocoContext.model, mjOBJ_ACTUATOR, name.c_str() );

    if ( _id != -1 )
    {
        g_MujocoContext.data->ctrl[ _id ] = val;
    }
}