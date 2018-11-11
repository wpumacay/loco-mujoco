    
#include <mujoco.h>
#include <glfw3.h>

#include "mjcf/mjcf.h"


#ifndef MUJOCO_RESOURCES_PATH
    #define MUJOCO_RESOURCES_PATH "../../res/"
#endif

#define PENDULUM_NUM_LINKS 2
#define PENDULUM_LINK_LEGTH 0.5f
#define PENDULUM_BASE_LINK_NAME "link_"
#define PENDULUM_BASE_JOINT_NAME "joint_"

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

MjContext g_MujocoContext;


mjcf::GenericElement* createPendulum( int numLinks );

mjcf::GenericElement* _createWorldBody();
mjcf::GenericElement* _createPendulumLinks();
mjcf::GenericElement* _createLink( mjcf::GenericElement* parent, int linkIndx = 0 );

mjcf::GenericElement* _createBody( const std::string& name,
                                   const mjcf::Vec3& pos = { 0, 0, 0 },
                                   const mjcf::Vec4& quat = { 0, 0, 0, 1 } );
mjcf::GenericElement* _createGeometry( const std::string& name,
                                       const std::string& type,
                                       const mjcf::Sizef& size,
                                       float mass = 0.0f,
                                       const mjcf::Vec3& pos = { 0, 0, 0 },
                                       const mjcf::Vec4& quat = { 0, 0, 0, 1 } );
mjcf::GenericElement* _createGeometry( const std::string& name,
                                       const std::string& type,
                                       const mjcf::Sizef& size,
                                       const mjcf::Sizef& fromto,
                                       float mass = 0.0f );

int main()
{
    auto _pendulum = createPendulum( 2 );
    mjcf::saveGenericModel( _pendulum, "double_pendulum.xml" );

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

    auto _worldBody = _createWorldBody();

    // attach some visual aspects, base plane and general info
    {
        // floor
        auto _floor = _createBody( "floor" );
        auto _floorGeom = _createGeometry( "floor", "plane", { 3, { 3.0, 3.0, 0.2 } }, 0.0f );
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

mjcf::GenericElement* _createWorldBody()
{
    return new mjcf::GenericElement( "worldbody" );
}


mjcf::GenericElement* _createLink( mjcf::GenericElement* parent, int linkIndx )
{
    auto _link = _createBody( std::string( PENDULUM_BASE_LINK_NAME ) + std::to_string( linkIndx ),
                              { 0, 0, ( ( linkIndx == 0 ) ? 0.0f :-PENDULUM_LINK_LEGTH ) } );
    {
        auto _linkGeom = _createGeometry( std::string( PENDULUM_BASE_LINK_NAME ) + std::to_string( linkIndx ),
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
    auto _baseLink = _createBody( "base", { 0, 0, PENDULUM_NUM_LINKS * PENDULUM_LINK_LEGTH + 0.5f } );
    // add some geometry to this element
    {
        auto _baseGeom = _createGeometry( "base", "cylinder", 
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

    auto _effector = _createGeometry( "endmass", "sphere", 
                                      { 1, { 0.05f } }, 1.0f, 
                                      { 0, 0, -PENDULUM_LINK_LEGTH } );
    _currentLink->children.push_back( _effector );

    return _baseLink;
}

mjcf::GenericElement* _createBody( const std::string& name,
                                   const mjcf::Vec3& pos,
                                   const mjcf::Vec4& quat )
{
    auto _body = new mjcf::GenericElement( "body" );

    _body->setAttributeString( "name", name );
    _body->setAttributeVec3( "pos", pos );
    _body->setAttributeVec4( "quat", quat );

    return _body;
}

mjcf::GenericElement* _createGeometry( const std::string& name,
                                       const std::string& type,
                                       const mjcf::Sizef& size,
                                       float mass,
                                       const mjcf::Vec3& pos,
                                       const mjcf::Vec4& quat )
{
    auto _geometry = new mjcf::GenericElement( "geom" );

    _geometry->setAttributeString( "name", name );
    _geometry->setAttributeString( "type", type );
    _geometry->setAttributeArrayFloat( "size", size );
    _geometry->setAttributeFloat( "mass", mass );
    _geometry->setAttributeVec3( "pos", pos );
    _geometry->setAttributeVec4( "quat", quat );

    return _geometry;
}

mjcf::GenericElement* _createGeometry( const std::string& name,
                                       const std::string& type,
                                       const mjcf::Sizef& size,
                                       const mjcf::Sizef& fromto,
                                       float mass )
{
    auto _geometry = new mjcf::GenericElement( "geom" );

    _geometry->setAttributeString( "name", name );
    _geometry->setAttributeString( "type", type );
    _geometry->setAttributeArrayFloat( "size", size );
    _geometry->setAttributeFloat( "mass", mass );
    _geometry->setAttributeArrayFloat( "fromto", fromto );

    return _geometry;
}

