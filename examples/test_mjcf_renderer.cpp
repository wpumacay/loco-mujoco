
#include <cmath>
#include <mujoco.h>

#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>

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
};
MjContext g_MujocoContext;

mjcf::GenericElement* g_pendulum;

char g_error[1000];

struct GeomWrapper
{
    std::string     name;
    int             geomId;
    engine::LMesh*  meshObj;
};

// Initialization
void collectGeometries( std::vector< GeomWrapper* >& geometries,
                        mjcf::GenericElement* element )
{
    if ( element->etype == "geom" )
    {
        auto _gname = element->getAttributeString( "name" );
        auto _gtype = element->getAttributeString( "type" );
        auto _gsize = element->getAttributeArrayFloat( "size" );
        auto _gfromto = element->getAttributeArrayFloat( "fromto" );

        // make meshObj
        engine::LMesh* _meshObj = NULL;
        // @TODO: Should definitely pass this into helpers to get vec2s, vec3s, etc.
        if ( _gtype == "plane" )
        {
            // should normalize sizes accordingly
            float _width, _depth;
            if ( _gsize.ndim == 0 )
            {
                // This is weird, but just in case make some default dimenions
                std::cout << "WARNING> you passed a plane with no size ... :/" << std::endl;
                _width = 3.0f;
                _depth = 3.0f;
            }
            else if ( _gsize.ndim == 1 )
            {
                // the dimensions should be repeated
                _width = _depth = _gsize.buff[0];
            }
            else if ( _gsize.ndim >= 2 )
            {
                // each dimensiones has a separate field
                _width = _gsize.buff[0];
                _depth = _gsize.buff[1];
                // third is spacing
            }

            _meshObj = engine::LMeshBuilder::createPlane( _width, _depth );
        }
        else if ( _gtype == "sphere" )
        {
            float _radius;

            if ( _gsize.ndim == 0 )
            {
                // This is weird, but just in case make some default dimenions
                std::cout << "WARNING> you passed a sphere with no size ... :/" << std::endl;
                _radius = 0.1f;
            }
            else if ( _gsize.ndim == 1 )
            {
                _radius = _gsize.buff[0];
            }
            else
            {
                // just in case, if someone passed more parameters than needed
                // it's like ... "thanks, but no thanks xD", so just use the first two
                std::cout << "WARNING> you passed a sphere with more parameters than needed" << std::endl;
                _radius = _gsize.buff[0];
            }

            _meshObj = engine::LMeshBuilder::createSphere( _radius );
        }
        else if ( _gtype == "capsule" ||
                  _gtype == "cylinder" )
        {
            float _radius, _length;

            // This one is a bit trick, because it might have a fromto
            if ( _gfromto.ndim == 6 ) // use fromto
            {
                /*   _____________________
                *   |                     |
                *  |  s                 e  |
                *   |_____________________|
                */
                // first 3 are start point (s)
                float _x1 = _gfromto.buff[0];
                float _y1 = _gfromto.buff[1];
                float _z1 = _gfromto.buff[2];
                // second 3 are the end point (e)
                float _x2 = _gfromto.buff[3];
                float _y2 = _gfromto.buff[4];
                float _z2 = _gfromto.buff[5];
                // get the length of the capsule (the space orientation is ...
                // computed from the scene, so we just use that one)
                float _dx = ( _x2 - _x1 );
                float _dy = ( _y2 - _y1 );
                float _dz = ( _z2 - _z1 );
                _length = sqrtf( _dx * _dx + _dy * _dy + _dz * _dz );

                // get the radius from the size param
                if ( _gsize.ndim == 1 )
                {
                    _radius = _gsize.buff[0];
                }
                else
                {
                    std::cout << "WARNING> wrong size dim for capsule/cylinder with fromto" << std::endl;
                    _radius = 0.25f * _length;
                }
            }
            else
            {
                if ( _gsize.ndim == 2 )
                {
                    _radius = _gsize.buff[0];
                    _length = _gsize.buff[1];
                }
                else
                {
                    // default, just in case passed less than (radius,length)
                    std::cout << "WARNING> wrong size dim for capsule/cylinder" << std::endl;
                    _radius = 0.05f;
                    _length = 0.1f;
                }
            }

            if ( _gtype == "capsule" )
            {
                _meshObj = engine::LMeshBuilder::createCapsule( _radius, _length );
            }
            else
            {
                _meshObj = engine::LMeshBuilder::createCylinder( _radius, _length );
            }
        }
        else if ( _gtype == "box" )
        {
            float _hwidth, _hdepth, _hheight;

            if ( _gsize.ndim == 3 )
            {
                _hwidth  = _gsize.buff[0];
                _hdepth  = _gsize.buff[1];
                _hheight = _gsize.buff[2];
            }
            else
            {
                std::cout << "WARNING> 0 dims passed to size for box creation" << std::endl;
                _hwidth = _hdepth = _hheight = 0.1f;
            }

            _meshObj = engine::LMeshBuilder::createBox( 2.0f * _hwidth, 2.0f * _hdepth, 2.0f * _hheight );
        }

        if ( _meshObj )
        {
            auto _wrappedGeom = new GeomWrapper();
            _wrappedGeom->meshObj = _meshObj;
            _wrappedGeom->name = _gname;

            geometries.push_back( _wrappedGeom );
        }
    }

    // @TODO: There is something weird here. Perhaps the name is not mandatory ...
    // so I would not be able to get the geometry information from the scene :O.
    // We should change this to a collect on demand every update, and if not created, then
    // just create a new one. Still, perhaps this initialization could serve as a pool initialization
    // :( will have to change this later

    for ( size_t i = 0; i < element->children.size(); i++ )
    {
        collectGeometries( geometries, element->children[i] );
    }
}

void _updateGeometry( GeomWrapper* geom, mjvScene* scene )
{
    auto _mjgeom = scene->geoms[ geom->geomId ];

    // set position
    {
        geom->meshObj->pos.x = _mjgeom.pos[0];
        geom->meshObj->pos.y = _mjgeom.pos[1];
        geom->meshObj->pos.z = _mjgeom.pos[2];
        
        
    }
    // set the rotation
    {
        geom->meshObj->rotation.set( 0, 0, _mjgeom.mat[0] );
        geom->meshObj->rotation.set( 0, 1, _mjgeom.mat[1] );
        geom->meshObj->rotation.set( 0, 2, _mjgeom.mat[2] );

        geom->meshObj->rotation.set( 1, 0, _mjgeom.mat[3] );
        geom->meshObj->rotation.set( 1, 1, _mjgeom.mat[4] );
        geom->meshObj->rotation.set( 1, 2, _mjgeom.mat[5] );

        geom->meshObj->rotation.set( 2, 0, _mjgeom.mat[6] );
        geom->meshObj->rotation.set( 2, 1, _mjgeom.mat[7] );
        geom->meshObj->rotation.set( 2, 2, _mjgeom.mat[8] );
    }
}

void updateGeometries( std::vector< GeomWrapper* >& geometries,
                       mjModel* model, mjvScene* scene )
{
    for ( size_t i = 0; i < geometries.size(); i++ )
    {
        auto _wrappedGeom = geometries[i];
        auto _id = mj_name2id( model, mjOBJ_GEOM, _wrappedGeom->name.c_str() );
        if ( _id != -1 )
        {
            _wrappedGeom->geomId = _id;
            // will check the api, just a sec
            _updateGeometry( _wrappedGeom, scene );
        }
    }
}

int main()
{
    // Create pendulum and initialize some mujoco
    {
        g_pendulum = pendulum::create( PENDULUM_NUM_LINKS );
        std::string _savefile( MUJOCO_RESOURCES_PATH );
        _savefile += PENDULUM_MODEL_FILE;
        mjcf::saveGenericModel( g_pendulum, _savefile );

        mj_activate( "/home/gregor/.mujoco/mjkey.txt" );

        g_MujocoContext.model = mj_loadXML( _savefile.c_str(), NULL, g_error, 1000 );
        if ( !g_MujocoContext.model )
        {
            std::cout << "ERROR> " << std::string( g_error ) << std::endl;
            return 1;
        }

        // initialize data
        g_MujocoContext.data = mj_makeData( g_MujocoContext.model );
        // initialize visualization data structures
        mjv_defaultCamera( &g_MujocoContext.camera );
        mjv_defaultOption( &g_MujocoContext.options );
        mjv_defaultScene( &g_MujocoContext.scene );
        // create scene and context
        mjv_makeScene( g_MujocoContext.model, &g_MujocoContext.scene, 2000);
    }

    auto _app = engine::LApp::GetInstance();
    auto _scene = _app->scene();
    
    // make a sample camera
    // auto _camera = new engine::LFpsCamera( "fps",
    //                                        engine::LVec3( 1.0f, 2.0f, -1.0f ),
    //                                        engine::LVec3( -2.0f, -4.0f, -2.0f ),
    //                                        engine::LICamera::UP_Z );
    auto _camera = new engine::LFixedCamera3d( "fixed",
                                               engine::LVec3( 2.0f, 4.0f, 2.0f ),
                                               engine::LVec3( 0.0f, 0.0f, 0.0f ),
                                               engine::LICamera::UP_Z );
    // make a sample light source
    auto _light = new engine::LLightDirectional( engine::LVec3( 0.2, 0.2, 0.2 ), 
                                                 engine::LVec3( 0.8, 0.8, 0.8 ),
                                                 engine::LVec3( 0.15, 0.15, 0.15 ), 
                                                 0, 
                                                 engine::LVec3( -1, -1, -1 ) );
    _light->setVirtualPosition( engine::LVec3( 5, 0, 5 ) );

    // add these components to the scene
    _scene->addCamera( _camera );
    _scene->addLight( _light );

    std::vector< GeomWrapper* > _geometries;
    collectGeometries( _geometries, g_pendulum );

    for ( size_t i = 0; i < _geometries.size(); i++ )
    {
        _scene->addRenderable( _geometries[i]->meshObj );
    }

    // run main loop, target real-time simulation and 60 fps rendering
    while( _app->isActive() )
    {
        float _z = PENDULUM_LINK_LEGTH * PENDULUM_NUM_LINKS + 0.5f;
        float _y = 0.0;
        float _x = std::sin( g_MujocoContext.data->time / 2.0f );

        mjcint::setBodyPosition( g_MujocoContext.model, "base", { _x, _y, _z } );

        mjcint::setActuatorCtrl( g_MujocoContext.model, g_MujocoContext.data, 
                                 "torque_0", 2.0f * std::sin( 5.0f * g_MujocoContext.data->time ) );

        mjtNum simstart = g_MujocoContext.data->time;
        while( g_MujocoContext.data->time - simstart < 1.0 / 60.0 )
            mj_step(g_MujocoContext.model, g_MujocoContext.data);

        // update scene and render
        mjv_updateScene( g_MujocoContext.model, 
                         g_MujocoContext.data, 
                         &g_MujocoContext.options, 
                         NULL, &g_MujocoContext.camera, 
                         mjCAT_ALL, &g_MujocoContext.scene);

        // update the mesh positions from abstract renderer
        updateGeometries( _geometries, 
                          g_MujocoContext.model, 
                          &g_MujocoContext.scene );

        // update application
        _app->update();
    }

    //free visualization storage
    mjv_freeScene(&g_MujocoContext.scene);

    // free MuJoCo model and data, deactivate
    mj_deleteData(g_MujocoContext.data);
    mj_deleteModel(g_MujocoContext.model);
    mj_deactivate();

    return 0;
}
