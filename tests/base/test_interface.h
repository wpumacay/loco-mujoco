
#pragma once

// MuJoCo API functionality
#include <mujoco.h>
// Rendering functionality from 'cat1' engine
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>
// UI functionality (from Dear ImGui)
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// Base functionality (math and a few helpers) from tysoc-core
#include <tysoc_common.h>

#define DEFAULT_DENSITY 1000.0f // density of water, same default as in mujoco

namespace mujoco
{

    std::string mjtGeom2string( int type );
    tysoc::TVec3 mjtNum2vec3( mjtNum* numPtr );
    tysoc::TVec4 mjtNum2vec4( mjtNum* numPtr );
    tysoc::TVec4 mjtNumQuat2vec4( mjtNum* numPtr );
    tysoc::TVec3 floatptr2vec3( float* floatPtr );
    tysoc::TVec4 floatptr2vec4( float* floatPtr );

    /**
    *   Wrapper for a single body with geometries as children
    */
    class SimBody
    {
        protected :

        int             m_bodyId;
        std::string     m_bodyName;
        tysoc::TVec3    m_bodyWorldPos;
        tysoc::TVec4    m_bodyWorldQuat;
        tysoc::TMat4    m_bodyWorldTransform;

        std::vector< int >                      m_geomsIds;
        std::vector< std::string >              m_geomsNames;
        std::vector< tysoc::TMat4 >             m_geomsLocalTransforms;
        std::vector< engine::LIRenderable* >    m_geomsGraphics;

        mjModel* m_mjcModelPtr; // a reference to the mujoco-model data structure
        mjData* m_mjcDataPtr;   // a reference to the mujoco-data data structure

        /* Grabs all geometries associated with this body */
        void _grabGeometries();

        /* Builds a geometry's graphics */
        engine::LIRenderable* _buildGeomGraphics( const std::string& type,
                                                  const tysoc::TVec3& size,
                                                  const tysoc::TVec4& color );

        public :

        /* Construct the body wrapper given the name of the body */
        SimBody( const std::string& bodyName,
                 mjModel* mjcModelPtr,
                 mjData* mjcDataPtr );

        /* Frees|unlinks all related resources of the currently wrapped body */
        ~SimBody();

        /* Updates all internal components from the backend information */
        void update();

        /* Returns the unique identifier of the wrapped body */
        int id() { return m_bodyId; }

        /* Returns the unique name of the wrapped body */
        std::string name() { return m_bodyName; }

        /* Returns all meshes linked to each geometry */
        std::vector< engine::LIRenderable* > geomsGraphics() { return m_geomsGraphics; }

        /* Prints some debug information */
        void print();

        /* Resets the body to some configuration */
        void reset();
    };


    class ITestApplication
    {

        protected :

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;
        mjvCamera*  m_mjcCameraPtr;
        mjvOption*  m_mjcOptionPtr;
        std::string m_mjcModelFile;
        char        m_mjcErrorMsg[1000];

        engine::LApp* m_graphicsApp;
        engine::LScene* m_graphicsScene;

        std::vector< SimBody* >             m_simBodies;
        std::map< std::string, SimBody* >   m_simBodiesMap;

        bool m_isRunning;
        bool m_isTerminated;

        void _initScenario();
        void _initPhysics(); 
        void _initGraphics();
        // User should override this and create what he wants
        virtual void _initScenarioInternal() = 0;
        // Special functionality used after taking a step
        virtual void _stepInternal() {};
        // Special functionality used before calling reset
        virtual void _resetInternal() {};
        // UI functionality
        virtual void _renderUI() {};

        public :

        ITestApplication();
        ~ITestApplication();

        void init();
        void reset();
        void step();
        void togglePause();

        bool isTerminated() { return m_isTerminated; }

        /* Returns a body-wrapper of a body with a specific name in the simulation */
        SimBody* getBody( const std::string& name );
    };

}