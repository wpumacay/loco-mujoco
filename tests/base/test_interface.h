
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
    std::string mjtJoint2string( int type );
    tysoc::TVec3 mjtNum2vec3( mjtNum* numPtr );
    tysoc::TVec4 mjtNum2vec4( mjtNum* numPtr );
    tysoc::TVec4 mjtNumQuat2vec4( mjtNum* numPtr );
    tysoc::TVec3 floatptr2vec3( float* floatPtr );
    tysoc::TVec4 floatptr2vec4( float* floatPtr );

    /**
    *   Wrapper for a single joint associated with a mjc-body
    */
    class SimJoint
    {
        protected :

        int             m_jointId;
        std::string     m_jointName;
        tysoc::TVec3    m_jointLocalPos;
        tysoc::TMat4    m_jointLocalTransform;
        tysoc::TMat4    m_jointWorldTransform;

        int m_jointType;            // type-id of the this joint
        int m_jointBodyParentId;    // id of the parent body
        int m_jointQposNum;         // number of generalized coordinates associated with this joint
        int m_jointQposAdr;         // offset of the qpos-s in qpos buffer
        int m_jointQvelNum;         // number of motion-degrees of freedom  associated with this joint
        int m_jointQvelAdr;         // offset of the qvel-s in qvel buffer

        std::vector< mjtNum > m_jointQpos0; // default values of the generalized coordinates

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;

        public :

        /* Constructs the joint wrapper given the joint-id of the body */
        SimJoint( int jointId,
                  mjModel* mjcModelPtr,
                  mjData* mjcDataPtr );

        /* Frees|unlinks all related resources of the currently wrapper joint */
        ~SimJoint();

        /* Updates all internal information by quering the backend */
        void update();

        /* Returns the unique identifier of the wrapped joint */
        int id() { return m_jointId; }

        /* Returns the unique name of the wrapped joint */
        std::string name() { return m_jointName; }

        /* Prints some debug information */
        void print();

        /* Resets the joint qpos->qpos0 and qvel->zeros */
        void reset();

        /* Resets the joint qpos->qpos-given and qvel->zeros */
        void reset( const std::vector< mjtNum >& qpos );

        /* Returns the local transform of the joint w.r.t. its parent body */
        tysoc::TVec3 localPosition() { return m_jointLocalPos; }

        /* Returns the local transform of the joint w.r.t. its parent body */
        tysoc::TMat4 localTransform() { return m_jointLocalTransform; }

        /* Returns the world transform of the joint */
        tysoc::TMat4 worldTransform() { return m_jointWorldTransform; }
    };

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

        int m_jointsNum; // number of joints that this body has
        int m_jointsAdr; // offset of this body's joints in the joints buffer

        std::vector< SimJoint* >            m_simJoints;
        std::map< std::string, SimJoint* >  m_simJointsMap;

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;

        /* Grabs all geometries associated with this body */
        void _grabGeometries();

        /* Grabs some useful information from the mjc-body */
        void _grabJoints();

        /* Builds a geometry's graphics */
        engine::LIRenderable* _buildGeomGraphics( const std::string& type,
                                                  const tysoc::TVec3& size,
                                                  const tysoc::TVec4& color );

        public :

        /* Constructs the body wrapper given the name of the body */
        SimBody( const std::string& bodyName,
                 mjModel* mjcModelPtr,
                 mjData* mjcDataPtr );

        /* Frees|unlinks all related resources of the currently wrapped body */
        ~SimBody();

        /* Updates all internal information by quering the backend */
        void update();

        /* Returns the unique identifier of the wrapped body */
        int id() { return m_bodyId; }

        /* Returns the unique name of the wrapped body */
        std::string name() { return m_bodyName; }

        /* Returns all meshes linked to each geometry */
        std::vector< engine::LIRenderable* > geomsGraphics() { return m_geomsGraphics; }

        /* Returns all sim-joint wrappers of the joints associated with this body */
        std::vector< SimJoint* > joints() { return m_simJoints; }

        /* Returns a joint with a given name */
        SimJoint* getJointByName( const std::string& name );

        /* Prints some debug information */
        void print();

        /* Resets the body to some configuration */
        void reset();

        /* Returns the world-position of the body */
        tysoc::TVec3 position() { return m_bodyWorldPos; }

        /* Returns the world-orientation of the body */
        tysoc::TVec4 orientation() { return m_bodyWorldQuat; }

        /* Returns the world transform of the body */
        tysoc::TMat4 worldTransform() { return m_bodyWorldTransform; }
    };

    /**
    *   Wrapper for a whole lot of bodies that conform a single agent-model
    */
    class SimAgent
    {
        protected :

        int             m_rootBodyId;
        std::string     m_rootBodyName;
        tysoc::TVec3    m_rootBodyWorldPos;
        tysoc::TVec4    m_rootBodyWorldQuat;
        tysoc::TMat4    m_rootBodyWorldTransform;

        std::vector< SimBody* > m_bodies;

        mjModel*    m_mjcModelPtr; // a reference to the mujoco-model data structure
        mjData*     m_mjcDataPtr;  // a reference to the mujoco-data data structure

        /* Collects all bodies associated with this agent */
        void _collectBodies( const std::vector< SimBody* >& bodies );

        public :

        /* Constructs the agent wrapper given the rootbody of the agent */
        SimAgent( int rootBodyId,
                  const std::vector< SimBody* >& bodies,
                  mjModel* mjcModelPtr,
                  mjData* mjcDataPtr );

        /* Frees|unlinks all related resources of the currently wrapped agent */
        ~SimAgent();

        /* Updates all internal components from the backend information */
        void update();

        /* Resets the agent to some configuration */
        void reset();

        /* Returns all body wrappers associated with this agent */
        std::vector< SimBody* > bodies() { return m_bodies; }
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

        std::vector< SimAgent* > m_simAgents;

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

        /* Returns all agent-wrappers in the simulation */
        std::vector< SimAgent* > agents() { return m_simAgents; }
    };

}