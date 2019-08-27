
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
    std::string mjtTrn2string( int type );
    std::string mjtDyn2string( int type );
    std::string mjtGain2string( int type );
    std::string mjtBias2string( int type );

    tysoc::TVec2 mjtNum2vec2( mjtNum* numPtr );
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

        /* Returns the type of this joint */
        int type() { return m_jointType; }

        /* Prints some debug information */
        void print();

        /* Resets the joint qpos->qpos0 and qvel->zeros */
        void reset();

        /* Resets the joint qpos->qpos-given and qvel->zeros */
        void reset( const std::vector< mjtNum >& qpos );

        /* Sets the qpos-values given */
        void setQpos( const std::vector< mjtNum >& qpos );

        /* Gets the qpos-values */
        std::vector< mjtNum > getQpos();

        /* Returns the local transform of the joint w.r.t. its parent body */
        tysoc::TVec3 localPosition() { return m_jointLocalPos; }

        /* Returns the local transform of the joint w.r.t. its parent body */
        tysoc::TMat4 localTransform() { return m_jointLocalTransform; }

        /* Returns the world transform of the joint */
        tysoc::TMat4 worldTransform() { return m_jointWorldTransform; }
    };

    /**
    *   Wrapper for a single actuator attached to a joint of an agent. So
    *   far we only support joints as transmissions, but will updated later.
    *   Keep in mind that actuators in mujoco are SISO (single-input single-output)
    */
    class SimActuator
    {
        protected :

        static int s_numActuators;

        int             m_actuatorId;       // unique identifier of this actuator in the mjcf model
        int             m_linkedJointId;    // id of the joint to which this actuator is linked
        std::string     m_actuatorName;     // name associated with this actuator
        std::string     m_actuatorType;     // either torque, position, velocity, or general
        std::string     m_transmissionType; // either joint|force-in-parent|slider-crank|tendon|site
        std::string     m_dynamicsType;     // either none|integrator|filter|muscle|user
        std::string     m_gainType;         // either fixed|muscle|user
        std::string     m_biasType;         // either none|affine|muscle|user

        mjModel* m_mjcModelPtr;
        mjData* m_mjcDataPtr;

        public :

        /* Whether or not this actuator has limited control to be accepted (clamped to range) */
        bool limitedCtrl;

        /* Whether or not this actuator has limited force|torque to be generated (clamped to range) */
        bool limitedForce;

        /* Range used to clamp the control inputs */
        tysoc::TVec2 rangeControls;

        /* Range used to clamp the force|torque outputs */
        tysoc::TVec2 rangeForces;

        /* Parameters for the dynamics of this actuator (if any, see mujoco-actuation-model) */
        std::array< TScalar, mjNDYN > dynprm;

        /* Parameters for the gain of this actuator (if any, see mujoco-actuation-model) */
        std::array< TScalar, mjNGAIN > gainprm;

        /* Parameters for the bias of this actuator (if any, see mujoco-actuation-model) */
        std::array< TScalar, mjNBIAS > biasprm;

        /* Scaler factors (gear) used for this actuator */
        std::array< TScalar, 6 > gear;

        /* Control value applied to this actuator */
        TScalar ctrl;

        /* Constructs a wrapper for an actuator created from a mjcf model  */
        SimActuator( int actuatorId,
                     mjModel* mjcModelPtr,
                     mjData* mjcDataPtr );

        /* Frees|unlinks all related resources of the currently wrapped actutaor */
        ~SimActuator();

        /* Updates internal information to and from the backend, like setting controls, etc. */
        void update();

        /* Returns the unique-id of this actuator */
        int id() { return m_actuatorId; }

        /* Returns the unique-name of this actuator */
        std::string name() { return m_actuatorName; }

        /* Returns the unique-id of the joint used for the transmission of this actuator */
        int jointId() { return m_linkedJointId; }

        /* Returns the actuator type */
        std::string type() { return m_actuatorType ;}
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

        tysoc::TVec3 m_extComForce;
        tysoc::TVec3 m_extComTorque;

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

        /* Returns all ids of the geometries associated with this body */
        std::vector< int > geomsIds() { return m_geomsIds; }

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

        /* Returns the net com-force */
        tysoc::TVec3 comForce() { return m_extComForce; }

        /* Returns the net com-torque */
        tysoc::TVec3 comTorque() { return m_extComTorque; }
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
        std::vector< SimJoint* > m_joints;
        std::vector< SimActuator* > m_actuators;

        mjModel*    m_mjcModelPtr; // a reference to the mujoco-model data structure
        mjData*     m_mjcDataPtr;  // a reference to the mujoco-data data structure

        /* Collects all bodies associated with this agent */
        void _collectBodies( const std::vector< SimBody* >& bodies );

        /* Collects all actuators associated with this agent */
        void _collectActuators( const std::vector< SimActuator* >& actuators );

        public :

        /* Constructs the agent wrapper given the rootbody of the agent */
        SimAgent( int rootBodyId,
                  const std::vector< SimBody* >& bodies,
                  const std::vector< SimActuator* >& actuators,
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

        /* Returns all joint wrappers associated with this agent */
        std::vector< SimJoint* > joints() { return m_joints; }

        /* Returns all actuator wrappers associated with this agent */
        std::vector< SimActuator* > actuators() { return m_actuators; }
    };


    /**
    *   Wrapper for contacts in the engine
    */
    class SimContact
    {
        protected :

        bool                    m_active;
        tysoc::TVec3            m_worldPos;
        tysoc::TMat3            m_worldRot;
        engine::LIRenderable*   m_graphicsContactPoint;
        engine::LIRenderable*   m_graphicsContactDirection;

        int m_geomId1;
        int m_geomId2;

        std::string m_geomName1;
        std::string m_geomName2;

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;

        public :

        /* Creates a contact wrapper, by default with no contact assigned*/
        SimContact( mjModel* mjcModelPtr,
                    mjData* mjcDataPtr );

        /* Releases the resources of this contact wrapper */
        ~SimContact();

        /* Updates the wrapper resosurces from the mujoco-contact information */
        void update( const mjContact& contactInfo );

        /* Resets the resources of this contact wrapper (no draw, etc.) */
        void reset();

        /* Returns state of this contact wrapper (active=true->wraps contact) */
        bool active() { return m_active; }

        /* Returns the graphics object for the contact point */
        engine::LIRenderable* graphicsContactPoint() { return m_graphicsContactPoint; }

        /* Returns the graphics object for the contact direction (normal) */
        engine::LIRenderable* graphicsContactDirection() { return m_graphicsContactDirection; }
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

        std::vector< SimActuator* >             m_simActuators;
        std::map< std::string, SimActuator* >   m_simActuatorsMap;

        std::vector< SimAgent* > m_simAgents;

        std::vector< SimContact* > m_simContacts;

        int m_currentAgentIndx;
        std::string m_currentAgentName;

        bool m_isRunning;
        bool m_isTerminated;
        bool m_isMjcActivated;

        bool m_uiUsingActuators;

        void _initScenario();
        void _initPhysics(); 
        void _initGraphics();

        void _renderUi();
        void _renderUiAgents();
        // User should override this and create what he wants
        virtual void _initScenarioInternal() = 0;
        // Special functionality used after taking a step
        virtual void _stepInternal() {}
        // Special functionality used before calling reset
        virtual void _resetInternal() {}
        // UI functionality
        virtual void _renderUiInternal() {}
        // Entry-point for when the application is fully created
        virtual void _onApplicationStart() {}

        public :

        ITestApplication();
        ~ITestApplication();

        void init();
        void reset();
        void step();
        void togglePause();

        /* construct the whole model again */
        void reload( const std::string& modelfile );

        bool isTerminated() { return m_isTerminated; }

        /* Returns a body-wrapper of a body with a specific name in the simulation */
        SimBody* getBody( const std::string& name );

        /* Returns all agent-wrappers in the simulation */
        std::vector< SimAgent* > agents() { return m_simAgents; }
    };

}