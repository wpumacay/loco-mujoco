
#pragma once

#include <vector>
#include <iostream>
#include <string>

#include <LMeshBuilder.h>
#include <LScene.h>

#include <agent/types/agent_kintree.h>

#include <tysocVizCommon.h>

#define VIZKINTREE_AXES_DEFAULT_SIZE 0.1f
#define VIZKINTREE_COLLISION_DEFAULT_MARGIN 0.01f

#define VIZKINTREE_BODY_DEFAULT_SIZE { 0.025f, 0.0f, 0.0f }
#define VIZKINTREE_BODY_DEFAULT_COLOR { 0.7f, 0.8f, 0.2f }

#define VIZKINTREE_JOINT_DEFAULT_SIZE { 0.0125f, 0.025f, 0.0f }
#define VIZKINTREE_JOINT_DEFAULT_COLOR { 0.1f, 0.1f, 0.85f }

#define VIZKINTREE_ACTUATOR_DEFAULT_SIZE { 0.025f, 0.0125f, 0.0f }
#define VIZKINTREE_ACTUATOR_DEFAULT_COLOR { 0.1f, 0.55f, 0.85f }

#define VIZKINTREE_SENSOR_DEFAULT_SIZE { 0.01f, 0.01f, 0.01f }
#define VIZKINTREE_SENSOR_DEFAULT_COLOR { 0.0f, 0.9f, 0.9f }

#define VIZKINTREE_SENSOR_DEFAULT_SIZE { 0.01f, 0.01f, 0.01f }
#define VIZKINTREE_SENSOR_DEFAULT_COLOR { 0.0f, 0.9f, 0.9f }

#define VIZKINTREE_VISUAL_DEFAULT_COLOR { 0.25f, 0.25f, 1.0f }

#define VIZKINTREE_COLLISION_DEFAULT_COLOR { 0.25f, 1.0f, 0.25f }

namespace tysoc{
namespace viz{

    struct TVizKinCollision
    {
        engine::LModel*             axesPtr;
        engine::LIRenderable*       meshPtr;
        agent::TKinTreeCollision*   collisionPtr;
    };

    struct TVizKinVisual
    {
        engine::LModel*         axesPtr;
        engine::LIRenderable*   meshPtr;
        agent::TKinTreeVisual*  visualPtr;
    };

    struct TVizKinSensor
    {
        engine::LModel*         axesPtr;
        engine::LIRenderable*   meshPtr;
        agent::TKinTreeSensor*  sensorPtr;
    };

    struct TVizKinActuator
    {
        engine::LModel*             axesPtr;
        engine::LIRenderable*       meshPtr;
        agent::TKinTreeActuator*    actuatorPtr;
    };

    struct TVizKinJoint
    {
        engine::LModel*         axesPtr;
        engine::LIRenderable*   meshPtr;
        agent::TKinTreeJoint*   jointPtr;
    };

    struct TVizKinBody
    {
        engine::LModel*         axesPtr;
        engine::LIRenderable*   meshPtr;
        agent::TKinTreeBody*    bodyPtr;
    };

    struct TVizDrawState
    {
        bool drawAsWireframe;
        bool drawFrameAxes;
        bool showBodies;
        bool showVisuals;
        bool showCollisions;
        bool showJoints;
        bool showSensors;
        bool showActuators;

        TVizDrawState()
        {
            drawAsWireframe     = false;
            drawFrameAxes       = false;
            showBodies          = false;
            showVisuals         = true;
            showCollisions      = false;
            showSensors         = false;
            showJoints          = false;
            showActuators       = false;
        }
    };

    /**
    * This is a wrapper on top of a kintree for our ...
    * visualizer. This will construct the engine-mesh data ...
    * to be use by the visualizer, and update these meshes ...
    * properties using the wrapped kintree (which should be ...
    * updated by the underlying physics backend.)
    */
    class TVizKinTree
    {

        private :

        engine::LScene*         m_scenePtr;
        agent::TAgentKinTree*   m_agentKinTreePtr;

        std::vector< TVizKinBody >         m_vizBodies;
        std::vector< TVizKinJoint >        m_vizJoints;
        std::vector< TVizKinSensor >       m_vizSensors;
        std::vector< TVizKinVisual >       m_vizVisuals;
        std::vector< TVizKinActuator >     m_vizActuators;
        std::vector< TVizKinCollision >    m_vizCollisions;

        void _updateBody( TVizKinBody& kinBody );
        void _updateJoint( TVizKinJoint& kinJoint );
        void _updateSensor( TVizKinSensor& kinSensor );
        void _updateVisual( TVizKinVisual& kinVisual );
        void _updateActuator( TVizKinActuator& kinActuator );
        void _updateCollision( TVizKinCollision& kinCollision );

        void _collectFromKinTree();
        void _collectKinBodies();
        void _collectKinJoints();
        void _collectKinActuators();
        void _collectKinSensors();
        void _collectKinVisuals();
        void _collectKinCollisions();

        engine::LIRenderable* _createMesh( const std::string& type,
                                           const TVec3& size,
                                           const TVec3& cAmbient,
                                           const TVec3& cDiffuse,
                                           const TVec3& cSpecular,
                                           const std::string& filename = "" );

        public :

        TVizKinTree( agent::TAgentKinTree* agentKinTreePtr,
                     engine::LScene* scenePtr );
        ~TVizKinTree();

        TVizDrawState drawState;

        void update();
        agent::TAgentKinTree* getKinTreePtr();
    };


}}