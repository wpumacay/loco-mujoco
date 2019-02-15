
#pragma once

#include <tysocMjcTerrain.h>
#include <tysocMjcFactory.h>
#include <tysocMjcKinTreeAgent.h>

// abstract api to extend from
#include <api_adapter.h>

#ifndef MUJOCO_LICENSE_FILE
    #define MUJOCO_LICENSE_FILE "~/.mujoco/mjkey.txt"
#endif

namespace tysoc {
namespace mujoco {

    // @TODO: Mode main generic description to generic api
    // @TODO: Update the description here to describe the specifics

    /**
    *
    * About the API:
    *
    *   This API extends the generic API from which all concrete ...
    *   implementations inherit from (the generic API that will) ...
    *   be exposed to the user.
    *
    *   The implementation specific stuff are written in the adapters ...
    *   shown below, which have all specific functionality from the physics ...
    *   backend to be used.
    *
    *   In order for this to work well with other subs-systems (like the visualizer) ...
    *   and to make it backend-agnostic, we essentially interact with the functionality ...
    *   from the generic API only.
    *
    *   To achieve this, we make sure that all data necessary for other subsystems ...
    *   is stored in the generic API's data structures. Then the subsystems interact ...
    *   with this data only (write info like actuator ctrls, etc., or read info like ...
    *   object positions, sensor observations, etc.)
    *
    *   The overall "step" consists on the following parts that ensure the behavior ...
    *   mentioned above :
    *
    *
    *   ---> step
    *     
    *       ---> pre-step ( collect from the generic API the info necessary ...
    *                       for the backend, like ctrls, terrain updated positions, etc. )
    *
    *       ---> update-step ( make one step in the simulation by calling the backend with ...
    *                          the updated info from before )
    *
    *       ---> post-step ( write back info from the backend into the generic API datastructures ...
    *                        in order to be used by other sub-systems )
    *
    *
    */


    class TTysocMjcApi : public TTysocCommonApi
    {

        private :

        std::vector< TMjcTerrainGenWrapper* >       m_terrainGenWrappers;
        std::vector< TMjcKinTreeAgentWrapper* >     m_kinTreeAgentWrappers;

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;
        mjvCamera*  m_mjcCameraPtr;
        mjvOption*  m_mjcOptionPtr;

        static bool HAS_ACTIVATED_MUJOCO;// @HACK: checks that mujoco is only activated once

        protected :

        void _preStep() override;
        void _updateStep() override;
        void _postStep() override;

        void _collectFromScenarioInternal() override;
        void _resetInternal() override;

        public :

        TTysocMjcApi();
        ~TTysocMjcApi();

        void addKinTreeAgentWrapper( TMjcKinTreeAgentWrapper* agentKinTreeWrapperPtr );
        void addTerrainGenWrapper( TMjcTerrainGenWrapper* terrainGenWrapperPtr );

        bool initializeMjcApi();

        mjModel* getMjcModel() { return m_mjcModelPtr; }
        mjData* getMjcData() { return m_mjcDataPtr; }
        mjvScene* getMjcScene() { return m_mjcScenePtr; }
        mjvCamera* getMjcCamera() { return m_mjcCameraPtr; }
        mjvOption* getMjcOption() { return m_mjcOptionPtr; }
    };


    
}}