
#pragma once

#include <simulation_base.h>

#include <mujoco_terrain_wrapper.h>
#include <mujoco_agent_wrapper.h>

#include <adapters/mujoco_collision_adapter.h>
#include <adapters/mujoco_body_adapter.h>

#ifndef MUJOCO_LICENSE_FILE
    #define MUJOCO_LICENSE_FILE "~/.mujoco/mjkey.txt"
#endif

namespace tysoc {
namespace mujoco {


    class TMjcSimulation : public TISimulation
    {

        private :

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;
        mjvCamera*  m_mjcCameraPtr;
        mjvOption*  m_mjcOptionPtr;

        mjcf::GenericElement* m_mjcfResourcesPtr;
        std::vector< mjcf::GenericElement* > m_mjcfMeshResources;

        static bool HAS_ACTIVATED_MUJOCO;// @HACK: checks that mujoco is only activated once

        protected :

        bool _initializeInternal() override;
        void _preStepInternal() override;
        void _simStepInternal() override;
        void _postStepInternal() override;
        void _resetInternal() override;

        public :

        TMjcSimulation( TScenario* scenarioPtr,
                        const std::string& workingDir );
        ~TMjcSimulation();

        mjModel* getMjcModel() { return m_mjcModelPtr; }
        mjData* getMjcData() { return m_mjcDataPtr; }
        mjvScene* getMjcScene() { return m_mjcScenePtr; }
        mjvCamera* getMjcCamera() { return m_mjcCameraPtr; }
        mjvOption* getMjcOption() { return m_mjcOptionPtr; }
    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr,
                                                const std::string& workingDir );
    
}}