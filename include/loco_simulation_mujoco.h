#pragma once

#include <loco_common_mujoco.h>
#include <loco_simulation.h>
#include <utils/loco_parsing_common.h>
#include <utils/loco_parsing_schema.h>
#include <utils/loco_parsing_element.h>

namespace loco {
namespace mujoco {

    class TMujocoSimulation : public TISimulation
    {
    public :

        TMujocoSimulation( TScenario* scenarioRef  );

        ~TMujocoSimulation();

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal() override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

        mjModel* mjc_model() { return m_mjcModel.get(); }

        const mjModel* mjc_model() const { return m_mjcModel.get(); }

        mjData* mjc_data() { return m_mjcData.get(); }

        const mjData* mjc_data() const { return m_mjcData.get(); }

    private :

        // Owned MuJoCo-mjModel struct (access mujoco resources related to model structure)
        std::unique_ptr<mjModel, MjcModelDeleter> m_mjcModel;
        // Owned MuJoCo-mjData struct (access mujoco resources related to simulation computations)
        std::unique_ptr<mjData, MjcDataDeleter> m_mjcData;
        // Flag to check if MuJoCo has already been activated (can only call activate once)
        static bool s_HasActivatedMujoco;
    };

extern "C" TISimulation* simulation_create( loco::TScenario* scenarioRef );

}}