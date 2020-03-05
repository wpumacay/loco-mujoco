#pragma once

#include <loco_common_mujoco.h>
#include <loco_simulation.h>
#include <utils/loco_parsing_common.h>
#include <utils/loco_parsing_schema.h>
#include <utils/loco_parsing_element.h>

#include <adapters/loco_collision_adapter_mujoco.h>
#include <adapters/loco_single_body_adapter_mujoco.h>

namespace loco {
namespace mujoco {

    class TMujocoSimulation : public TISimulation
    {
    public :

        TMujocoSimulation( TScenario* scenarioRef  );

        TMujocoSimulation( const TMujocoSimulation& other ) = delete;

        TMujocoSimulation& operator=( const TMujocoSimulation& other ) = delete;

        ~TMujocoSimulation();

        parsing::TElement* mjcf_element() { return m_mjcfSimulationElement.get(); }

        const parsing::TElement* mjcf_element() const { return m_mjcfSimulationElement.get(); }

        mjModel* mjc_model() { return m_mjcModel.get(); }

        const mjModel* mjc_model() const { return m_mjcModel.get(); }

        mjData* mjc_data() { return m_mjcData.get(); }

        const mjData* mjc_data() const { return m_mjcData.get(); }

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal() override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

    private :

        void _CreateSingleBodyAdapters();

        // void _CreateCompoundAdapters();

        // void _CreateKintreeAdapters();

        // void _CreateTerrainGeneratorAdapters();

        void _CollectResourcesFromSingleBodies();

        // void _CollectResourcesFromCompounds();

        // void _CollectResourcesFromKintrees();

        // void _CollectResourcesFromTerrainGenerators();

    private :

        // Owned MuJoCo-mjModel struct (access mujoco resources related to model structure)
        std::unique_ptr<mjModel, MjcModelDeleter> m_mjcModel;
        // Owned MuJoCo-mjData struct (access mujoco resources related to simulation computations)
        std::unique_ptr<mjData, MjcDataDeleter> m_mjcData;
        // Owned mjcf Element used to store the simulation object
        std::unique_ptr<parsing::TElement> m_mjcfSimulationElement;
        // Checking-set to avoid double-additions of assets with same name
        std::set<std::string> m_mjcfAssetsNames;
        // Checking-set to avoid double-additions of assets with same filepath
        std::set<std::string> m_mjcfAssetsFilepaths;
        // Flag to check if MuJoCo has already been activated (can only call activate once)
        static bool s_HasActivatedMujoco;
    };

extern "C" TISimulation* simulation_create( loco::TScenario* scenarioRef );

}}