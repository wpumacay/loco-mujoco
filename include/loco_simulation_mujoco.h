#pragma once

#include <loco_common_mujoco.h>
#include <loco_simulation.h>
#include <utils/loco_parsing_common.h>
#include <utils/loco_parsing_schema.h>
#include <utils/loco_parsing_element.h>

#include <primitives/loco_single_body_collider_adapter_mujoco.h>
#include <primitives/loco_single_body_adapter_mujoco.h>

namespace loco {
namespace mujoco {

    class TMujocoSimulation : public TISimulation
    {
    public :

        TMujocoSimulation( TScenario* scenarioRef  );

        TMujocoSimulation( const TMujocoSimulation& other ) = delete;

        TMujocoSimulation& operator=( const TMujocoSimulation& other ) = delete;

        ~TMujocoSimulation();

        parsing::TElement* mjcf_element() { return m_MjcfSimulationElement.get(); }

        const parsing::TElement* mjcf_element() const { return m_MjcfSimulationElement.get(); }

        mjModel* mjc_model() { return m_MjcModel.get(); }

        const mjModel* mjc_model() const { return m_MjcModel.get(); }

        mjData* mjc_data() { return m_MjcData.get(); }

        const mjData* mjc_data() const { return m_MjcData.get(); }

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal( const TScalar& dt ) override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

        void _SetTimeStepInternal( const TScalar& time_step ) override;

        void _SetGravityInternal( const TVec3& gravity ) override;

    private :

        void _CreateSingleBodyAdapters();

        // void _CreateCompoundAdapters();

        // void _CreateKintreeAdapters();

        // void _CreateTerrainGeneratorAdapters();

        void _CollectResourcesFromSingleBodies();

        // void _CollectResourcesFromCompounds();

        // void _CollectResourcesFromKintrees();

        // void _CollectResourcesFromTerrainGenerators();

        void _CollectContacts();

    private :

        // Owned MuJoCo-mjModel struct (access mujoco resources related to model structure)
        std::unique_ptr<mjModel, MjcModelDeleter> m_MjcModel;
        // Owned MuJoCo-mjData struct (access mujoco resources related to simulation computations)
        std::unique_ptr<mjData, MjcDataDeleter> m_MjcData;
        // Owned mjcf Element used to store the simulation object
        std::unique_ptr<parsing::TElement> m_MjcfSimulationElement;
        // Checking-set to avoid double-additions of assets with same name
        std::set<std::string> m_MjcfAssetsNames;
        // Checking-set to avoid double-additions of assets with same filepath
        std::set<std::string> m_MjcfAssetsFilepaths;
        // Flag to check if MuJoCo has already been activated (can only call activate once)
        static bool s_HasActivatedMujoco;
    };

extern "C" TISimulation* simulation_create( loco::TScenario* scenarioRef );

}}