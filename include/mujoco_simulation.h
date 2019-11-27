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

    struct TMjcContact
    {
        TVec3 position;
        TMat3 rotation;
        TMat4 transform;

        std::string collider1Name;
        std::string collider2Name;
    };

    class TMjcContactManager
    {

    public :

        TMjcContactManager( TScenario* scenario, 
                            mjModel* mjcModel, 
                            mjData* mjcData );
        ~TMjcContactManager();

        void update();
        void reset();

        std::vector< TMjcContact > contacts() const { return m_contacts; }

    private :

        std::vector< TMjcContact > m_contacts;

        TScenario* m_scenario;
        mjModel* m_mjcModel;
        mjData* m_mjcData;

    };

    class TMjcSimulation : public TISimulation
    {

    public :

        TMjcSimulation( TScenario* scenarioPtr );
        ~TMjcSimulation();

        mjModel* getMjcModel() { return m_mjcModelPtr; }
        mjData* getMjcData() { return m_mjcDataPtr; }

    protected :

        bool _initializeInternal() override;
        void _preStepInternal() override;
        void _simStepInternal() override;
        void _postStepInternal() override;
        void _resetInternal() override;

    private :

        void _collectResourcesFromBodyAdapter( TMjcBodyAdapter* bodyAdapter );
        void _collectResourcesFromAgentAdapter( TMjcKinTreeAgentWrapper* agentAdapter );
        void _collectResourcesFromTerrainGenAdapter( TMjcTerrainGenWrapper* terrainGenAdapter );

    private :

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;

        mjcf::GenericElement* m_mjcfResourcesPtr;
        std::vector< mjcf::GenericElement* > m_mjcfMeshResources;

        TMjcContactManager* m_contactManager;

        static bool HAS_ACTIVATED_MUJOCO;// @HACK: checks that mujoco is only activated once
    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr );
    
}}