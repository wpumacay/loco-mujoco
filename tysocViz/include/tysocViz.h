
#pragma once

// cat1 engine functionality
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>

// tysocBaseApi functionality
#include <api_adapter.h>
// and some specific viz wrappers
#include <tysocVizKinTree.h>

#include <tysocUI.h>

namespace tysoc {
namespace viz {

    // @TODO|CHECK: This should be the abstraction, and the concretions ...
    // should be the mujoco integrated visualizer, and my own visualizer, such ...
    // that we can think ahead to see if we can accomodate other rendering engines

    struct TVizAgentMeshWrapper
    {
        tysoc::agent::TAgentGeom* geometry;
        engine::LMesh* glMesh;
    };

    struct TVizTerrainMeshWrapper
    {
        tysoc::terrain::TTerrainPrimitive* geometry;
        engine::LMesh* glMesh;
    };

    class TVisualizer
    {
        private :

        engine::LApp* m_glAppPtr;
        engine::LScene* m_glScenePtr;

        std::vector< TVizTerrainMeshWrapper* > m_terrainMeshWrappers;
        std::vector< TVizAgentMeshWrapper* > m_agentMeshWrappers;

        std::vector< tysoc::viz::TVizKinTree* > m_vizKinTreeWrappers;

        tysoc::TTysocCommonApi* m_tysocApiPtr;

        void _collectAgentResources( tysoc::agent::TAgent* agentPtr );
        void _cacheAgentGeometry( tysoc::agent::TAgentGeom* agentGeomPtr );
        void _collectKinTreeAgent( tysoc::agent::TAgentKinTree* kinTreeAgentPtr );

        void _collectTerrainGenResources( tysoc::terrain::TTerrainGenerator* terrainGenPtr );
        void _cacheTerrainGeometry( tysoc::terrain::TTerrainPrimitive* terrainGeomPtr );

        void _updateAgentWrapper( TVizAgentMeshWrapper* agentWrapperPtr );
        void _updateVizKinTree( tysoc::viz::TVizKinTree* vizKinTreePtr );
        void _updateTerrainWrapper( TVizTerrainMeshWrapper* terrainWrapperPtr );

        void _resizeMesh( engine::LMesh* meshPtr, 
                          tysoc::terrain::TTerrainPrimitive* terrainGeomPtr );

        void _setColor( engine::LMesh* meshPtr, float* color );

        void _updateSensor( tysoc::sensor::TSensor* sensorPtr );

        // UI functionality
        tysoc::ui::TVizUiContext m_uiContext;
        void _renderUI();

        public :

        TVisualizer( tysoc::TTysocCommonApi* api );
        ~TVisualizer();
        
        void initialize();
        void update();
        bool isActive();
    };

}}