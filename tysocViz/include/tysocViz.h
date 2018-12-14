
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
// and also the current UI functionality (WIP)
#include <tysocUI.h>

namespace tysoc {
namespace viz {

    // @TODO|CHECK: This should be the abstraction, and the concretions ...
    // should be the mujoco integrated visualizer, and my own visualizer, such ...
    // that we can think ahead to see if we can accomodate other rendering engines

    // @TODO|@CHECK: Abstract away the mesh wrapping functionality, the same way ...
    // I did for the agent kintree wrapping functionality. It should be cleaner and ...
    // leave the abstract visualization interface without wrapping jargon from concretions.
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

        std::vector< tysoc::viz::TVizKinTree* > m_vizKinTreeWrappers;

        tysoc::TTysocCommonApi* m_tysocApiPtr;

        void _collectKinTreeAgent( tysoc::agent::TAgentKinTree* kinTreeAgentPtr );

        void _collectTerrainGenResources( tysoc::terrain::TITerrainGenerator* terrainGenPtr );
        void _cacheTerrainGeometry( tysoc::terrain::TTerrainPrimitive* terrainGeomPtr );

        void _updateVizKinTree( tysoc::viz::TVizKinTree* vizKinTreePtr );
        void _updateTerrainWrapper( TVizTerrainMeshWrapper* terrainWrapperPtr );

        void _resizeMesh( engine::LMesh* meshPtr, 
                          tysoc::terrain::TTerrainPrimitive* terrainGeomPtr );

        void _setColor( engine::LMesh* meshPtr, float* color );

        void _updateSensor( tysoc::sensor::TISensor* sensorPtr );

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