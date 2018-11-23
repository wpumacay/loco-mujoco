
#pragma once

// cat1 engine functionality
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>

// tysocBaseApi functionality
#include <api_adapter.h>

namespace tysocViz
{

    struct TVizAgentMeshWrapper
    {
        tysocagent::TAgentGeom* geometry;
        engine::LMesh* glMesh;
    };

    struct TVizTerrainMeshWrapper
    {
        tysocterrain::TTerrainPrimitive* geometry;
        engine::LMesh* glMesh;
    };

    class TVisualizer
    {
        private :

        engine::LApp* m_glAppPtr;
        engine::LScene* m_glScenePtr;

        std::vector< TVizTerrainMeshWrapper* > m_terrainMeshWrappers;
        std::vector< TVizAgentMeshWrapper* > m_agentMeshWrappers;

        tysoc::TTysocCommonApi* m_tysocApiPtr;

        void _collectAgentResources( tysocagent::TAgent* agentPtr );
        void _cacheAgentGeometry( tysocagent::TAgentGeom* agentGeomPtr );

        void _collectTerrainGenResources( tysocterrain::TTerrainGenerator* terrainGenPtr );
        void _cacheTerrainGeometry( tysocterrain::TTerrainPrimitive* terrainGeomPtr );

        void _updateAgentWrapper( TVizAgentMeshWrapper* agentWrapperPtr );
        void _updateTerrainWrapper( TVizTerrainMeshWrapper* terrainWrapperPtr );

        void _resizeMesh( engine::LMesh* meshPtr, 
                          tysocterrain::TTerrainPrimitive* terrainGeomPtr );

        void _setColor( engine::LMesh* meshPtr, float* color );

        public :

        TVisualizer( tysoc::TTysocCommonApi* api );
        ~TVisualizer();
        
        void initialize();
        void update();
        bool isActive();
    };

}