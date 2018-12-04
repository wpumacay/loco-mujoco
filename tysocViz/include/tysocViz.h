
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

        std::vector< tysoc::viz::TVizKinTree* > m_vizKinTreeWrappers;

        tysoc::TTysocCommonApi* m_tysocApiPtr;

        void _collectAgentResources( tysocagent::TAgent* agentPtr );
        void _cacheAgentGeometry( tysocagent::TAgentGeom* agentGeomPtr );
        void _collectKinTreeAgent( tysoc::agent::TAgentKinTree* kinTreeAgentPtr );

        void _collectTerrainGenResources( tysocterrain::TTerrainGenerator* terrainGenPtr );
        void _cacheTerrainGeometry( tysocterrain::TTerrainPrimitive* terrainGeomPtr );

        void _updateAgentWrapper( TVizAgentMeshWrapper* agentWrapperPtr );
        void _updateVizKinTree( tysoc::viz::TVizKinTree* vizKinTreePtr );
        void _updateTerrainWrapper( TVizTerrainMeshWrapper* terrainWrapperPtr );

        void _resizeMesh( engine::LMesh* meshPtr, 
                          tysocterrain::TTerrainPrimitive* terrainGeomPtr );

        void _setColor( engine::LMesh* meshPtr, float* color );

        void _updateSensor( tysocsensor::TSensor* sensorPtr );

        public :

        TVisualizer( tysoc::TTysocCommonApi* api );
        ~TVisualizer();
        
        void initialize();
        void update();
        bool isActive();
    };

}