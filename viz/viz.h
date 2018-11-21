
#pragma once

#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>

namespace viz
{

    class Visualizer
    {
        private :

        std::vector< engine::LMesh* > m_terrainMeshes;
        std::map< std::string, engine::LMesh* > m_agentMeshes;

        public :

        Visualizer();
        ~Visualizer();
        
        void initialize();
        void update();
    };






}