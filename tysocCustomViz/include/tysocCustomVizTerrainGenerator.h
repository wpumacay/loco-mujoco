
#pragma once

#include <vector>
#include <iostream>
#include <string>

#include <LMeshBuilder.h>
#include <LScene.h>

#include <terrain/terrain_base.h>
// some commom functionality
#include <tysocCustomVizCommon.h>

#define VIZTERRAIN_AXES_DEFAULT_SIZE 0.1f

namespace tysoc {
namespace viz {

    struct TCustomVizTerrainPrimitive
    {
        engine::LModel*                 axesPtr;
        engine::LIRenderable*           meshPtr;
        terrain::TTerrainPrimitive*     terrainPrimitivePtr;
    };

    struct TCustomVizTerrainDrawState
    {
        bool drawAsWireframe;
        bool showFrameAxes;
        bool showPrimitives;

        TCustomVizTerrainDrawState()
        {
            drawAsWireframe     = false;
            showFrameAxes       = false;
            showPrimitives      = true;
        }
    };

    class TCustomVizTerrainGenerator
    {

        private :

        engine::LScene*                 m_scenePtr;
        terrain::TITerrainGenerator*    m_terrainGeneratorPtr;

        std::vector< TCustomVizTerrainPrimitive > m_vizTerrainPrimitives;

        void _collectTerrainPrimitives();

        engine::LIRenderable* _createMeshPrimitive( terrain::TTerrainPrimitive* terrainPrimitivePtr );
        engine::LIRenderable* _createMeshFromData( terrain::TTerrainPrimitive* terrainPrimitivePtr );

        void _resizeMesh( engine::LIRenderable* renderablePtr, terrain::TTerrainPrimitive* terrainPrimitivePtr );
        void _updateVizTerrainPrimitive( TCustomVizTerrainPrimitive& vizTerrainPrimitivePtr );

        public :

        TCustomVizTerrainGenerator( terrain::TITerrainGenerator* terrainGeneratorPtr,
                              engine::LScene* scenePtr );
        ~TCustomVizTerrainGenerator();

        TCustomVizTerrainDrawState drawState;

        void update();
        terrain::TITerrainGenerator* getTerrainGeneratorPtr();

    };


}}