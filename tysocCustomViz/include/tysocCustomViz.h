
#pragma once

// interface for our VIZ implementation
#include <viz/viz.h>
// cat1 engine functionality
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>
// and some specific viz wrappers
#include <tysocCustomVizKinTree.h>
#include <tysocCustomVizTerrainGenerator.h>
// and the current UI functionality
#include <tysocCustomUI.h>



namespace tysoc {
namespace viz {


    class TCustomVisualizer : public TIVisualizer
    {

        private :

        // cat1 rendering engine resources
        engine::LApp*       m_glAppPtr;
        engine::LScene*     m_glScenePtr;
        // visualization wrappers
        std::vector< TCustomVizKinTree* >             m_vizKinTreeWrappers;
        std::vector< TCustomVizTerrainGenerator* >    m_vizTerrainGeneratorWrappers;
        // the UI context
        TCustomContextUI*   m_uiContextPtr;

        void _setupGlRenderingEngine();
        void _collectKinTreeAgent( agent::TAgentKinTree* kinTreeAgentPtr );
        void _collectTerrainGenerator( terrain::TITerrainGenerator* terrainGeneratorPtr );
        void _renderSensorReading( sensor::TISensor* sensorPtr );

        protected :

        void _initializeInternal() override;
        void _updateInternal() override;
        void _renderUIInternal() override;
        bool _isActiveInternal() override;

        TIVizCamera* _createCameraInternal( const std::string& name,
                                            const std::string& type,
                                            const TVec3& pos,
                                            const TMat3& rot ) override;
        void _changeToCameraInternal( TIVizCamera* cameraPtr ) override;
        void _grabCameraFrameInternal( TIVizCamera* cameraPtr,
                                       TIVizTexture& rgbTexture,
                                       TIVizTexture& depthTexture ) override;

        TIVizLight* _createLightInternal( const std::string& name,
                                          const std::string& type,
                                          const TVec3& pos ) override;

        public :

        TCustomVisualizer( TTysocCommonApi* tysocApiPtr );
        ~TCustomVisualizer(); // @CHECK: check for virtual destructors

    };


}}