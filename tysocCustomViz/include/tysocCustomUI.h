
#pragma once

// interface for our UI implementation
#include <viz/ui.h>
// cat1 rendering engine resources
#include <LApp.h>
// dear imgui resources
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// kintree visualization wrapper
#include <tysocCustomVizKinTree.h>

namespace tysoc {
namespace viz {

    struct TCustomContextUI
    {
        bool                                        isUiActive;
        bool                                        isBasicUiActive;
        GLFWwindow*                                 glfwWindowPtr;
        std::vector< viz::TCustomVizKinTree* >      vizKinTreePtrs;
    };

    class TCustomUI : public TIVisualizerUI
    {

        private :

        GLFWwindow*         m_glfwWindowPtr;
        TCustomContextUI*   m_uiContextPtr;

        // basic functionality ******************************************
        size_t          m_basicCurrentKinTreeIndx;
        std::string     m_basicCurrentKinTreeName;

        void _renderBasicMainMenu();
        void _renderBasicKinTreeVisualsMenu( TCustomVizKinTree* vizKinTreePtr );
        void _renderBasicKinTreeActionsMenu( agent::TAgentKinTree* agentKinTreePtr );
        // **************************************************************

        protected :

        void _initUIInternal() override;
        void _renderUIInternal() override;

        public :

        TCustomUI( TScenario* scenarioPtr,
                   TCustomContextUI* uiContextPtr );
        ~TCustomUI();
    };




}}