
#pragma once

// interface for our UI implementation
#include <viz/ui.h>
// cat1 rendering engine resources
#include <glfw3.h>
// dear imgui resources
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>

namespace tysoc {
namespace viz {

    struct TMujocoContextUI
    {
        bool                                        isUiActive;
        bool                                        isBasicUiActive;
        GLFWwindow*                                 glfwWindowPtr;
    };

    class TMujocoUI : public TIVisualizerUI
    {

        private :

        GLFWwindow*         m_glfwWindowPtr;
        TMujocoContextUI*   m_uiContextPtr;

        // basic functionality ******************************************
        size_t          m_basicCurrentKinTreeIndx;
        std::string     m_basicCurrentKinTreeName;

        void _renderBasicMainMenu();
        void _renderBasicKinTreeActionsMenu( agent::TAgentKinTree* agentKinTreePtr );
        // **************************************************************

        protected :

        void _initUIInternal() override;
        void _renderUIInternal() override;

        public :

        TMujocoUI( TScenario* scenarioPtr,
                   TMujocoContextUI* uiContextPtr );
        ~TMujocoUI();
    };




}}