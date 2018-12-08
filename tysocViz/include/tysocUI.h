
#pragma once

#include <LApp.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <tysocVizKinTree.h>

namespace tysoc{
namespace ui{

    struct TVizUiContext
    {
        GLFWwindow*                         glfwWindowPtr;
        bool                                isSimulationActive;
        std::vector< viz::TVizKinTree* >    vizKinTreePtrs;
        viz::TVizKinTree*                   vizCurrentKinTreePtr;
        std::string                         vizCurrentKinTreeName;
        size_t                              vizCurrentKinTreeIndx;

        TVizUiContext()
        {
            isSimulationActive      = true;
            glfwWindowPtr           = NULL;
            vizCurrentKinTreeName   = "";
            vizCurrentKinTreeIndx   = -1;
            vizCurrentKinTreePtr    = NULL;
        }
    };

    void initUI( TVizUiContext& uiContext );
    void renderUI( TVizUiContext& uiContext );

    void _handleMainMenu( TVizUiContext& uiContext );
    void _handleKinTreeMenu( TVizUiContext& uiContext );

}}