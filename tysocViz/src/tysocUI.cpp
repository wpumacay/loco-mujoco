
#include <tysocUI.h>

namespace tysoc{ 
namespace ui{ 

    void initUI( TVizUiContext& uiContext )
    {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& _io = ImGui::GetIO(); (void) _io;

        ImGui_ImplGlfw_InitForOpenGL( uiContext.glfwWindowPtr, false );
        ImGui_ImplOpenGL3_Init( "#version 130" );

        ImGui::StyleColorsDark();
    }

    void renderUI( TVizUiContext& uiContext )
    {
        if ( !uiContext.isUiActive )
        {
            return;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        _handleMainMenu( uiContext );

        ImGui::Render();
        int _ww, _wh;
        glfwGetFramebufferSize( uiContext.glfwWindowPtr, &_ww, &_wh );
        glViewport( 0, 0, _ww, _wh );
        ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );
    }

    void _handleMainMenu( TVizUiContext& uiContext )
    {
        ImGui::Begin( "Main Menu" );

        ImGui::Checkbox( "SimActive", &uiContext.isSimulationActive );
        ImGui::Spacing();

        if ( ImGui::BeginCombo( "KinTrees", uiContext.vizCurrentKinTreeName.c_str() ) )
        {
            for ( size_t i = 0; i < uiContext.vizKinTreePtrs.size(); i++ )
            {
                auto _vizKinTreeName = uiContext.vizKinTreePtrs[i]->getKinTreePtr()->name();
                bool _isSelected = ( _vizKinTreeName == uiContext.vizCurrentKinTreeName );
                if ( ImGui::Selectable( _vizKinTreeName.c_str(), _isSelected ) )
                {
                    uiContext.vizCurrentKinTreeName = _vizKinTreeName;
                    uiContext.vizCurrentKinTreeIndx = i;
                }
                if ( _isSelected )
                {
                    ImGui::SetItemDefaultFocus();
                }
            }

            ImGui::EndCombo();
        }

        ImGui::Spacing();

        if ( uiContext.vizCurrentKinTreeIndx != -1 )
        {
            uiContext.vizCurrentKinTreePtr = uiContext.vizKinTreePtrs[ uiContext.vizCurrentKinTreeIndx ];
            _handleKinTreeMenu( uiContext );
        }

        ImGui::End();
    }

    void _handleKinTreeMenu( TVizUiContext& uiContext )
    {
        auto _currentVizKinTree     = uiContext.vizCurrentKinTreePtr;
        auto _currentKinTreeAgent   = _currentVizKinTree->getKinTreePtr();

        ImGui::Checkbox( "drawAsWireframe?", &_currentVizKinTree->drawState.drawAsWireframe );
        ImGui::Spacing();
        ImGui::Checkbox( "drawFrameAxes?", &_currentVizKinTree->drawState.drawFrameAxes );
        ImGui::Spacing();
        ImGui::Checkbox( "showBodies?", &_currentVizKinTree->drawState.showBodies );
        ImGui::Spacing();
        ImGui::Checkbox( "showVisuals?", &_currentVizKinTree->drawState.showVisuals );
        ImGui::Spacing();
        ImGui::Checkbox( "showCollisions?", &_currentVizKinTree->drawState.showCollisions );
        ImGui::Spacing();
        ImGui::Checkbox( "showJoints?", &_currentVizKinTree->drawState.showJoints );
        ImGui::Spacing();
        ImGui::Checkbox( "showSensors?", &_currentVizKinTree->drawState.showSensors );
        ImGui::Spacing();
        ImGui::Checkbox( "showActuators?", &_currentVizKinTree->drawState.showActuators );
        ImGui::Spacing();

        auto _actuators = _currentKinTreeAgent->getKinTreeActuators();
        std::vector< TScalar > _actions;

        for ( size_t i = 0; i < _actuators.size(); i++ )
        {
            float _val = 0.0f;
            ImGui::SliderFloat( _actuators[i]->name.c_str(), 
                                &_val,
                                _actuators[i]->minCtrl,
                                _actuators[i]->maxCtrl );
            _actions.push_back( _val );
        }

        _currentKinTreeAgent->setActions( _actions );
    }



}}