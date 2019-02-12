
#include <tysocCustomUI.h>



namespace tysoc {
namespace viz {

    TCustomUI::TCustomUI( TScenario* scenarioPtr,
                          TCustomContextUI* uiContextPtr )
        : TIVisualizerUI( scenarioPtr )
    {
        m_uiContextPtr          = uiContextPtr;
        m_glfwWindowPtr         = uiContextPtr->glfwWindowPtr;

        m_basicCurrentKinTreeName    = "";
        m_basicCurrentKinTreeIndx    = -1;
    }

    TCustomUI::~TCustomUI()
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        
        m_glfwWindowPtr = NULL;

        if ( m_uiContextPtr )
        {
            delete m_uiContextPtr;
            m_uiContextPtr = NULL;
        }
    }

    void TCustomUI::_initUIInternal()
    {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& _io = ImGui::GetIO(); (void) _io;

        ImGui_ImplGlfw_InitForOpenGL( m_glfwWindowPtr, false );
        ImGui_ImplOpenGL3_Init( "#version 130" );

        ImGui::StyleColorsDark();
    }

    void TCustomUI::_renderUIInternal()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if ( m_uiContextPtr->isUiActive )
        {
            // @TODO|@CHECK|@WIP: Should abstract away the ...
            // widgets and render them like if it was tree

            // Demo functionality
            if ( m_uiContextPtr->isBasicUiActive )
                _renderBasicMainMenu();
        }

        ImGui::Render();
        int _ww, _wh;
        glfwGetFramebufferSize( m_glfwWindowPtr, &_ww, &_wh );
        glViewport( 0, 0, _ww, _wh );
        ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );
    }

    // Basic UI functionality (for testing purposes) *********************************

    void TCustomUI::_renderBasicMainMenu()
    {
        ImGui::Begin( "Main Menu" );

        auto _kinTreeAgents = m_scenarioPtr->getAgentsByType( agent::AGENT_TYPE_KINTREE );

        if ( ImGui::BeginCombo( "Kinematic trees", m_basicCurrentKinTreeName.c_str() ) )
        {
            for ( size_t i = 0; i < _kinTreeAgents.size(); i++ )
            {
                auto _kinTreeAgent = _kinTreeAgents[i];
                bool _isSelected = ( _kinTreeAgent->name() == m_basicCurrentKinTreeName );

                if ( ImGui::Selectable( _kinTreeAgent->name().c_str(), _isSelected ) )
                {
                    m_basicCurrentKinTreeName = _kinTreeAgent->name();
                    m_basicCurrentKinTreeIndx = i;
                }

                if ( _isSelected )
                {
                    ImGui::SetItemDefaultFocus();
                }
            }

            ImGui::EndCombo();
        }

        ImGui::End();

        if ( m_basicCurrentKinTreeIndx != -1 )
        {
            size_t _indx = m_basicCurrentKinTreeIndx;
            _renderBasicKinTreeVisualsMenu( m_uiContextPtr->vizKinTreePtrs[_indx] );
            _renderBasicKinTreeActionsMenu( ( agent::TAgentKinTree* ) _kinTreeAgents[_indx] );
        }
    }

    void TCustomUI::_renderBasicKinTreeVisualsMenu( TCustomVizKinTree* vizKinTreePtr )
    {
        ImGui::Begin( "Kinematic Tree visual options" );

        ImGui::Checkbox( "drawAsWireframe?", &vizKinTreePtr->drawState.drawAsWireframe );
        ImGui::Spacing();
        ImGui::Checkbox( "drawFrameAxes?", &vizKinTreePtr->drawState.drawFrameAxes );
        ImGui::Spacing();
        ImGui::Checkbox( "showBodies?", &vizKinTreePtr->drawState.showBodies );
        ImGui::Spacing();
        ImGui::Checkbox( "showVisuals?", &vizKinTreePtr->drawState.showVisuals );
        ImGui::Spacing();
        ImGui::Checkbox( "showCollisions?", &vizKinTreePtr->drawState.showCollisions );
        ImGui::Spacing();
        ImGui::Checkbox( "showJoints?", &vizKinTreePtr->drawState.showJoints );
        ImGui::Spacing();
        ImGui::Checkbox( "showSensors?", &vizKinTreePtr->drawState.showSensors );
        ImGui::Spacing();
        ImGui::Checkbox( "showActuators?", &vizKinTreePtr->drawState.showActuators );

        ImGui::End();
    }

    void TCustomUI::_renderBasicKinTreeActionsMenu( agent::TAgentKinTree* agentKinTreePtr )
    {
        ImGui::Begin( "Kinematic Tree actuator options" );

        auto _actuators = agentKinTreePtr->getKinTreeActuators();
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

        agentKinTreePtr->setActions( _actions );

        auto _sensors = agentKinTreePtr->getKinTreeSensors();

        for ( size_t i = 0; i < _sensors.size(); i++ )
        {
            if ( _sensors[i]->type == "joint" )
            {
                auto _kinJointSensor = reinterpret_cast< agent::TKinTreeJointSensor* >( _sensors[i] );
                ImGui::Text( "theta%d: %.3f", (int)i, _kinJointSensor->theta );
                ImGui::Text( "thetadot%d: %.3f", (int)i, _kinJointSensor->thetadot );
            }
            else if ( _sensors[i]->type == "body" )
            {
                auto _kinBodySensor = reinterpret_cast< agent::TKinTreeBodySensor* >( _sensors[i] );
                ImGui::Text( "linvel%d: %.3f, %.3f, %.3f", (int)i, 
                             _kinBodySensor->linVelocity.x, 
                             _kinBodySensor->linVelocity.y, 
                             _kinBodySensor->linVelocity.z );
                ImGui::Text( "linacc%d: %.3f, %.3f, %.3f", (int)i,  
                             _kinBodySensor->linAcceleration.x, 
                             _kinBodySensor->linAcceleration.y, 
                             _kinBodySensor->linAcceleration.z );
            }
        }

        ImGui::End();
    }

    // *******************************************************************************

}}