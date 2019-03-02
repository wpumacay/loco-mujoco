
#include <mujocoviz_ui.h>



namespace tysoc {
namespace viz {

    TMujocoUI::TMujocoUI( TScenario* scenarioPtr,
                          TMujocoContextUI* uiContextPtr )
        : TIVisualizerUI( scenarioPtr )
    {
        m_uiContextPtr          = uiContextPtr;
        m_glfwWindowPtr         = uiContextPtr->glfwWindowPtr;

        m_basicCurrentKinTreeName    = "";
        m_basicCurrentKinTreeIndx    = -1;
    }

    TMujocoUI::~TMujocoUI()
    {
        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        m_glfwWindowPtr = NULL;

        if ( m_uiContextPtr )
        {
            delete m_uiContextPtr;
            m_uiContextPtr = NULL;
        }
    }

    void TMujocoUI::_initUIInternal()
    {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& _io = ImGui::GetIO(); (void) _io;

        ImGui_ImplGlfw_InitForOpenGL( m_glfwWindowPtr, false );
        ImGui_ImplOpenGL2_Init();

        ImGui::StyleColorsDark();
    }

    void TMujocoUI::_renderUIInternal()
    {
        ImGui_ImplOpenGL2_NewFrame();
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
        ImGui_ImplOpenGL2_RenderDrawData( ImGui::GetDrawData() );
    }

    // Basic UI functionality (for testing purposes) *********************************

    void TMujocoUI::_renderBasicMainMenu()
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
            _renderBasicKinTreeActionsMenu( ( agent::TAgentKinTree* ) _kinTreeAgents[_indx] );
        }
    }

    void TMujocoUI::_renderBasicKinTreeActionsMenu( agent::TAgentKinTree* agentKinTreePtr )
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

        ImGui::End();
    }

    // *******************************************************************************

}}