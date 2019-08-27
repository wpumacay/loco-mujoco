
#include <test_interface.h>

class AppExample : public mujoco::ITestApplication
{
    private :

    std::string m_currentActuatorName;
    std::string m_mjcfCurrentModel;
    std::vector< std::string > m_mjcfModelFiles;

    protected :

    void _initScenarioInternal() override;
    void _renderUiInternal() override;

    public :

    AppExample();
    ~AppExample();
};

AppExample::AppExample()
    : mujoco::ITestApplication()
{
    m_mjcfCurrentModel = "";
    m_currentActuatorName = "";
}

AppExample::~AppExample()
{
    // nothing here
}

void AppExample::_initScenarioInternal()
{
    // load the basic.xml mujoco scene
    m_mjcModelFile = std::string( TYSOC_PATH_WORKING_DIR ) + "tests/actuators/actuators_pd.xml";
    m_mjcfModelFiles = mujoco::collectAvailableModels( std::string( TYSOC_PATH_WORKING_DIR ) + "/tests/actuators/" );
}

void AppExample::_renderUiInternal()
{
    // render a menu to choose an actuator to check it further
    ImGui::Begin( "Available actuators" );

    if ( ImGui::BeginCombo( "Select an actuator to analyze", m_currentActuatorName.c_str() ) )
    {
        for ( size_t i = 0; i < m_simActuators.size(); i++ )
        {
            bool _isSelected = ( m_simActuators[i]->name() == m_currentActuatorName );

            if ( ImGui::Selectable( m_simActuators[i]->name().c_str(), _isSelected ) )
                m_currentActuatorName = m_simActuators[i]->name();

            if ( _isSelected )
                ImGui::SetItemDefaultFocus();
        }

        ImGui::EndCombo();
    }

    if ( m_currentActuatorName != "" )
    {
        auto _actuatorPtr = m_simActuatorsMap[m_currentActuatorName];

        ImGui::Text( "Id                : %i", _actuatorPtr->id() );
        ImGui::Text( "Type              : %s", _actuatorPtr->type().c_str() );
        ImGui::Text( "Transmission      : %s", _actuatorPtr->trntype().c_str() );
        ImGui::Text( "Dynamics          : %s", _actuatorPtr->dyntype().c_str() );
        ImGui::Text( "Gain              : %s", _actuatorPtr->gaintype().c_str() );
        ImGui::Text( "Bias              : %s", _actuatorPtr->biastype().c_str() );

        ImGui::Text( "Limited-control   : %s", std::to_string( _actuatorPtr->limitedCtrl ).c_str() );
        if ( _actuatorPtr->limitedCtrl )
            ImGui::Text( "Range-control     : %s", tysoc::TVec2::toString( _actuatorPtr->rangeControls ).c_str() );

        ImGui::Text( "Limited-force     : %s", std::to_string( _actuatorPtr->limitedForce ).c_str() );
        if ( _actuatorPtr->limitedForce ) 
            ImGui::Text( "Range-force       : %s", tysoc::TVec2::toString( _actuatorPtr->rangeForces ).c_str() );

        std::stringstream _ssDynPrm;
        _ssDynPrm << "( ";
        for ( size_t i = 0; i < _actuatorPtr->dynprm.size(); i++ )
            _ssDynPrm << _actuatorPtr->dynprm[i] << " ";
        _ssDynPrm << ")\n\r";
        ImGui::Text( "Dynamics-params   : %s", _ssDynPrm.str().c_str() );

        std::stringstream _ssGainPrm;
        _ssGainPrm << "( ";
        for ( size_t i = 0; i < _actuatorPtr->gainprm.size(); i++ )
            _ssGainPrm << _actuatorPtr->gainprm[i] << " ";
        _ssGainPrm << ")\n\r";
        ImGui::Text( "Gain-params       : %s", _ssGainPrm.str().c_str() );

        std::stringstream _ssBiasPrm;
        _ssBiasPrm << "( ";
        for ( size_t i = 0; i < _actuatorPtr->biasprm.size(); i++ )
            _ssBiasPrm << _actuatorPtr->biasprm[i] << " ";
        _ssBiasPrm << ")\n\r";
        ImGui::Text( "Bias-params       : %s", _ssBiasPrm.str().c_str() );

        std::stringstream _ssGear;
        _ssGear << "( ";
        for ( size_t i = 0; i < _actuatorPtr->gear.size(); i++ )
            _ssGear << _actuatorPtr->gear[i] << " ";
        _ssGear << ")\n\r";
        ImGui::Text( "Gear-params       : %s", _ssGear.str().c_str() );

    }

    ImGui::End();


    // render a menu to choose models to reload from
    ImGui::Begin( "Available mjcf models" );

    if ( ImGui::BeginCombo( "Select model to load", m_mjcfCurrentModel.c_str() ) )
    {
        for ( size_t i = 0; i < m_mjcfModelFiles.size(); i++ )
        {
            bool _isSelected = ( m_mjcfModelFiles[i] == m_mjcfCurrentModel );

            if ( ImGui::Selectable( m_mjcfModelFiles[i].c_str(), _isSelected ) )
                m_mjcfCurrentModel = m_mjcfModelFiles[i];

            if ( _isSelected )
                ImGui::SetItemDefaultFocus();
        }

        ImGui::EndCombo();
    }

    ImGui::Spacing();

    bool _reloadRequested = false;
    if ( m_mjcfCurrentModel != "" )
        _reloadRequested = ImGui::Button( "Reload" );

    ImGui::End();

    if ( _reloadRequested && m_mjcfCurrentModel != "" )
    {
        reload( std::string( TYSOC_PATH_WORKING_DIR ) + std::string( "tests/actuators/" ) + m_mjcfCurrentModel );

        m_currentActuatorName = "";
        m_currentAgentName = "";
        m_currentAgentIndx = -1;
    }
}

int main()
{

    auto _app = new AppExample();

    _app->init();
    _app->reset();

    while ( !_app->isTerminated() )
        _app->step();

    delete _app;

    return 0;
}