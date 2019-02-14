
#include <tysocMjc.h>
#include <tysocCustomViz.h>

static std::string MODEL_FORMAT = "mjcf";
static std::string MODEL_NAME = "humanoid";

static std::string TYSOC_MJCF_TEMPLATES     = std::string( TYSOCMJC_RESOURCES_PATH ) + std::string( "templates/mjcf/" );
static std::string TYSOC_URDF_TEMPLATES     = std::string( TYSOCMJC_RESOURCES_PATH ) + std::string( "templates/urdf/" );
static std::string TYSOC_RLSIM_TEMPLATES    = std::string( TYSOCMJC_RESOURCES_PATH ) + std::string( "templates/rlsim/" );

tysoc::agent::TAgentKinTree* createAgent( const std::string& format,
                                          const std::string& modelName,
                                          const std::string& agentName,
                                          const tysoc::TVec3& position )
{
    if ( format == "urdf" )
    {
        auto _modelPath = TYSOC_URDF_TEMPLATES + modelName + std::string( ".urdf" );
        auto _modelData = tysoc::urdf::loadGenericModel( _modelPath );

        return tysoc::agent::createKinTreeAgent( agentName, position, _modelData );
    }
    else if ( format == "rlsim" )
    {
        auto _modelPath = TYSOC_RLSIM_TEMPLATES + modelName + std::string( ".json" );
        auto _modelData = tysoc::rlsim::loadGenericModel( _modelPath );
        
        return tysoc::agent::createKinTreeAgent( agentName, position, _modelData );
    }
    else if ( format == "mjcf" )
    {
        auto _modelPath = TYSOC_MJCF_TEMPLATES + modelName + std::string( ".xml" );
        auto _modelData = tysoc::mjcf::loadGenericModel( _modelPath );
        
        return tysoc::agent::createKinTreeAgent( agentName, position, _modelData );
    }

    std::cout << "ERROR> format: " << format << " not supported" << std::endl;
    return NULL;
}

static tysoc::TTysocCommonApi* g_runtime = NULL;
static tysoc::mujoco::TTysocMjcApi* g_apiMujoco = NULL;

tysoc::TTysocCommonApi* initialize_runtime( const std::string& apiType,
                                            tysoc::TScenario* scenarioPtr )
{
    if ( apiType == tysoc::API_TYPE_MUJOCO )
    {
        if ( !g_apiMujoco )
        {
            g_apiMujoco = new tysoc::mujoco::TTysocMjcApi();
            g_apiMujoco->setScenario( scenarioPtr );
            // @TODO|@CHECK: Should redesign the initialization mechanism
            g_apiMujoco->collectFromScenario();

            if ( !g_apiMujoco->initializeMjcApi() )
            {
                std::cout << "WARNING> There was an error while initializing the mjcApi" << std::endl;
            }
            else
            {
                std::cout << "INFO> Successfully created mujoco Api" << std::endl;
            }
        }
        
        return g_apiMujoco;
    }

    std::cout << "WARNING> Api type: " << apiType << " is not supported" << std::endl;
    return NULL;
}

int main( int argc, const char** argv )
{
    if ( argc > 2 )
    {
        try
        {
            MODEL_FORMAT = std::string( argv[1] );
            MODEL_NAME = std::string( argv[2] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> should pass FORMAT(mjcf|urdf|rlsim) and MODEL_NAME(see templates)" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }
    }

    /* ***************************************************************************/

    auto _scenario = new tysoc::TScenario();

    auto _agent = createAgent( MODEL_FORMAT, MODEL_NAME, "agent0", { 0.0f, 0.0f, 0.0f } );

    if ( !_agent )
    {
        std::cout << "ERROR> (format|model): " 
                  << MODEL_FORMAT << "|" << MODEL_NAME 
                  << " not found" << std::endl;
        return 1;
    }

    _scenario->addAgent( _agent );

    auto _viz = new tysoc::viz::TCustomVisualizer( _scenario );
    _viz->initialize();

    while( _viz->isActive() )
    {
        // update visualizer
        _viz->update();

        if ( g_runtime )
        {
            g_runtime->step();
        }

        if ( _viz->checkSingleKeyPress( tysoc::keys::KEY_B ) )
        {
            g_runtime = initialize_runtime( tysoc::API_TYPE_MUJOCO,
                                            _scenario );
        }
    }

    delete _viz;

    return 0;
}
