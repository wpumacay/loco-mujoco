
#include <runtime.h>

static std::string MODEL_FORMAT = "urdf";
static std::string MODEL_NAME = "laikago";

static std::string TYSOC_MJCF_TEMPLATES     = std::string( TYSOC_PATH_MJCF_TEMPLATES );
static std::string TYSOC_URDF_TEMPLATES     = std::string( TYSOC_PATH_URDF_TEMPLATES );
static std::string TYSOC_RLSIM_TEMPLATES    = std::string( TYSOC_PATH_RLSIM_TEMPLATES );

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

static tysoc::TRuntime* g_runtime = NULL;
static tysoc::TISimulation* g_simulation = NULL;
static tysoc::TIVisualizer* g_visualizer = NULL;

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

    auto _agent = createAgent( MODEL_FORMAT, MODEL_NAME, "agent0", { 0.0f, 0.0f, 1.0f } );

    if ( !_agent )
    {
        std::cout << "ERROR> (format|model): " 
                  << MODEL_FORMAT << "|" << MODEL_NAME 
                  << " not found" << std::endl;
        return 1;
    }

    _scenario->addAgent( _agent );

    g_runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO, 
                                     tysoc::config::rendering::MJCVIZ );

    g_visualizer = g_runtime->createVisualizer( _scenario );
    g_visualizer->initialize();

    while( g_visualizer->isActive() )
    {
        if ( g_simulation )
            g_simulation->step();

        if ( g_visualizer )
            g_visualizer->update();

        if ( g_visualizer->checkSingleKeyPress( tysoc::keys::KEY_B ) )
        {
            g_simulation = g_runtime->createSimulation( _scenario );
            g_simulation->initialize();
        }

        if ( g_visualizer->checkSingleKeyPress( tysoc::keys::KEY_R ) )
        {
            if ( g_simulation )
            {
                g_simulation->reset();
                g_simulation = NULL;
                g_runtime->destroySimulation();
            }
        }
    }

    g_runtime->destroyVisualizer();
    g_runtime->destroySimulation();
    g_visualizer = NULL;
    g_simulation = NULL;

    return 0;
}
