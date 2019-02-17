
#include <mujoco_simulation.h>

static std::string MODEL_NAME = "laikago";

int main( int argc, const char** argv )
{
    if ( argc > 1 )
    {
        try
        {
            MODEL_NAME = std::string( argv[1] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> Should pass an string for the model template" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }
    }

    /* ***************************************************************************/
    auto _tysocApi = new tysoc::mujoco::TTysocMjcApi();
    auto _factory = new tysoc::mujoco::TMjcFactory();

    // create kintree agents
    auto _agent = _factory->createKinTreeAgentFromUrdf( MODEL_NAME,
                                                        MODEL_NAME,
                                                        0.0, 0.0, 0.25 );
    if ( !_agent )
        return 1;
    
    // and add it to the runtime
    _tysocApi->addKinTreeAgentWrapper( _agent );

    if ( !_tysocApi->initializeMjcApi() )
    {
        std::cout << "There was an error initializing the MjcApi" << std::endl;
        return 1;
    }

    /* ***************************************************************************/

    // delete _tysocApi;

    return 0;
}
