
#include <tysocMjc.h>
#include <tysocViz.h>


static int NUM_AGENTS = 5;

int main( int argc, const char** argv )
{
    if ( argc > 1 )
    {
        try
        {
            NUM_AGENTS = std::stoi( argv[1] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> Should pass an int for numagents" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }
    }

    /* ***************************************************************************/
    auto _tysocApi = new tysocMjc::TTysocMjcApi();
    auto _factory = new tysocMjc::TMjcFactory();

    tysocMjc::TGenericParams _terrainParams;
    // perlin params
    {
        _terrainParams.set( "profiler", "perlin" );
        _terrainParams.set( "perlinProfileOctaves", 4 );
        _terrainParams.set( "perlinProfilePersistance", 0.5f );
        _terrainParams.set( "perlinProfileLacunarity", 2.0f );
        _terrainParams.set( "perlinProfileNoiseScale", 10.0f );
    }
    // sine params
    {
        // _terrainParams.set( "profiler", "sine" );
        // _terrainParams.set( "sineProfileAmplitude", 2.0f );
        // _terrainParams.set( "sineProfilePeriod", 10.0f );
        // _terrainParams.set( "sineProfilePhase", 1.57f );
    }

    _terrainParams.set( "profileDeltaX", 0.25f );
    _terrainParams.set( "profileDepth", 1.0f );
    _terrainParams.set( "profileTickness", 0.01f );


    for ( size_t i = 0; i < NUM_AGENTS; i++ )
    {
        // create agent wrapper
        auto _agent = _factory->createAgent( std::string( "walker_" ) + std::to_string( i ),
                                             "ball",
                                             1.0f, i * 2.5f, 1.5f );

        // create agent wrapper
        mjcf::Vec3 _startPosition = { 0.0f, i * 2.5f, 0.0f };
        _terrainParams.set( "startPosition", _startPosition );
        auto _terrain = _factory->createTerrainGen( std::string( "terrain_proc" ) + std::to_string( i ),
                                                    "procedural", _terrainParams );
        
        auto _terrainGen        = _terrain->terrainGenerator();
        auto _terrainGenInfo    = _terrainGen->generatorInfo();
        _terrainGenInfo->trackingpoint.x = 0.0f;
        _terrainGenInfo->trackingpoint.y = i * 2.5f;
        _terrainGenInfo->trackingpoint.z = 0.0f;

        _tysocApi->addAgentWrapper( _agent );
        _tysocApi->addTerrainGenWrapper( _terrain );
    }

    auto _scenario = new tysoc::TScenario();
    _tysocApi->setScenario( _scenario );

    if ( !_tysocApi->initializeMjcApi() )
    {
        std::cout << "There was an error initializing the MjcApi" << std::endl;
        return 1;
    }

    /* ***************************************************************************/

    auto _viz = new tysocViz::TVisualizer( _tysocApi );
    _viz->initialize();

    float _currentX = 0.0f;

    while( _viz->isActive() )
    {
        // update api
        _tysocApi->step();

        // update visualizer
        _viz->update();

        _currentX += 0.025f;

        auto _terrainGens = _tysocApi->getTerrainGenerators();
        for ( size_t i = 0; i < _terrainGens.size(); i++ )
        {
            auto _genInfoPtr = _terrainGens[i]->generatorInfo();
            _genInfoPtr->trackingpoint.x = _currentX;
        }


        // for ( size_t i = 0; i < NUM_AGENTS; i++ )
        // {
        //     auto _agentName = std::string( "walker_" ) + std::to_string( i );
        //     auto _actuatorName = std::string( "mjcact_" ) + _agentName + std::string( "_right_hip" );
        //     _tysocApi->setAgentAction( _agentName, _actuatorName, std::cos( _tysocApi->getMjcData()->time ) );
        // }

        auto _agents = _tysocApi->getAgents();
        for ( auto it = _agents.begin(); it != _agents.end(); it++ )
        {
            float x, y, z;
            _tysocApi->getAgentPosition( it->first, x, y, z );
            _tysocApi->setAgentPosition( it->first, _currentX - 0.5f, y, z );
        }

    }

    delete _viz;
    delete _tysocApi;

    return 0;
}
