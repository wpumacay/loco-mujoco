
#include <mujoco_simulation.h>
#include <tysocMujocoViz.h>


static int NUM_AGENTS = 4;

#define SECTION_DEPTH 2.0f

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
    auto _tysocApi = new tysoc::mujoco::TTysocMjcApi();
    auto _factory = new tysoc::mujoco::TMjcFactory();

    tysoc::TGenericParams _terrainParams;
    // // sections - path - perlin profile
    // {
    //     _terrainParams.set( "sectionType", "path" );
    //     _terrainParams.set( "sectionDepth", SECTION_DEPTH );
    //     _terrainParams.set( "pathProfile", "perlin" );
    //     _terrainParams.set( "componentsSpacingX", 0.25f );
    //     _terrainParams.set( "componentsThickness", 0.2f );
    //     _terrainParams.set( "perlinProfileOctaves", 4 );
    //     _terrainParams.set( "perlinProfilePersistance", 0.5f );
    //     _terrainParams.set( "perlinProfileLacunarity", 2.0f );
    //     _terrainParams.set( "perlinProfileNoiseScale", 10.0f );
    // }
    {
        _terrainParams.set( "sectionType", "blocky" );
        _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        _terrainParams.set( "sectionLength", 250.0f );
        _terrainParams.set( "sectionUsesBase", 1 );
        _terrainParams.set( "sectionUsesSides", 1 );
        _terrainParams.set( "sectionBlockyBaseHeight", 0.05f );
        _terrainParams.set( "sectionBlockyBaseWidth", 0.75f );
        _terrainParams.set( "sectionBlockyBaseSpacingX", 4.0f );
        _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.75f );
        _terrainParams.set( "sectionBlockyPercentDepthMin", 0.5f );
        _terrainParams.set( "sectionBlockyPercentDepthMax", 0.75f );
        _terrainParams.set( "sectionBlockyPercentHeightMin", 0.75f );
        _terrainParams.set( "sectionBlockyPercentHeightMax", 1.25f );
        _terrainParams.set( "sectionBlockyPercentWidthMin", 0.5f );
        _terrainParams.set( "sectionBlockyPercentWidthMax", 2.0f );
        _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.9f );
        _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.1f );
        _terrainParams.set( "sectionBlockyPercentOffsetZMin", 0.75f );
        _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.25f );
    }


    auto _scenario = new tysoc::TScenario();
    _tysocApi->setScenario( _scenario );

    for ( size_t i = 0; i < NUM_AGENTS; i++ )
    {
        // create a terrain generator
        tysoc::TVec3 _startPosition = { 0.0f, i * ( SECTION_DEPTH + 1.0f ), 0.0f };
        _terrainParams.set( "startPosition", _startPosition );
        auto _terrain = _factory->createTerrainGen( std::string( "terrain_proc" ) + std::to_string( i ),
                                                    "procedural", _terrainParams );
        
        auto _terrainGen        = _terrain->terrainGenerator();
        auto _terrainGenInfo    = _terrainGen->generatorInfo();
        _terrainGenInfo->trackingpoint.x = 0.0f;
        _terrainGenInfo->trackingpoint.y = i * ( SECTION_DEPTH + 1.0f );
        _terrainGenInfo->trackingpoint.z = 0.0f;

        _tysocApi->addTerrainGenWrapper( _terrain );

        std::string _templateModel = ( i % 2 == 0 ) ? "humanoid" : "baxter";

        // create kintree agents
        auto _agent = _factory->createKinTreeAgentFromMjcf( _templateModel + std::to_string( i ),
                                                            _templateModel,
                                                            3.0f, i * ( SECTION_DEPTH + 1.0f ), 2.5f );

        // create some sensors
        auto _sensor1Name = _templateModel + std::string( "_sensor_" ) + std::to_string( i ) + std::string( "_pathterrain" );
        auto _sensor1 = new tysoc::sensor::TSectionsTerrainSensor( _sensor1Name,
                                                                   ( tysoc::terrain::TSectionsTerrainGenerator* )_terrain->terrainGenerator(),
                                                                       _agent->agent(), true );

        auto _sensor2Name = _templateModel + std::string( "_sensor_" ) + std::to_string( i ) + std::string( "_intrinsics" );
        auto _sensor2 = new tysoc::sensor::TAgentIntrinsicsSensor( _sensor2Name,
                                                                   _agent->agent() );

        // and add it to the runtime
        _tysocApi->addKinTreeAgentWrapper( _agent );
        _tysocApi->getScenario()->addSensor( _sensor1 );
        _tysocApi->getScenario()->addSensor( _sensor2 );
    }

    if ( !_tysocApi->initializeMjcApi() )
    {
        std::cout << "There was an error initializing the MjcApi" << std::endl;
        return 1;
    }

    /* ***************************************************************************/

    auto _viz = new tysoc::viz::TMujocoVisualizer( _scenario );
    _viz->setMjcModel( _tysocApi->getMjcModel() );
    _viz->setMjcData( _tysocApi->getMjcData() );
    _viz->setMjcScene( _tysocApi->getMjcScene() );
    _viz->setMjcCamera( _tysocApi->getMjcCamera() );
    _viz->setMjcOption( _tysocApi->getMjcOption() );
    _viz->initialize();

    float _currentX = 0.0f;

    while( _viz->isActive() )
    {
        // update api
        _tysocApi->step();

        // update visualizer
        _viz->update();

        if ( _viz->isKeyDown( tysoc::keys::KEY_A ) )
        {
            std::cout << "INFO> key A is down" << std::endl;
        }

        if ( _viz->checkSingleKeyPress( tysoc::keys::KEY_B ) )
        {
            std::cout << "INFO> key B has been pressed (single check)" << std::endl;
        }

        _currentX += 0.025f;

        // auto _terrainGens = _tysocApi->getScenario()->getTerrainGenerators();
        // for ( size_t i = 0; i < _terrainGens.size(); i++ )
        // {
        //     auto _genInfoPtr = _terrainGens[i]->generatorInfo();
        //     _genInfoPtr->trackingpoint.x = _currentX;
        // }

        // auto _iagents = _tysocApi->getAgents();
        // for ( size_t i = 0; i < _iagents.size(); i++ )
        // {
        //     if ( _iagents[i]->getType() != "kintree" )
        //     {
        //         continue;
        //     }
        //     auto _iagent = reinterpret_cast< tysoc::agent::TAgentKinTree* >( _iagents[i] );
        //     _iagent->setActions( tysoc::generateRandomArray( _iagent->getActionDim(),
        //                                                      -0.15, 
        //                                                      0.15 ) );
        // }
    }

    delete _viz;
    delete _tysocApi;

    return 0;
}
