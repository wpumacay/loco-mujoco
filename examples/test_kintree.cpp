
#include <mujoco_simulation.h>
#include <tysocCustomViz.h>

static int NUM_AGENTS = 6;

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

    tysoc::TGenericParams _terrainParams[6];
    // sections - path - perlin profile
    {
        _terrainParams[0].set( "sectionType", "path" );
        _terrainParams[0].set( "sectionDepth", SECTION_DEPTH );
        _terrainParams[0].set( "pathProfile", "perlin" );
        _terrainParams[0].set( "componentsSpacingX", 0.25f );
        _terrainParams[0].set( "componentsThickness", 0.2f );
        _terrainParams[0].set( "perlinProfileOctaves", 4 );
        _terrainParams[0].set( "perlinProfilePersistance", 0.5f );
        _terrainParams[0].set( "perlinProfileLacunarity", 2.0f );
        _terrainParams[0].set( "perlinProfileNoiseScale", 10.0f );
    }
    // sections - path - sine profile
    {
        _terrainParams[1].set( "sectionType", "path" );
        _terrainParams[1].set( "sectionDepth", SECTION_DEPTH );
        _terrainParams[1].set( "pathProfile", "sine" );
        _terrainParams[1].set( "componentsSpacingX", 0.25f );
        _terrainParams[1].set( "componentsThickness", 0.2f );
        _terrainParams[1].set( "sineProfileAmplitude", 1.0f );
        _terrainParams[1].set( "sineProfilePeriod", 10.0f );
        _terrainParams[1].set( "sineProfilePhase", 1.57f );
        _terrainParams[1].set( "componentsThickness", 0.2f );
    }
    // sections - blocky
    {
        _terrainParams[2].set( "sectionType", "blocky" );
        _terrainParams[2].set( "sectionDepth", SECTION_DEPTH );
        _terrainParams[2].set( "sectionLength", 250.0f );
        _terrainParams[2].set( "sectionUsesBase", 1 );
        _terrainParams[2].set( "sectionUsesSides", 1 );
        _terrainParams[2].set( "sectionBlockyBaseHeight", 0.5f );
        _terrainParams[2].set( "sectionBlockyBaseWidth", 0.25f );
        _terrainParams[2].set( "sectionBlockyBaseSpacingX", 4.0f );
        _terrainParams[2].set( "sectionBlockyBaseOffsetZ", 0.0f );
        _terrainParams[2].set( "sectionBlockyPercentDepthMin", 0.25f );//1.0f
        _terrainParams[2].set( "sectionBlockyPercentDepthMax", 0.75f );//1.0f
        _terrainParams[2].set( "sectionBlockyPercentHeightMin", 0.75f );
        _terrainParams[2].set( "sectionBlockyPercentHeightMax", 1.25f );
        _terrainParams[2].set( "sectionBlockyPercentWidthMin", 0.5f );
        _terrainParams[2].set( "sectionBlockyPercentWidthMax", 2.0f );
        _terrainParams[2].set( "sectionBlockyPercentSpacingXMin", 0.9f );
        _terrainParams[2].set( "sectionBlockyPercentSpacingXMax", 1.1f );
        _terrainParams[2].set( "sectionBlockyPercentOffsetZMin", 1.0f );
        _terrainParams[2].set( "sectionBlockyPercentOffsetZMax", 1.0f );

        _terrainParams[3].set( "sectionType", "blocky" );
        _terrainParams[3].set( "sectionDepth", SECTION_DEPTH );
        _terrainParams[3].set( "sectionLength", 250.0f );
        _terrainParams[3].set( "sectionUsesBase", 1 );
        _terrainParams[3].set( "sectionUsesSides", 0 );
        _terrainParams[3].set( "sectionBlockyBaseHeight", 0.5f );
        _terrainParams[3].set( "sectionBlockyBaseWidth", 0.25f );
        _terrainParams[3].set( "sectionBlockyBaseSpacingX", 4.0f );
        _terrainParams[3].set( "sectionBlockyBaseOffsetZ", 0.0f );
        _terrainParams[3].set( "sectionBlockyPercentDepthMin", 1.0f );
        _terrainParams[3].set( "sectionBlockyPercentDepthMax", 1.0f );
        _terrainParams[3].set( "sectionBlockyPercentHeightMin", 0.75f );
        _terrainParams[3].set( "sectionBlockyPercentHeightMax", 1.25f );
        _terrainParams[3].set( "sectionBlockyPercentWidthMin", 0.5f );
        _terrainParams[3].set( "sectionBlockyPercentWidthMax", 2.0f );
        _terrainParams[3].set( "sectionBlockyPercentSpacingXMin", 0.9f );
        _terrainParams[3].set( "sectionBlockyPercentSpacingXMax", 1.1f );
        _terrainParams[3].set( "sectionBlockyPercentOffsetZMin", 1.0f );
        _terrainParams[3].set( "sectionBlockyPercentOffsetZMax", 1.0f );

        _terrainParams[4].set( "sectionType", "blocky" );
        _terrainParams[4].set( "sectionDepth", SECTION_DEPTH );
        _terrainParams[4].set( "sectionLength", 250.0f );
        _terrainParams[4].set( "sectionUsesBase", 0 );
        _terrainParams[4].set( "sectionUsesSides", 0 );
        _terrainParams[4].set( "sectionBlockyBaseHeight", 0.1f );
        _terrainParams[4].set( "sectionBlockyBaseWidth", 1.0f );
        _terrainParams[4].set( "sectionBlockyBaseSpacingX", 2.5f );
        _terrainParams[4].set( "sectionBlockyBaseOffsetZ", 0.0f );
        _terrainParams[4].set( "sectionBlockyPercentDepthMin", 1.0f );
        _terrainParams[4].set( "sectionBlockyPercentDepthMax", 1.0f );
        _terrainParams[4].set( "sectionBlockyPercentHeightMin", 1.0f );
        _terrainParams[4].set( "sectionBlockyPercentHeightMax", 1.0f );
        _terrainParams[4].set( "sectionBlockyPercentWidthMin", 0.75f );
        _terrainParams[4].set( "sectionBlockyPercentWidthMax", 1.25f );
        _terrainParams[4].set( "sectionBlockyPercentSpacingXMin", 0.75f );
        _terrainParams[4].set( "sectionBlockyPercentSpacingXMax", 1.25f );
        _terrainParams[4].set( "sectionBlockyPercentOffsetZMin", 1.0f );
        _terrainParams[4].set( "sectionBlockyPercentOffsetZMax", 1.0f );

        _terrainParams[5].set( "sectionType", "blocky" );
        _terrainParams[5].set( "sectionDepth", SECTION_DEPTH );
        _terrainParams[5].set( "sectionLength", 250.0f );
        _terrainParams[5].set( "sectionUsesBase", 1 );
        _terrainParams[5].set( "sectionUsesSides", 1 );
        _terrainParams[5].set( "sectionBlockyBaseHeight", 0.05f );
        _terrainParams[5].set( "sectionBlockyBaseWidth", 0.75f );
        _terrainParams[5].set( "sectionBlockyBaseSpacingX", 4.0f );
        _terrainParams[5].set( "sectionBlockyBaseOffsetZ", 0.75f );
        _terrainParams[5].set( "sectionBlockyPercentDepthMin", 0.5f );
        _terrainParams[5].set( "sectionBlockyPercentDepthMax", 0.75f );
        _terrainParams[5].set( "sectionBlockyPercentHeightMin", 0.75f );
        _terrainParams[5].set( "sectionBlockyPercentHeightMax", 1.25f );
        _terrainParams[5].set( "sectionBlockyPercentWidthMin", 0.5f );
        _terrainParams[5].set( "sectionBlockyPercentWidthMax", 2.0f );
        _terrainParams[5].set( "sectionBlockyPercentSpacingXMin", 0.9f );
        _terrainParams[5].set( "sectionBlockyPercentSpacingXMax", 1.1f );
        _terrainParams[5].set( "sectionBlockyPercentOffsetZMin", 0.75f );
        _terrainParams[5].set( "sectionBlockyPercentOffsetZMax", 1.25f );
    }


    auto _scenario = new tysoc::TScenario();
    _tysocApi->setScenario( _scenario );

    for ( size_t i = 0; i < NUM_AGENTS; i++ )
    {
        // create a terrain generator
        tysoc::TVec3 _startPosition = { 0.0f, i * ( SECTION_DEPTH + 1.0f ), 0.0f };
        _terrainParams[i].set( "startPosition", _startPosition );
        auto _terrain = _factory->createTerrainGen( std::string( "terrain_proc" ) + std::to_string( i ),
                                                    "procedural", _terrainParams[i] );
        
        auto _terrainGen        = _terrain->terrainGenerator();
        auto _terrainGenInfo    = _terrainGen->generatorInfo();
        _terrainGenInfo->trackingpoint.x = 0.0f;
        _terrainGenInfo->trackingpoint.y = i * ( SECTION_DEPTH + 1.0f );
        _terrainGenInfo->trackingpoint.z = 0.0f;

        _tysocApi->addTerrainGenWrapper( _terrain );

        std::string _templateModel = ( i % 3 == 0 ) ? "walker" : ( ( i % 3  == 1 ) ? "walker" : "walker" );

        // create kintree agents
        auto _agent = _factory->createKinTreeAgentFromMjcf( _templateModel + std::to_string( i ),
                                                            _templateModel,
                                                            2.0f, i * ( SECTION_DEPTH + 1.0f ), 4.5f );

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
        // _tysocApi->getScenario()->addSensor( _sensor1 );
        // _tysocApi->getScenario()->addSensor( _sensor2 );
    }

    if ( !_tysocApi->initializeMjcApi() )
    {
        std::cout << "There was an error initializing the MjcApi" << std::endl;
        return 1;
    }

    /* ***************************************************************************/

    auto _viz = new tysoc::viz::TCustomVisualizer( _scenario );
    _viz->initialize();

    float _currentX = 2.0f;

    while( _viz->isActive() )
    {
        // update api
        _tysocApi->step();

        // update visualizer
        _viz->update();

        _currentX += 0.025f;

        // auto _terrainGens = _tysocApi->getScenario()->getTerrainGenerators();
        // for ( size_t i = 0; i < _terrainGens.size(); i++ )
        // {
        //     auto _genInfoPtr = _terrainGens[i]->generatorInfo();
        //     _genInfoPtr->trackingpoint.x = _currentX;
        // }

        // auto _ballAgent = _scenario->getAgentByName( "ball0" );
        // if ( _ballAgent )
        // {
        //     tysoc::mujoco::utils::setTerrainBodyPosition( _tysocApi->getMjcModel(),
        //                                                   _tysocApi->getMjcData(),
        //                                                   "body_ball0_rootbody",
        //                                                   { _currentX, 0.0f, 2.0f } );
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
