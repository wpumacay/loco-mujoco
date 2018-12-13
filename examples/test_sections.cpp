
#include <tysocMjc.h>
#include <tysocViz.h>


static int NUM_AGENTS = 2;

#define SECTION_DEPTH 3.0f

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

    tysoc::mujoco::TGenericParams _terrainParams;
    // sections - path - perlin profile
    {
        // _terrainParams.set( "sectionType", "path" );
        // _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        // _terrainParams.set( "pathProfile", "perlin" );
        // _terrainParams.set( "perlinProfileOctaves", 4 );
        // _terrainParams.set( "perlinProfilePersistance", 0.5f );
        // _terrainParams.set( "perlinProfileLacunarity", 2.0f );
        // _terrainParams.set( "perlinProfileNoiseScale", 10.0f );
    }
    // // sections - path - sine profile
    // {
    //     _terrainParams.set( "sectionType", "path" );
    //     _terrainParams.set( "sectionDepth", SECTION_DEPTH );
    //     _terrainParams.set( "pathProfile", "sine" );
    //     _terrainParams.set( "sineProfileAmplitude", 1.0f );
    //     _terrainParams.set( "sineProfilePeriod", 10.0f );
    //     _terrainParams.set( "sineProfilePhase", 1.57f );
    // }
    // sections - blocky
    {
        // _terrainParams.set( "sectionType", "blocky" );
        // _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        // _terrainParams.set( "sectionLength", 250.0f );
        // _terrainParams.set( "sectionUsesBase", 1 );
        // _terrainParams.set( "sectionUsesSides", 1 );
        // _terrainParams.set( "sectionBlockyBaseHeight", 0.5f );
        // _terrainParams.set( "sectionBlockyBaseWidth", 0.25f );
        // _terrainParams.set( "sectionBlockyBaseSpacingX", 4.0f );
        // _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.0f );
        // _terrainParams.set( "sectionBlockyPercentDepthMin", 0.25f );//1.0f
        // _terrainParams.set( "sectionBlockyPercentDepthMax", 0.75f );//1.0f
        // _terrainParams.set( "sectionBlockyPercentHeightMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentHeightMax", 1.25f );
        // _terrainParams.set( "sectionBlockyPercentWidthMin", 0.5f );
        // _terrainParams.set( "sectionBlockyPercentWidthMax", 2.0f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.9f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.1f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.0f );

        _terrainParams.set( "sectionType", "blocky" );
        _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        _terrainParams.set( "sectionLength", 250.0f );
        _terrainParams.set( "sectionUsesBase", 1 );
        _terrainParams.set( "sectionUsesSides", 0 );
        _terrainParams.set( "sectionBlockyBaseHeight", 0.5f );
        _terrainParams.set( "sectionBlockyBaseWidth", 0.25f );
        _terrainParams.set( "sectionBlockyBaseSpacingX", 4.0f );
        _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.0f );
        _terrainParams.set( "sectionBlockyPercentDepthMin", 1.0f );
        _terrainParams.set( "sectionBlockyPercentDepthMax", 1.0f );
        _terrainParams.set( "sectionBlockyPercentHeightMin", 0.75f );
        _terrainParams.set( "sectionBlockyPercentHeightMax", 1.25f );
        _terrainParams.set( "sectionBlockyPercentWidthMin", 0.5f );
        _terrainParams.set( "sectionBlockyPercentWidthMax", 2.0f );
        _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.9f );
        _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.1f );
        _terrainParams.set( "sectionBlockyPercentOffsetZMin", 1.0f );
        _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.0f );

        // _terrainParams.set( "sectionType", "blocky" );
        // _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        // _terrainParams.set( "sectionLength", 250.0f );
        // _terrainParams.set( "sectionUsesBase", 0 );
        // _terrainParams.set( "sectionUsesSides", 0 );
        // _terrainParams.set( "sectionBlockyBaseHeight", 0.1f );
        // _terrainParams.set( "sectionBlockyBaseWidth", 1.0f );
        // _terrainParams.set( "sectionBlockyBaseSpacingX", 2.5f );
        // _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.0f );
        // _terrainParams.set( "sectionBlockyPercentDepthMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentDepthMax", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentHeightMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentHeightMax", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentWidthMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentWidthMax", 1.25f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.25f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.0f );

        // _terrainParams.set( "sectionType", "blocky" );
        // _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        // _terrainParams.set( "sectionLength", 250.0f );
        // _terrainParams.set( "sectionUsesBase", 1 );
        // _terrainParams.set( "sectionUsesSides", 1 );
        // _terrainParams.set( "sectionBlockyBaseHeight", 0.05f );
        // _terrainParams.set( "sectionBlockyBaseWidth", 0.75f );
        // _terrainParams.set( "sectionBlockyBaseSpacingX", 4.0f );
        // _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentDepthMin", 0.5f );
        // _terrainParams.set( "sectionBlockyPercentDepthMax", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentHeightMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentHeightMax", 1.25f );
        // _terrainParams.set( "sectionBlockyPercentWidthMin", 0.5f );
        // _terrainParams.set( "sectionBlockyPercentWidthMax", 2.0f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.9f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.1f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.25f );
    }


    auto _scenario = new tysoc::TScenario();
    _tysocApi->setScenario( _scenario );

    for ( size_t i = 0; i < NUM_AGENTS; i++ )
    {
        // create agent wrapper
        auto _agent = _factory->createAgent( std::string( "walker_" ) + std::to_string( i ),
                                             "ball",
                                             2.0f, i * ( SECTION_DEPTH + 1.0f ), 1.5f );

        // create agent wrapper
        mjcf::Vec3 _startPosition = { 0.0f, i * ( SECTION_DEPTH + 1.0f ), 0.0f };
        _terrainParams.set( "startPosition", _startPosition );
        auto _terrain = _factory->createTerrainGen( std::string( "terrain_proc" ) + std::to_string( i ),
                                                    "procedural", _terrainParams );
        
        auto _terrainGen        = _terrain->terrainGenerator();
        auto _terrainGenInfo    = _terrainGen->generatorInfo();
        _terrainGenInfo->trackingpoint.x = 0.0f;
        _terrainGenInfo->trackingpoint.y = i * ( SECTION_DEPTH + 1.0f );
        _terrainGenInfo->trackingpoint.z = 0.0f;

        // create a sensor
        auto _sensor1Name = std::string( "walker_sensor_" ) + std::to_string( i ) + std::string( "_pathterrain" );
        auto _sensor1 = new tysoc::sensor::TSectionsTerrainSensor( _sensor1Name,
                                                                 ( tysoc::terrain::TSectionsTerrainGenerator* )_terrain->terrainGenerator(),
                                                                 _agent->agent(), true );

        auto _sensor2Name = std::string( "walker_sensor_" ) + std::to_string( i ) + std::string( "_intrinsics" );
        auto _sensor2 = new tysoc::sensor::TAgentIntrinsicsSensor( _sensor2Name,
                                                                 _agent->agent() );

        _tysocApi->addAgentWrapper( _agent );
        _tysocApi->addTerrainGenWrapper( _terrain );
        _tysocApi->getScenario()->addSensor( _sensor1 );
        _tysocApi->getScenario()->addSensor( _sensor2 );
    }

    if ( !_tysocApi->initializeMjcApi() )
    {
        std::cout << "There was an error initializing the MjcApi" << std::endl;
        return 1;
    }

    /* ***************************************************************************/

    auto _viz = new tysoc::viz::TVisualizer( _tysocApi );
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
            _tysocApi->setAgentPosition( it->first, _currentX - 2.25f, y, z );
        }

    }

    delete _viz;
    delete _tysocApi;

    return 0;
}
