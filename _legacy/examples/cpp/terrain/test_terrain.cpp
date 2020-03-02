
#include <chrono>
#include <runtime.h>
#include <model_loader.h>
#include <mujoco_config.h>

const float SECTION_DEPTH = 5.0f;

static std::string MODEL_FORMAT = "rlsim";
static std::string MODEL_NAME = "dog3d";

tysoc::TAgent* createAgent( const std::string& format,
                                   const std::string& modelName,
                                   const std::string& agentName,
                                   const tysoc::TVec3& position,
                                   const tysoc::TVec3& rotation = tysoc::TVec3() )
{
    auto _modelLoader = tysoc::TModelLoader::Create();

    if ( format == "urdf" )
    {
        auto _modelData = _modelLoader->getUrdfModel( modelName );

        return new tysoc::TAgent( _modelData, agentName, position, rotation );
    }
    else if ( format == "rlsim" )
    {
        auto _modelData = _modelLoader->getRlsimModel( modelName );
        
        return new tysoc::TAgent( _modelData, agentName, position, rotation );
    }
    else if ( format == "mjcf" )
    {
        auto _modelData = _modelLoader->getMjcfModel( modelName );
        
        return new tysoc::TAgent( _modelData, agentName, position, rotation );
    }

    std::cout << "ERROR> format: " << format << " not supported" << std::endl;
    return nullptr;
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

    auto _scenario = new tysoc::TScenario();

    auto _agent0 = createAgent( MODEL_FORMAT, MODEL_NAME, "agent0", { 5.0f, 0.0f, 1.5f }, { 0.0f, 0.0f, TYSOC_PI / 2.0f } );
    auto _agent1 = createAgent( MODEL_FORMAT, MODEL_NAME, "agent1", { 5.0f, 2.0f * SECTION_DEPTH, 1.5f }, { 0.0f, 0.0f, TYSOC_PI / 2.0f } );

    _scenario->addAgent( _agent0 );
    _scenario->addAgent( _agent1 );

    auto _terrainGenStatic = new tysoc::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "plane", 
                                        { 200.0f, SECTION_DEPTH, 0.2f }, 
                                        { 0.0f, 0.0f, 0.0f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f } );
    _terrainGenStatic->createPrimitive( "plane", 
                                        { 200.0f, SECTION_DEPTH, 0.2f }, 
                                        { 0.0f, 2.0f * SECTION_DEPTH, 0.0f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f } );
    _scenario->addTerrainGenerator( _terrainGenStatic );

    auto _terrainGenParams1 = tysoc::TBlockyParams();
    _terrainGenParams1.usesBase          = false;
    _terrainGenParams1.usesSides         = false;
    _terrainGenParams1.sectionLength     = 20.0f;
    _terrainGenParams1.baseDepth         = SECTION_DEPTH;
    _terrainGenParams1.baseHeight        = 0.5f;
    _terrainGenParams1.baseWidth         = 0.25f;
    _terrainGenParams1.baseSpacingX      = 4.0f;
    _terrainGenParams1.baseOffsetZ       = 0.0f;
    _terrainGenParams1.percentDepth      = { 1.0f, 1.0f };
    _terrainGenParams1.percentHeight     = { 0.75f, 1.25f };
    _terrainGenParams1.percentWidth      = { 0.5f, 2.0f };
    _terrainGenParams1.percentSpacingX   = { 0.9f, 1.1f };
    _terrainGenParams1.percentOffsetZ    = { 1.0f, 1.0f };

    auto _terrainGenBlocky1 = new tysoc::TBlockyTerrainGenerator( "terrainGen1", 0.0f, 0.0f, 0.0f, _terrainGenParams1 );
    _terrainGenBlocky1->generatorInfo()->trackingpoint = { 7.5f, 0.0f, 0.0f };
    _terrainGenBlocky1->initialize();

    _scenario->addTerrainGenerator( _terrainGenBlocky1 );

    auto _terrainGenParams2 = tysoc::TBlockyParams();
    _terrainGenParams2.usesBase          = false;
    _terrainGenParams2.usesSides         = true;
    _terrainGenParams2.sectionLength     = 20.0f;
    _terrainGenParams2.baseDepth         = SECTION_DEPTH;
    _terrainGenParams2.baseHeight        = 0.5f;
    _terrainGenParams2.baseWidth         = 0.25f;
    _terrainGenParams2.baseSpacingX      = 4.0f;
    _terrainGenParams2.baseOffsetZ       = 0.0f;
    _terrainGenParams2.percentDepth      = { 0.25f, 0.75f };
    _terrainGenParams2.percentHeight     = { 0.75f, 1.25f };
    _terrainGenParams2.percentWidth      = { 0.5f, 2.0f };
    _terrainGenParams2.percentSpacingX   = { 0.9f, 1.1f };
    _terrainGenParams2.percentOffsetZ    = { 1.0f, 1.0f };

    auto _terrainGenBlocky2 = new tysoc::TBlockyTerrainGenerator( "terrainGen2", 10.0f, 0.0f, 0.0f, _terrainGenParams2 );
    _terrainGenBlocky2->generatorInfo()->trackingpoint = { 17.5f, 0.0f, 0.0f };
    _terrainGenBlocky2->initialize();

    _scenario->addTerrainGenerator( _terrainGenBlocky2 );

    auto _terrainGenParams3 = tysoc::TBlockyParams();
    _terrainGenParams3.usesBase          = false;
    _terrainGenParams3.usesSides         = true;
    _terrainGenParams3.sectionLength     = 20.0f;
    _terrainGenParams3.baseDepth         = SECTION_DEPTH;
    _terrainGenParams3.baseHeight        = 0.05f;
    _terrainGenParams3.baseWidth         = 0.75f;
    _terrainGenParams3.baseSpacingX      = 4.0f;
    _terrainGenParams3.baseOffsetZ       = 0.75f;
    _terrainGenParams3.percentDepth      = { 0.5f, 0.75f };
    _terrainGenParams3.percentHeight     = { 0.75f, 1.25f };
    _terrainGenParams3.percentWidth      = { 0.5f, 2.0f };
    _terrainGenParams3.percentSpacingX   = { 0.9f, 1.1f };
    _terrainGenParams3.percentOffsetZ    = { 0.75f, 1.25f };

    auto _terrainGenBlocky3 = new tysoc::TBlockyTerrainGenerator( "terrainGen3", 0.0f, 2.0f * SECTION_DEPTH, 0.0f, _terrainGenParams3 );
    _terrainGenBlocky3->generatorInfo()->trackingpoint = { 7.5f, 2.0f * SECTION_DEPTH, 0.0f };
    _terrainGenBlocky3->initialize();

    _scenario->addTerrainGenerator( _terrainGenBlocky3 );

    auto _terrainGenParams4 = tysoc::TBlockyParams();
    _terrainGenParams4.usesBase          = false;
    _terrainGenParams4.usesSides         = true;
    _terrainGenParams4.sectionLength     = 20.0f;
    _terrainGenParams4.baseDepth         = SECTION_DEPTH;
    _terrainGenParams4.baseHeight        = 3.0f;
    _terrainGenParams4.baseWidth         = 0.25f;
    _terrainGenParams4.baseSpacingX      = 4.0f;
    _terrainGenParams4.baseOffsetZ       = 0.0f;
    _terrainGenParams4.percentDepth      = { 0.25f, 0.75f };
    _terrainGenParams4.percentHeight     = { 1.0f, 1.0f };
    _terrainGenParams4.percentWidth      = { 0.5f, 2.0f };
    _terrainGenParams4.percentSpacingX   = { 0.9f, 1.1f };
    _terrainGenParams4.percentOffsetZ    = { 1.0f, 1.0f };

    auto _terrainGenBlocky4 = new tysoc::TBlockyTerrainGenerator( "terrainGen4", 10.0f, 2.0f * SECTION_DEPTH, 0.0f, _terrainGenParams4 );
    _terrainGenBlocky4->generatorInfo()->trackingpoint = { 17.5f, 2.0f * SECTION_DEPTH, 0.0f };
    _terrainGenBlocky4->initialize();

    _scenario->addTerrainGenerator( _terrainGenBlocky4 );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO, 
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    _simulation->togglePause();

    while ( _visualizer->isActive() )
    {
        //// auto _start = std::chrono::high_resolution_clock::now();

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_P ) )
            _simulation->togglePause();

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_ESCAPE ) )
            break;

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_R ) )
            _simulation->reset();

        _simulation->step();

        _visualizer->update();

        //// auto _duration = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::high_resolution_clock::now() - _start );
        //// std::cout << "step-time: " << _duration.count() << " ||| fps: " << ( 1000.0 / _duration.count() ) << std::endl;
    }

    _runtime->destroyVisualizer();
    _runtime->destroySimulation();
    _visualizer = NULL;
    _simulation = NULL;

    return 0;
}
