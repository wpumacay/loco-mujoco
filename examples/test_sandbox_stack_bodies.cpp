

// Includes from core functionality
#include <runtime.h>
// Includes from mujoco functionality
#include <mujoco_common.h>

tysoc::sandbox::TBody* createBody( const std::string& name,
                                   const tysoc::TVec3& position,
                                   const tysoc::TVec3& size,
                                   const tysoc::TVec3& color )
{
    auto _bbox = new tysoc::sandbox::TFreeBody();
    _bbox->name = name;
    _bbox->type = "box";
    _bbox->size = size;
    _bbox->color = color;
    _bbox->worldTransform.setPosition( position );

    return _bbox;
}

int main()
{
    auto _terrainGenStatic = new tysoc::terrain::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "plane", 
                                        { 10.0f, 10.0f, 0.1f }, 
                                        { 0.0f, 0.0f, 0.0f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f },
                                        "chessboard" );

    auto _scenario = new tysoc::TScenario();
    _scenario->addTerrainGenerator( _terrainGenStatic );

    for ( size_t k = 0; k < 5; k++ )
    {
        for ( size_t i = 0; i < 5; i++ )
        {
            for ( size_t j = 0; j < 5; j++ )
            {
                auto _name = std::string( "bbox" ) + "_" + std::to_string( i ) + "_" + std::to_string( j ) + "_" + std::to_string( k );
                auto _body = createBody( _name,
                                         { 0.2f * i, 0.2f * j, 2 + 0.2f * k },
                                         { 0.15, 0.15, 0.15 },
                                         { 0.25f + 0.75f * ( i / 4.0f ),
                                           0.25f + 0.75f * ( j / 4.0f ),
                                           0.25f + 0.75f * ( k / 4.0f ) } );

                _scenario->addBody( _body );
            }
        }
    }

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO,
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    while ( _visualizer->isActive() )
    {
        _simulation->step();

        _visualizer->update();
    }

    return 0;
}