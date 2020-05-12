
#include <loco.h>
#include <gtest/gtest.h>

TEST( TestLocoRuntimeMujocoBackend, TestRuntimeMujocoBackend )
{
    loco::InitUtils();

    auto vis_data = loco::TVisualData();
    vis_data.type = loco::eShapeType::CAPSULE;
    vis_data.size = { 0.1, 0.2, 0.1 };
    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::CAPSULE;
    col_data.size = { 0.1, 0.2, 0.1 };

    auto body_data = loco::TBodyData();
    body_data.dyntype = loco::eDynamicsType::DYNAMIC;
    body_data.collision = col_data;
    body_data.visual = vis_data;

    auto body_obj = std::make_unique<loco::TSingleBody>( "body_0", body_data, tinymath::Vector3f( 1.0, 1.0, 1.0 ), tinymath::Matrix3f() );
    auto scenario = std::make_unique<loco::TScenario>();
    scenario->AddSingleBody( std::move( body_obj ) );

    auto runtime = std::make_unique<loco::TRuntime>( loco::config::physics::MUJOCO,
                                                     loco::config::rendering::NONE );

    auto simulationRef = runtime->CreateSimulation( scenario.get() );
    simulationRef->Step();
    simulationRef->Reset();
    simulationRef->Pause();
    simulationRef->Resume();
    EXPECT_EQ( simulationRef->backendId(), "MUJOCO" );

    auto visualizerRef = runtime->CreateVisualizer( scenario.get() );
    auto cameraRef = visualizerRef->CreateCamera( "cam_orbit_0", 
                                                  loco::eVizCameraType::ORBIT,
                                                  { 3.0f, 3.0f, 3.0f },
                                                  { 0.0f, 0.0f, 0.0f } );
    auto lightRef = visualizerRef->CreateLight( "light_point_0",
                                                loco::eVizLightType::POINT,
                                                { 0.4f, 0.4f, 0.4f },
                                                { 0.8f, 0.8f, 0.8f },
                                                { 0.8f, 0.8f, 0.8f } );
    visualizerRef->Render();
    visualizerRef->Reset();
    EXPECT_EQ( visualizerRef->backendId(), "null" );
    EXPECT_TRUE( visualizerRef->HasCameraNamed( "cam_orbit_0" ) );
    EXPECT_TRUE( visualizerRef->HasLightNamed( "light_point_0" ) );
    EXPECT_TRUE( visualizerRef->GetCameraByName( "cam_orbit_0" ) != nullptr );
    EXPECT_TRUE( visualizerRef->GetLightByName( "light_point_0" ) != nullptr );
    EXPECT_TRUE( tinymath::allclose( visualizerRef->GetCameraByName( "cam_orbit_0" )->position(), cameraRef->position() ) );
    EXPECT_TRUE( tinymath::allclose( visualizerRef->GetLightByName( "light_point_0" )->ambient(), lightRef->ambient() ) );

    runtime->DestroySimulation();
    runtime->DestroyVisualizer();
}