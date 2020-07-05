
#include <loco.h>
#include <gtest/gtest.h>

TEST( TestLocoRemoveObjects, TestRemoveSingleBody )
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

    auto body_obj = std::make_unique<loco::primitives::TSingleBody>( "body_0", body_data, tinymath::Vector3f( 1.0, 1.0, 1.0 ), tinymath::Matrix3f() );
    auto scenario = std::make_unique<loco::TScenario>();
    scenario->AddSingleBody( std::move( body_obj ) );

    auto runtime = std::make_unique<loco::TRuntime>( loco::config::physics::MUJOCO,
                                                     loco::config::rendering::GLVIZ_GLFW );

    auto simulation_ref = runtime->CreateSimulation( scenario.get() );
    auto visualizer_ref = runtime->CreateVisualizer( scenario.get() );
    simulation_ref->Step();
    visualizer_ref->Render();

    EXPECT_EQ( scenario->HasSingleBodyNamed( "body_0" ), true );
    EXPECT_EQ( scenario->GetNumSingleBodies(), 1 );

    scenario->RemoveSingleBodyByName( "body_0" );
    simulation_ref->Step();
    simulation_ref->Step();
    visualizer_ref->Render();

    EXPECT_EQ( scenario->HasSingleBodyNamed( "body_0" ), false );
    EXPECT_EQ( scenario->GetNumSingleBodies(), 0 );

    runtime->DestroySimulation();
    runtime->DestroyVisualizer();
}

TEST( TestLocoRemoveObjects, TestRemoveDrawable )
{
    loco::InitUtils();

    auto vis_data = loco::TVisualData();
    vis_data.type = loco::eShapeType::CAPSULE;
    vis_data.size = { 0.1, 0.2, 0.1 };

    auto drawable = std::make_unique<loco::visualizer::TDrawable>( "drawable_0", vis_data );
    auto scenario = std::make_unique<loco::TScenario>();
    scenario->AddDrawable( std::move( drawable ) );

    auto runtime = std::make_unique<loco::TRuntime>( loco::config::physics::MUJOCO,
                                                     loco::config::rendering::GLVIZ_GLFW );

    auto simulation_ref = runtime->CreateSimulation( scenario.get() );
    auto visualizer_ref = runtime->CreateVisualizer( scenario.get() );
    simulation_ref->Step();
    visualizer_ref->Render();

    EXPECT_EQ( scenario->HasDrawableNamed( "drawable_0" ), true );
    EXPECT_EQ( scenario->GetNumDrawables(), 1 );

    scenario->RemoveDrawableByName( "drawable_0" );
    simulation_ref->Step();
    visualizer_ref->Render();

    EXPECT_EQ( scenario->HasDrawableNamed( "drawable_0" ), false );
    EXPECT_EQ( scenario->GetNumDrawables(), 0 );

    runtime->DestroySimulation();
    runtime->DestroyVisualizer();
}