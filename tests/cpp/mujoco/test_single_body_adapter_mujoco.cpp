
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_mujoco.h>
#include <adapters/loco_single_body_adapter_mujoco.h>

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterBuild )
{
    loco::TLogger::Init();

    auto scenario = std::make_unique<loco::TScenario>();

    std::vector<loco::eShapeType> vec_shape_types = { loco::eShapeType::BOX,
                                                      loco::eShapeType::SPHERE,
                                                      loco::eShapeType::PLANE,
                                                      loco::eShapeType::CYLINDER,
                                                      loco::eShapeType::CAPSULE,
                                                      loco::eShapeType::ELLIPSOID };
    std::vector<loco::TVec3> vec_shape_sizes = { { 0.1, 0.2, 0.3 },
                                                 { 0.1, 0.1, 0.1 },
                                                 { 10.0, 10.0, 1.0 },
                                                 { 0.2, 0.8, 0.2 },
                                                 { 0.2, 0.8, 0.2 },
                                                 { 0.2, 0.3, 0.4 } };
    std::vector<loco::TVec3> vec_positions = { { -1.0, -1.0, 1.0 }, // box
                                               { -1.0,  0.0, 1.0 }, // sphere
                                               {  0.0,  0.0, 0.0 }, // plane
                                               {  1.0,  1.0, 1.0 }, // cylinder
                                               {  1.0,  0.0, 1.0 }, // capsule
                                               {  0.0,  1.0, 1.0 } }; // ellipsoid
    for ( size_t i = 0; i < vec_shape_types.size(); i++ )
    {
        auto col_data = loco::TCollisionData();
        col_data.type = vec_shape_types[i];
        col_data.size = vec_shape_sizes[i];
        auto vis_data = loco::TVisualData();
        vis_data.type = vec_shape_types[i];
        vis_data.size = vec_shape_sizes[i];
        auto body_data = loco::TBodyData();
        body_data.collision = col_data;
        body_data.visual = vis_data;

        const auto body_name = loco::mujoco::enumShape_to_mjcShape( vec_shape_types[i] ) + "_body";
        auto body_obj = std::make_unique<loco::TSingleBody>( body_name, body_data, vec_positions[i], loco::TMat3() );

        scenario->AddSingleBody( std::move( body_obj ) );
    }

    auto simulation = std::make_unique<loco::mujoco::TMujocoSimulation>( scenario.get() );
    simulation->Initialize();

    auto single_bodies_list = scenario->GetSingleBodiesList();
    for ( auto single_body : single_bodies_list )
    {
        if ( auto single_body_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyAdapter*>( single_body->adapter() ) )
            LOCO_CORE_TRACE( "mjcf-xml-body:\n{0}", single_body_adapter->element_resources()->ToString() );
    }
}