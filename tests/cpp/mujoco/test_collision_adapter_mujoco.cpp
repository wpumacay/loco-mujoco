
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_mujoco.h>
#include <adapters/loco_collision_adapter_mujoco.h>

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterBuild )
{
    loco::TLogger::Init();

    std::vector<loco::eShapeType> vec_col_types = { loco::eShapeType::BOX,
                                                    loco::eShapeType::SPHERE,
                                                    loco::eShapeType::PLANE,
                                                    loco::eShapeType::CYLINDER,
                                                    loco::eShapeType::CAPSULE,
                                                    loco::eShapeType::ELLIPSOID };
    std::vector<loco::TVec3> vec_col_sizes = { { 0.1, 0.2, 0.3 },
                                               { 0.1, 0.1, 0.1 },
                                               { 10.0, 10.0, 1.0 },
                                               { 0.2, 0.8, 0.2 },
                                               { 0.2, 0.8, 0.2 },
                                               { 0.2, 0.3, 0.4 } };
    std::vector<loco::TCollisionData> vec_col_data;
    for ( size_t i = 0; i < vec_col_types.size(); i++ )
    {
        auto col_data = loco::TCollisionData();
        col_data.type = vec_col_types[i];
        col_data.size = vec_col_sizes[i];
        vec_col_data.push_back( col_data );
    }
    std::vector<std::unique_ptr<loco::TCollision>> vec_colliders;
    std::vector<std::unique_ptr<loco::mujoco::TMujocoCollisionAdapter>> vec_colliders_adapters;
    for ( size_t i = 0; i < vec_col_data.size(); i++ )
    {
        const auto collider_name = loco::mujoco::enumShape_to_mjcShape( vec_col_data[i].type ) + "_collider";
        auto col_obj = std::make_unique<loco::TCollision>( collider_name, vec_col_data[i] );
        auto col_adapter = std::make_unique<loco::mujoco::TMujocoCollisionAdapter>( col_obj.get() );
        col_adapter->Build();
        EXPECT_TRUE( col_adapter->element_resources() );
        vec_colliders.push_back( std::move( col_obj ) );
        vec_colliders_adapters.push_back( std::move( col_adapter ) );
    }

    for ( size_t i = 0; i < vec_colliders_adapters.size(); i++ )
        LOCO_CORE_TRACE( "mjcf-xml-collider:\n{0}", vec_colliders_adapters[i]->element_resources()->ToString() );
}