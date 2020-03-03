
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_mujoco.h>
#include <adapters/loco_collision_adapter_mujoco.h>

bool allclose_sf( const loco::TSizef& arr_1, const loco::TSizef& arr_2 )
{
    if ( arr_1.ndim != arr_2.ndim )
        return false;

    for ( size_t i = 0; i < arr_1.ndim; i++ )
        if ( std::abs( arr_1[i] - arr_2[i] ) > 1e-5 )
            return false;
    return true;
}

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

    std::vector<loco::TSizef> vec_expected_sizes = { { 0.05f, 0.1f, 0.15f },
                                                     { 0.1f },
                                                     { 5.0f, 5.0f, 1.0f },
                                                     { 0.2f, 0.4f },
                                                     { 0.2f, 0.4f },
                                                     { 0.2f, 0.3f, 0.4f } };
    std::vector<std::string> vec_expected_names = { "box_collider",
                                                    "sphere_collider",
                                                    "plane_collider",
                                                    "cylinder_collider",
                                                    "capsule_collider",
                                                    "ellipsoid_collider" };
    std::vector<std::string> vec_expected_types = { "box", "sphere", "plane", "cylinder", "capsule", "ellipsoid" };
    for ( size_t i = 0; i < vec_colliders_adapters.size(); i++ )
    {
        auto mjc_col_adapter = dynamic_cast<loco::mujoco::TMujocoCollisionAdapter*>( vec_colliders_adapters[i].get() );
        EXPECT_TRUE( mjc_col_adapter != nullptr );
        auto mjcf_resources = mjc_col_adapter->element_resources();
        EXPECT_TRUE( mjcf_resources != nullptr );
        EXPECT_TRUE( mjcf_resources->HasAttributeArrayFloat( "size" ) );
        EXPECT_TRUE( allclose_sf( mjcf_resources->GetArrayFloat( "size" ), vec_expected_sizes[i] ) );
        EXPECT_TRUE( mjcf_resources->HasAttributeString( "name" ) );
        EXPECT_EQ( mjcf_resources->GetString( "name" ), vec_expected_names[i] );
        EXPECT_TRUE( mjcf_resources->HasAttributeString( "type" ) );
        EXPECT_EQ( mjcf_resources->GetString( "type" ), vec_expected_types[i] );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec4( "quat" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec4( "quat" ), { 1.0f, 0.0f, 0.0f, 0.0f } ) );
    }
}