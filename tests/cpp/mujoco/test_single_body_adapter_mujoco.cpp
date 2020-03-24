
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_mujoco.h>
#include <primitives/loco_single_body_adapter_mujoco.h>

TEST( TestLocoMujocoSingleBodyAdapter, TestLocoMujocoSingleBodyAdapterBuild )
{
    loco::TLogger::Init();

    auto plane_col_data = loco::TCollisionData();
    auto plane_vis_data = loco::TVisualData();
    plane_col_data.type = loco::eShapeType::BOX;
    plane_vis_data.type = loco::eShapeType::BOX;
    plane_col_data.size = { 0.1f, 0.2f, 0.3f };
    plane_vis_data.size = { 0.1f, 0.2f, 0.3f };
    auto plane_body_data = loco::TBodyData();
    plane_body_data.collision = plane_col_data;
    plane_body_data.visual = plane_vis_data;

    const std::string plane_body_name = "floor";
    auto plane_body_obj = std::make_unique<loco::TSingleBody>( plane_body_name, plane_body_data, loco::TVec3(), loco::TMat3() );
    {
        auto plane_body_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyAdapter>( plane_body_obj.get() );
        plane_body_adapter->Build();
        const std::string expected_name = "floor";
        const std::string expected_jnt_name = "floor_freejnt";
        const loco::TVec3 expected_pos = { 0.0f, 0.0f, 0.0f };
        const loco::TVec4 expected_quaternion = { 1.0f, 0.0f, 0.0f, 0.0f }; // w-x-y-z

        auto mjcf_resources = plane_body_adapter->element_resources();
        ASSERT_TRUE( mjcf_resources != nullptr );
        EXPECT_TRUE( mjcf_resources->HasAttributeString( "name" ) );
        EXPECT_EQ( mjcf_resources->GetString( "name" ), expected_name );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec3( "pos" ), expected_pos ) );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec4( "quat" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec4( "quat" ), expected_quaternion ) );
        EXPECT_FALSE( mjcf_resources->HasChildOfType( "freejoint" ) );

        LOCO_TRACE( "mjcf-xml-body: \n{0}", mjcf_resources->ToString() );
    }

    auto box_col_data = loco::TCollisionData();
    auto box_vis_data = loco::TVisualData();
    box_col_data.type = loco::eShapeType::BOX;
    box_vis_data.type = loco::eShapeType::BOX;
    box_col_data.size = { 0.1f, 0.2f, 0.3f };
    box_vis_data.size = { 0.1f, 0.2f, 0.3f };
    auto box_body_data = loco::TBodyData();
    box_body_data.collision = box_col_data;
    box_body_data.visual = box_vis_data;
    box_body_data.dyntype = loco::eDynamicsType::DYNAMIC;

    const std::string box_body_name = "boxy";
    const loco::TVec3 box_body_position = { 1.0f, 2.0f, 3.0f };
    const loco::TVec4 box_body_quaternion = { 0.146f, 0.354f, 0.354f, 0.854f };
    auto box_body_obj = std::make_unique<loco::TSingleBody>( box_body_name, box_body_data, box_body_position, tinymath::rotation( box_body_quaternion ) );
    {
        auto box_body_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyAdapter>( box_body_obj.get() );
        box_body_adapter->Build();
        const std::string expected_name = "boxy";
        const std::string expected_jnt_name = "boxy_freejnt";
        const loco::TVec3 expected_pos = box_body_position;
        const loco::TVec4 expected_quat = { box_body_quaternion.w(), box_body_quaternion.x(),
                                            box_body_quaternion.y(), box_body_quaternion.z() };

        auto mjcf_resources = box_body_adapter->element_resources();
        ASSERT_TRUE( mjcf_resources != nullptr );
        EXPECT_TRUE( mjcf_resources->HasAttributeString( "name" ) );
        EXPECT_EQ( mjcf_resources->GetString( "name" ), expected_name );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec3( "pos" ), expected_pos ) );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec4( "quat" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec4( "quat" ), expected_quat, 1e-2f ) );
        ASSERT_TRUE( mjcf_resources->HasChildOfType( "freejoint" ) );
        auto joint_mjcf_resource = mjcf_resources->GetFirstChildOfType( "freejoint" );
        EXPECT_TRUE( joint_mjcf_resource->HasAttributeString( "name" ) );
        EXPECT_EQ( joint_mjcf_resource->GetString( "name" ), expected_jnt_name );

        LOCO_TRACE( "mjcf-xml-body: \n{0}", mjcf_resources->ToString() );
    }

    auto mesh_col_data = loco::TCollisionData();
    auto mesh_vis_data = loco::TVisualData();
    mesh_col_data.type = loco::eShapeType::MESH;
    mesh_vis_data.type = loco::eShapeType::MESH;
    mesh_col_data.size = { 0.1f, 0.2f, 0.3f };
    mesh_vis_data.size = { 0.1f, 0.2f, 0.3f };
    mesh_col_data.mesh_data.filename = loco::PATH_RESOURCES + "meshes/monkey.stl";
    mesh_vis_data.mesh_data.filename = loco::PATH_RESOURCES + "meshes/monkey.stl";
    auto mesh_body_data = loco::TBodyData();
    mesh_body_data.collision = mesh_col_data;
    mesh_body_data.visual = mesh_vis_data;
    mesh_body_data.dyntype = loco::eDynamicsType::DYNAMIC;
    mesh_body_data.inertia.mass = 1.0f;
    mesh_body_data.inertia.ixx = 0.21f;
    mesh_body_data.inertia.iyy = 0.31f;
    mesh_body_data.inertia.izz = 0.41f;
    mesh_body_data.inertia.ixy = 0.01f;
    mesh_body_data.inertia.ixz = 0.02f;
    mesh_body_data.inertia.iyz = 0.03f;

    const std::string mesh_body_name = "monkey_head";
    const loco::TVec3 mesh_body_position = { -1.0f, -2.0f, 3.0f };
    const loco::TVec4 mesh_body_quaternion = { 0.0f, 0.0f, 0.0f, 1.0f };
    auto mesh_body_obj = std::make_unique<loco::TSingleBody>( mesh_body_name, mesh_body_data, mesh_body_position, tinymath::rotation( mesh_body_quaternion ) );
    {
        auto mesh_body_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyAdapter>( mesh_body_obj.get() );
        mesh_body_adapter->Build();
        const std::string expected_name = "monkey_head";
        const std::string expected_jnt_name = "monkey_head_freejnt";
        const loco::TVec3 expected_pos = { -1.0f, -2.0f, 3.0f };
        const loco::TVec4 expected_quat = { 1.0f, 0.0f, 0.0f, 0.0f };
        const float expected_mass = 1.0f;

        auto mjcf_resources = mesh_body_adapter->element_resources();
        ASSERT_TRUE( mjcf_resources != nullptr );
        EXPECT_TRUE( mjcf_resources->HasAttributeString( "name" ) );
        EXPECT_EQ( mjcf_resources->GetString( "name" ), expected_name );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec3( "pos" ), expected_pos ) );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec4( "quat" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec4( "quat" ), expected_quat ) );
        ASSERT_TRUE( mjcf_resources->HasChildOfType( "freejoint" ) );
        auto joint_mjcf_resource = mjcf_resources->GetFirstChildOfType( "freejoint" );
        EXPECT_TRUE( joint_mjcf_resource->HasAttributeString( "name" ) );
        EXPECT_EQ( joint_mjcf_resource->GetString( "name" ), expected_jnt_name );
        ASSERT_TRUE( mjcf_resources->HasChildOfType( "inertial" ) );
        auto inertial_mjcf_resource = mjcf_resources->GetFirstChildOfType( "inertial" );
        EXPECT_TRUE( inertial_mjcf_resource->HasAttributeFloat( "mass" ) );
        EXPECT_TRUE( std::abs( inertial_mjcf_resource->GetFloat( "mass" ) - expected_mass ) < 1e-5 );
        EXPECT_TRUE( inertial_mjcf_resource->HasAttributeArrayFloat( "fullinertia" ) );
        auto inertia_arr = inertial_mjcf_resource->GetArrayFloat( "fullinertia" );
        ASSERT_EQ( inertia_arr.ndim, 6 );
        EXPECT_TRUE( std::abs( inertia_arr[0] - 0.21f ) < 1e-5 );
        EXPECT_TRUE( std::abs( inertia_arr[1] - 0.31f ) < 1e-5 );
        EXPECT_TRUE( std::abs( inertia_arr[2] - 0.41f ) < 1e-5 );
        EXPECT_TRUE( std::abs( inertia_arr[3] - 0.01f ) < 1e-5 );
        EXPECT_TRUE( std::abs( inertia_arr[4] - 0.02f ) < 1e-5 );
        EXPECT_TRUE( std::abs( inertia_arr[5] - 0.03f ) < 1e-5 );

        LOCO_TRACE( "mjcf-xml-body: \n{0}", mjcf_resources->ToString() );
    }

    auto sphere_col_data = loco::TCollisionData();
    auto sphere_vis_data = loco::TVisualData();
    sphere_col_data.type = loco::eShapeType::SPHERE;
    sphere_vis_data.type = loco::eShapeType::SPHERE;
    sphere_col_data.size = { 0.1f, 0.1f, 0.1f };
    sphere_vis_data.size = { 0.1f, 0.1f, 0.1f };
    auto sphere_body_data = loco::TBodyData();
    sphere_body_data.collision = sphere_col_data;
    sphere_body_data.visual = sphere_vis_data;
    sphere_body_data.dyntype = loco::eDynamicsType::DYNAMIC;
    sphere_body_data.inertia.mass = 10.0f;

    const std::string sphere_body_name = "heavy_sphere";
    const loco::TVec3 sphere_body_position = { 0.0f, 0.0f, 3.0f };
    const loco::TVec4 sphere_body_quaternion = { 0.0f, 0.0f, 0.0f, 1.0f };
    auto sphere_body_obj = std::make_unique<loco::TSingleBody>( sphere_body_name, sphere_body_data, sphere_body_position, tinymath::rotation( sphere_body_quaternion ) );
    {
        auto sphere_body_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyAdapter>( sphere_body_obj.get() );
        sphere_body_adapter->Build(); // will call col's adapter Build method
        auto sphere_col_ref = sphere_body_obj->collider();
        auto sphere_col_adapter = static_cast<loco::mujoco::TMujocoCollisionAdapter*>( sphere_col_ref->collider_adapter() );
        const std::string expected_name = "heavy_sphere";
        const std::string expected_jnt_name = "heavy_sphere_freejnt";
        const loco::TVec3 expected_pos = { 0.0f, 0.0f, 3.0f };
        const loco::TVec4 expected_quat = { 1.0f, 0.0f, 0.0f, 0.0f };
        const float expected_density = 10.0f / ( (4.0f / 3.0f) * loco::PI * 0.1f * 0.1f * 0.1f );

        auto mjcf_resources = sphere_body_adapter->element_resources();
        ASSERT_TRUE( mjcf_resources != nullptr );
        EXPECT_TRUE( mjcf_resources->HasAttributeString( "name" ) );
        EXPECT_EQ( mjcf_resources->GetString( "name" ), expected_name );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec3( "pos" ), expected_pos ) );
        EXPECT_TRUE( mjcf_resources->HasAttributeVec4( "quat" ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec4( "quat" ), expected_quat ) );
        ASSERT_TRUE( mjcf_resources->HasChildOfType( "freejoint" ) );
        auto joint_mjcf_resource = mjcf_resources->GetFirstChildOfType( "freejoint" );
        EXPECT_TRUE( joint_mjcf_resource->HasAttributeString( "name" ) );
        EXPECT_EQ( joint_mjcf_resource->GetString( "name" ), expected_jnt_name );
        ASSERT_FALSE( mjcf_resources->HasChildOfType( "inertial" ) );

        auto mjcf_col_resources = sphere_col_adapter->element_resources();
        ASSERT_TRUE( mjcf_col_resources != nullptr );
        EXPECT_TRUE( mjcf_col_resources->HasAttributeFloat( "density" ) );
        EXPECT_TRUE( std::abs( mjcf_col_resources->GetFloat( "density" ) - expected_density ) < 1e-5 );

        LOCO_TRACE( "mjcf-xml-body: \n{0}", mjcf_resources->ToString() );
    }

    auto scenario = std::make_unique<loco::TScenario>();
    scenario->AddSingleBody( std::move( plane_body_obj ) );
    scenario->AddSingleBody( std::move( box_body_obj ) );
    scenario->AddSingleBody( std::move( mesh_body_obj ) );
    scenario->AddSingleBody( std::move( sphere_body_obj ) );

    auto simulation = std::make_unique<loco::mujoco::TMujocoSimulation>( scenario.get() );
    simulation->Initialize();
}

TEST( TestLocoMujocoSingleBodyAdapter, TestLocoMujocoSingleBodyAdapterInitialize )
{
    loco::TLogger::Init();

    auto scenario = std::make_unique<loco::TScenario>();

    std::vector<loco::eShapeType> vec_shape_types = { loco::eShapeType::BOX,
                                                      loco::eShapeType::SPHERE,
                                                      loco::eShapeType::PLANE,
                                                      loco::eShapeType::CYLINDER,
                                                      loco::eShapeType::CAPSULE,
                                                      loco::eShapeType::ELLIPSOID };
    std::vector<loco::eDynamicsType> vec_dyntypes = { loco::eDynamicsType::STATIC,
                                                      loco::eDynamicsType::DYNAMIC,
                                                      loco::eDynamicsType::STATIC,
                                                      loco::eDynamicsType::DYNAMIC,
                                                      loco::eDynamicsType::STATIC,
                                                      loco::eDynamicsType::DYNAMIC };
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
        body_data.dyntype = vec_dyntypes[i];

        const auto body_name = loco::mujoco::enumShape_to_mjcShape( vec_shape_types[i] ) + "_body";
        auto body_obj = std::make_unique<loco::TSingleBody>( body_name, body_data, vec_positions[i], loco::TMat3() );

        scenario->AddSingleBody( std::move( body_obj ) );
    }

    auto simulation = std::make_unique<loco::mujoco::TMujocoSimulation>( scenario.get() );
    simulation->Initialize();

    auto single_bodies_list = scenario->GetSingleBodiesList();
    for ( size_t i = 0; i < single_bodies_list.size(); i++ )
    {
        auto single_body = single_bodies_list[i];
        auto single_body_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyAdapter*>( single_body->adapter() );
        ASSERT_TRUE( single_body_adapter != nullptr );
        EXPECT_TRUE( single_body_adapter->mjc_model() != nullptr );
        EXPECT_TRUE( single_body_adapter->mjc_data() != nullptr );
        EXPECT_NE( single_body_adapter->mjc_body_id(), -1 );
        EXPECT_EQ( single_body->dyntype(), vec_dyntypes[i] );

        if ( single_body->dyntype() != loco::eDynamicsType::DYNAMIC )
            continue;

        const ssize_t num_qpos_freejoint = 7;
        const ssize_t num_qvel_freejoint = 6;

        EXPECT_NE( single_body_adapter->mjc_joint_id(), -1 );
        EXPECT_NE( single_body_adapter->mjc_joint_qpos_adr(), -1 );
        EXPECT_NE( single_body_adapter->mjc_joint_qvel_adr(), -1 );
        EXPECT_EQ( single_body_adapter->mjc_joint_qpos_num(), num_qpos_freejoint );
        EXPECT_EQ( single_body_adapter->mjc_joint_qvel_num(), num_qvel_freejoint );
    }
}