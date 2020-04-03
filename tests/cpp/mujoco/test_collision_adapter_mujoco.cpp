
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_mujoco.h>
#include <primitives/loco_single_body_collider_adapter_mujoco.h>
//// #include <sys/stat.h>

bool allclose_sf( const loco::TSizef& arr_1, const loco::TSizef& arr_2 )
{
    if ( arr_1.ndim != arr_2.ndim )
        return false;

    for ( size_t i = 0; i < arr_1.ndim; i++ )
        if ( std::abs( arr_1[i] - arr_2[i] ) > 1e-5 )
            return false;
    return true;
}

std::vector<float> create_hfield( size_t nWidthSamples, size_t nDepthSamples )
{
    std::vector<float> hfield( nWidthSamples * nDepthSamples );
    for ( size_t i = 0; i < nDepthSamples; i++ )
    {
        for ( size_t j = 0; j < nWidthSamples; j++ )
        {
            const float x = (float)j / ( nWidthSamples - 1 ) - 0.5f;
            const float y = (float)i / ( nDepthSamples - 1 ) - 0.5f;
            hfield[i * nWidthSamples + j] = 2.5f * ( x * x + y * y );
        }
    }
    return hfield;
}

std::pair<std::vector<float>, std::vector<int>> create_mesh_tetrahedron()
{
    std::vector<float> vertices = { 0.0f, 0.0f, 0.0f,
                                    1.0f, 0.0f, 0.0f,
                                    0.0f, 1.0f, 0.0f,
                                    0.0f, 0.0f, 1.0f };
    std::vector<int> faces = { 0, 2, 1,
                               0, 1, 3,
                               1, 2, 3,
                               0, 3, 2 };
    return { vertices, faces };
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
    std::vector<std::unique_ptr<loco::TSingleBodyCollider>> vec_colliders;
    std::vector<std::unique_ptr<loco::mujoco::TMujocoSingleBodyColliderAdapter>> vec_colliders_adapters;
    for ( size_t i = 0; i < vec_col_data.size(); i++ )
    {
        const auto collider_name = loco::mujoco::enumShape_to_mjcShape( vec_col_data[i].type ) + "_collider";
        auto col_obj = std::make_unique<loco::TSingleBodyCollider>( collider_name, vec_col_data[i] );
        auto col_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyColliderAdapter>( col_obj.get() );
        col_adapter->Build();
        ASSERT_TRUE( col_adapter->element_resources() != nullptr );
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
        auto mjcf_resources = vec_colliders_adapters[i]->element_resources();
        ASSERT_TRUE( mjcf_resources != nullptr );
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

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterInitialize )
{
    loco::TLogger::Init();

    auto scenario = std::make_unique<loco::TScenario>();

    std::vector<loco::eShapeType> vec_shape_types = { loco::eShapeType::BOX,
                                                      loco::eShapeType::SPHERE,
                                                      loco::eShapeType::PLANE,
                                                      loco::eShapeType::CYLINDER,
                                                      loco::eShapeType::CYLINDER,
                                                      loco::eShapeType::CAPSULE,
                                                      loco::eShapeType::CAPSULE,
                                                      loco::eShapeType::ELLIPSOID,
                                                      loco::eShapeType::ELLIPSOID,
                                                      loco::eShapeType::ELLIPSOID };
    std::vector<loco::TVec3> vec_shape_sizes = { { 0.1, 0.2, 0.3 },
                                                 { 0.1, 0.1, 0.1 },
                                                 { 10.0, 10.0, 1.0 },
                                                 { 0.2, 0.8, 0.2 },
                                                 { 0.8, 0.2, 0.8 },
                                                 { 0.2, 0.8, 0.2 },
                                                 { 0.8, 0.2, 0.8 },
                                                 { 0.2, 0.3, 0.4 },
                                                 { 0.4, 0.2, 0.3 },
                                                 { 0.3, 0.4, 0.2 } };
    std::vector<loco::TVec3> vec_positions = { { -1.0, -1.0, 1.0 }, // box
                                               { -1.0,  0.0, 1.0 }, // sphere
                                               {  0.0,  0.0, 0.0 }, // plane
                                               {  1.0,  1.0, 1.0 }, // cylinder
                                               {  1.0,  1.0, 2.0 }, // cylinder
                                               {  1.0,  0.0, 1.0 }, // capsule
                                               {  1.0,  0.0, 2.0 }, // capsule
                                               {  0.0,  1.0, 1.0 }, // ellipsoid
                                               {  0.0,  1.0, 2.0 }, // ellipsoid
                                               {  0.0,  1.0, 3.0 } }; // ellipsoid
    std::vector<mjtGeom> vec_expected_mjc_types = { mjGEOM_BOX,
                                                    mjGEOM_SPHERE,
                                                    mjGEOM_PLANE,
                                                    mjGEOM_CYLINDER,
                                                    mjGEOM_CYLINDER,
                                                    mjGEOM_CAPSULE,
                                                    mjGEOM_CAPSULE,
                                                    mjGEOM_ELLIPSOID,
                                                    mjGEOM_ELLIPSOID,
                                                    mjGEOM_ELLIPSOID };
    std::vector<loco::TSizef> vec_expected_mjc_sizes = { { 0.05, 0.1, 0.15 },
                                                         { 0.1 },
                                                         { 5.0, 5.0, 1.0 },
                                                         { 0.2, 0.4 },
                                                         { 0.8, 0.1 },
                                                         { 0.2, 0.4 },
                                                         { 0.8, 0.1 },
                                                         { 0.2, 0.3, 0.4 },
                                                         { 0.4, 0.2, 0.3 },
                                                         { 0.3, 0.4, 0.2 } };
    ASSERT_EQ( vec_shape_types.size(), vec_shape_sizes.size() );
    ASSERT_EQ( vec_shape_sizes.size(), vec_positions.size() );
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

        const auto body_name = loco::mujoco::enumShape_to_mjcShape( vec_shape_types[i] ) + "_body_" + std::to_string( i );
        auto body_obj = std::make_unique<loco::TSingleBody>( body_name, body_data, vec_positions[i], loco::TMat3() );

        scenario->AddSingleBody( std::move( body_obj ) );
    }

    auto simulation = std::make_unique<loco::mujoco::TMujocoSimulation>( scenario.get() );
    simulation->Initialize();

    auto mjc_model = simulation->mjc_model();
    auto mjc_data = simulation->mjc_data();

    auto single_bodies_list = scenario->GetSingleBodiesList();
    for ( size_t i = 0; i < single_bodies_list.size(); i++ )
    {
        auto collider = single_bodies_list[i]->collider();
        ASSERT_TRUE( collider != nullptr );
        auto mjc_col_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyColliderAdapter*>( collider->collider_adapter() );
        ASSERT_TRUE( mjc_col_adapter != nullptr );

        const ssize_t mjc_geom_id = mjc_col_adapter->mjc_geom_id();
        const double mjc_geom_rbound = mjc_col_adapter->mjc_geom_radius_bound();
        const double expected_rbound = loco::mujoco::compute_primitive_rbound( vec_shape_types[i], vec_shape_sizes[i] );

        ASSERT_TRUE( mjc_geom_id != -1 );
        EXPECT_TRUE( std::abs( mjc_geom_rbound - expected_rbound ) < 1e-5 );

        EXPECT_EQ( mjc_model->geom_type[mjc_geom_id], vec_expected_mjc_types[i] );
        for ( size_t j = 0; j < vec_expected_mjc_sizes[i].ndim; j++ )
            EXPECT_TRUE( std::abs( mjc_model->geom_size[3 * mjc_geom_id + j] - vec_expected_mjc_sizes[i][j] ) < 1e-5 );
        // @todo: enable when collision-groups are enabled
        //// EXPECT_EQ( mjc_model->geom_contype[mjc_geom_id], collider->collisionGroup() );
        //// EXPECT_EQ( mjc_model->geom_conaffinity[mjc_geom_id], collider->collisionMask() );
    }
}

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterMeshBuild )
{
    loco::TLogger::Init();

    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::MESH;
    col_data.size = { 0.2f, 0.2f, 0.2f };
    col_data.mesh_data.filename = loco::PATH_RESOURCES + "meshes/monkey.stl";

    const auto collider_name = loco::mujoco::enumShape_to_mjcShape( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TSingleBodyCollider>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyColliderAdapter>( col_obj.get() );
    col_adapter->Build();

    auto mjcf_resources = col_adapter->element_resources();
    auto mjcf_asset_resources = col_adapter->element_asset_resources();
    ASSERT_TRUE( mjcf_resources != nullptr );
    ASSERT_TRUE( mjcf_asset_resources != nullptr );
    LOCO_CORE_TRACE( "mjcf-xml-collider:\n{0}", mjcf_resources->ToString() );
    LOCO_CORE_TRACE( "mjcf-xml-collider:\n{0}", mjcf_asset_resources->ToString() );

    EXPECT_FALSE( mjcf_resources->HasAttributeArrayFloat( "size" ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "name" ) );
    EXPECT_EQ( mjcf_resources->GetString( "name" ), "mesh_collider" );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "type" ) );
    EXPECT_EQ( mjcf_resources->GetString( "type" ), "mesh" );
    EXPECT_TRUE( mjcf_resources->HasAttributeVec3( "pos" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeVec4( "quat" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec4( "quat" ), { 1.0f, 0.0f, 0.0f, 0.0f } ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "mesh" ) );
    EXPECT_EQ( mjcf_resources->GetString( "mesh" ), "monkey" );

    EXPECT_TRUE( mjcf_asset_resources->HasAttributeString( "name" ) );
    EXPECT_EQ( mjcf_asset_resources->GetString( "name" ), "monkey" );
    EXPECT_TRUE( mjcf_asset_resources->HasAttributeString( "file" ) );
    EXPECT_EQ( mjcf_asset_resources->GetString( "file" ), loco::PATH_RESOURCES + "meshes/monkey.stl" );
    EXPECT_TRUE( mjcf_asset_resources->HasAttributeVec3( "scale" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_asset_resources->GetVec3( "scale" ), { 0.2f, 0.2f, 0.2f } ) );
}

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterMeshUserBuild )
{
    loco::TLogger::Init();

    auto vertices_faces = create_mesh_tetrahedron();

    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::MESH;
    col_data.size = { 0.2f, 0.2f, 0.2f };
    col_data.mesh_data.vertices = vertices_faces.first;
    col_data.mesh_data.faces = vertices_faces.second;

    const auto collider_name = loco::mujoco::enumShape_to_mjcShape( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TSingleBodyCollider>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyColliderAdapter>( col_obj.get() );
    col_adapter->Build();

    auto mjcf_resources = col_adapter->element_resources();
    auto mjcf_asset_resources = col_adapter->element_asset_resources();
    ASSERT_TRUE( mjcf_resources != nullptr );
    ASSERT_TRUE( mjcf_asset_resources != nullptr );
    LOCO_CORE_TRACE( "mjcf-xml-collider:\n{0}", mjcf_resources->ToString() );
    LOCO_CORE_TRACE( "mjcf-xml-collider:\n{0}", mjcf_asset_resources->ToString() );

    EXPECT_FALSE( mjcf_resources->HasAttributeArrayFloat( "size" ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "name" ) );
    EXPECT_EQ( mjcf_resources->GetString( "name" ), "mesh_collider" );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "type" ) );
    EXPECT_EQ( mjcf_resources->GetString( "type" ), "mesh" );
    EXPECT_TRUE( mjcf_resources->HasAttributeVec3( "pos" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeVec4( "quat" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec4( "quat" ), { 1.0f, 0.0f, 0.0f, 0.0f } ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "mesh" ) );
    EXPECT_EQ( mjcf_resources->GetString( "mesh" ), "mesh_collider_asset" );

    EXPECT_TRUE( mjcf_asset_resources->HasAttributeString( "name" ) );
    EXPECT_EQ( mjcf_asset_resources->GetString( "name" ), "mesh_collider_asset" );
    EXPECT_TRUE( mjcf_asset_resources->HasAttributeString( "file" ) );
    EXPECT_EQ( mjcf_asset_resources->GetString( "file" ), "mesh_collider.msh" );
    EXPECT_TRUE( mjcf_asset_resources->HasAttributeVec3( "scale" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_asset_resources->GetVec3( "scale" ), { 0.2f, 0.2f, 0.2f } ) );

    // @todo: enable only for linux platforms
    //// struct stat results;
    //// if ( stat( "mesh_collider.msh", &results ) == 0 )
    ////     LOCO_CORE_INFO( "mesh-file size in bytes : {0}", results.st_size );
    //// else
    ////     LOCO_CORE_ERROR( "Couldn't get file-size of mesh file" );
    //// const size_t expected_tetrahedron_data_nbytes = 112;
    //// EXPECT_EQ( results.st_size, expected_tetrahedron_data_nbytes );
}

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterMeshInitialize )
{
    loco::TLogger::Init();

    auto scenario = std::make_unique<loco::TScenario>();

    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::MESH;
    col_data.size = { 0.2f, 0.2f, 0.2f };
    col_data.mesh_data.filename = loco::PATH_RESOURCES + "meshes/monkey.stl";
    auto vis_data = loco::TVisualData();
    vis_data.type = loco::eShapeType::MESH;
    vis_data.size = { 0.2f, 0.2f, 0.2f };
    vis_data.mesh_data.filename = loco::PATH_RESOURCES + "meshes/monkey.stl";
    auto body_data = loco::TBodyData();
    body_data.collision = col_data;
    body_data.visual = vis_data;

    // Create duplicate meshes to check if no duplicate assets are added
    const std::string body_name_1 = "mesh_body_1";
    const std::string body_name_2 = "mesh_body_2";
    auto body_obj_1 = std::make_unique<loco::TSingleBody>( body_name_1, body_data, loco::TVec3(), loco::TMat3() );
    auto body_obj_2 = std::make_unique<loco::TSingleBody>( body_name_2, body_data, loco::TVec3(), loco::TMat3() );

    scenario->AddSingleBody( std::move( body_obj_1 ) );
    scenario->AddSingleBody( std::move( body_obj_2 ) );

    auto simulation = std::make_unique<loco::mujoco::TMujocoSimulation>( scenario.get() );
    simulation->Initialize();

    auto mjcf_simulation = simulation->mjcf_element();
    ASSERT_TRUE( mjcf_simulation != nullptr );
    auto mjcf_assets = mjcf_simulation->GetFirstChildOfType( loco::mujoco::LOCO_MJCF_ASSET_TAG );
    ASSERT_TRUE( mjcf_assets != nullptr );
    ASSERT_EQ( mjcf_assets->num_children(), 1 ); // Only one mesh-asset should be here (no duplicates)

    auto mjc_model = simulation->mjc_model();
    auto mjc_data = simulation->mjc_data();

    auto mesh_body = scenario->GetSingleBodyByName( body_name_1 );
    ASSERT_TRUE( mesh_body != nullptr );
    auto mesh_collider = mesh_body->collider();
    ASSERT_TRUE( mesh_collider != nullptr );
    auto mjc_col_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyColliderAdapter*>( mesh_collider->collider_adapter() );
    ASSERT_TRUE( mjc_col_adapter != nullptr );

    const ssize_t mjc_geom_id = mjc_col_adapter->mjc_geom_id();
    const ssize_t mjc_geom_mesh_id = mjc_col_adapter->mjc_geom_mesh_id();
    ASSERT_TRUE( mjc_geom_id != -1 );
    ASSERT_TRUE( mjc_geom_mesh_id != -1 );

    EXPECT_EQ( mjc_model->geom_type[mjc_geom_id], mjGEOM_MESH );
    EXPECT_EQ( mjc_model->geom_contype[mjc_geom_id], mesh_collider->collisionGroup() );
    EXPECT_EQ( mjc_model->geom_conaffinity[mjc_geom_id], mesh_collider->collisionMask() );
}

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterMeshUserInitialize )
{
    loco::TLogger::Init();

    auto scenario = std::make_unique<loco::TScenario>();

    auto vertices_faces = create_mesh_tetrahedron();

    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::MESH;
    col_data.size = { 0.2f, 0.2f, 0.2f };
    col_data.mesh_data.vertices = vertices_faces.first;
    col_data.mesh_data.faces = vertices_faces.second;
    auto vis_data = loco::TVisualData();
    vis_data.type = loco::eShapeType::MESH;
    vis_data.size = { 0.2f, 0.2f, 0.2f };
    vis_data.mesh_data.vertices = vertices_faces.first;
    vis_data.mesh_data.faces = vertices_faces.second;
    auto body_data = loco::TBodyData();
    body_data.collision = col_data;
    body_data.visual = vis_data;

    const auto body_name = "mesh_body";
    auto body_obj = std::make_unique<loco::TSingleBody>( body_name, body_data, loco::TVec3(), loco::TMat3() );

    scenario->AddSingleBody( std::move( body_obj ) );

    auto simulation = std::make_unique<loco::mujoco::TMujocoSimulation>( scenario.get() );
    simulation->Initialize();

    auto mjc_model = simulation->mjc_model();
    auto mjc_data = simulation->mjc_data();

    auto mesh_body = scenario->GetSingleBodyByName( body_name );
    ASSERT_TRUE( mesh_body != nullptr );
    auto mesh_collider = mesh_body->collider();
    ASSERT_TRUE( mesh_collider != nullptr );
    auto mjc_col_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyColliderAdapter*>( mesh_collider->collider_adapter() );
    ASSERT_TRUE( mjc_col_adapter != nullptr );

    const ssize_t mjc_geom_id = mjc_col_adapter->mjc_geom_id();
    const ssize_t mjc_geom_mesh_id = mjc_col_adapter->mjc_geom_mesh_id();
    ASSERT_TRUE( mjc_geom_id != -1 );
    ASSERT_TRUE( mjc_geom_mesh_id != -1 );

    const ssize_t expected_tetrahedron_num_vertices = 4;
    const ssize_t expected_tetrahedron_num_faces = 4;
    EXPECT_EQ( mjc_model->geom_type[mjc_geom_id], mjGEOM_MESH );
    EXPECT_EQ( mjc_model->mesh_vertnum[mjc_geom_mesh_id], expected_tetrahedron_num_vertices );
    EXPECT_EQ( mjc_model->mesh_facenum[mjc_geom_mesh_id], expected_tetrahedron_num_faces );
    // @todo: enable when collision-groups are enabled
    //// EXPECT_EQ( mjc_model->geom_contype[mjc_geom_id], collider->collisionGroup() );
    //// EXPECT_EQ( mjc_model->geom_conaffinity[mjc_geom_id], collider->collisionMask() );
}

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterHfieldBuild )
{
    loco::TLogger::Init();

    const size_t num_width_samples = 40;
    const size_t num_depth_samples = 40;
    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::HFIELD;
    col_data.size = { 10.0f, 10.0f, 2.0f }; // width, depth, scale-height
    col_data.hfield_data.nWidthSamples = num_width_samples;
    col_data.hfield_data.nDepthSamples = num_depth_samples;
    col_data.hfield_data.heights = create_hfield( num_width_samples, num_depth_samples );

    const loco::TVec4 expected_size = { 0.5f * 10.0f,
                                        0.5f * 10.0f,
                                        *std::max_element( col_data.hfield_data.heights.begin(),
                                                           col_data.hfield_data.heights.end() ) * 2.0f,
                                        loco::mujoco::LOCO_MUJOCO_HFIELD_BASE };

    const auto collider_name = loco::mujoco::enumShape_to_mjcShape( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TSingleBodyCollider>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyColliderAdapter>( col_obj.get() );
    col_adapter->Build();

    auto mjcf_resources = col_adapter->element_resources();
    auto mjcf_asset_resources = col_adapter->element_asset_resources();
    ASSERT_TRUE( mjcf_resources != nullptr );
    ASSERT_TRUE( mjcf_asset_resources != nullptr );
    LOCO_CORE_TRACE( "mjcf-xml-collider:\n{0}", mjcf_resources->ToString() );
    LOCO_CORE_TRACE( "mjcf-xml-collider:\n{0}", mjcf_asset_resources->ToString() );

    EXPECT_FALSE( mjcf_resources->HasAttributeArrayFloat( "size" ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "name" ) );
    EXPECT_EQ( mjcf_resources->GetString( "name" ), "hfield_collider" );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "type" ) );
    EXPECT_EQ( mjcf_resources->GetString( "type" ), "hfield" );
    EXPECT_TRUE( mjcf_resources->HasAttributeVec3( "pos" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeVec4( "quat" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_resources->GetVec4( "quat" ), { 1.0f, 0.0f, 0.0f, 0.0f } ) );
    EXPECT_TRUE( mjcf_resources->HasAttributeString( "hfield" ) );
    EXPECT_EQ( mjcf_resources->GetString( "hfield" ), "hfield_collider_asset" );

    EXPECT_TRUE( mjcf_asset_resources->HasAttributeString( "name" ) );
    EXPECT_EQ( mjcf_asset_resources->GetString( "name" ), "hfield_collider_asset" );
    EXPECT_TRUE( mjcf_asset_resources->HasAttributeInt( "nrow" ) );
    EXPECT_EQ( mjcf_asset_resources->GetInt( "nrow" ), num_depth_samples );
    EXPECT_TRUE( mjcf_asset_resources->HasAttributeInt( "ncol" ) );
    EXPECT_EQ( mjcf_asset_resources->GetInt( "ncol" ), num_width_samples );
    EXPECT_TRUE( mjcf_asset_resources->HasAttributeVec4( "size" ) );
    EXPECT_TRUE( tinymath::allclose( mjcf_asset_resources->GetVec4( "size" ), expected_size ) );
}

TEST( TestLocoMujocoCollisionAdapter, TestLocoMujocoCollisionAdapterHfieldInitialize )
{
    loco::TLogger::Init();

    auto scenario = std::make_unique<loco::TScenario>();

    const size_t num_width_samples = 40;
    const size_t num_depth_samples = 40;
    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::HFIELD;
    col_data.size = { 10.0f, 20.0f, 2.0f }; // width, depth, scale-height
    col_data.hfield_data.nWidthSamples = num_width_samples;
    col_data.hfield_data.nDepthSamples = num_depth_samples;
    col_data.hfield_data.heights = create_hfield( num_width_samples, num_depth_samples );
    auto vis_data = loco::TVisualData();
    vis_data.type = loco::eShapeType::HFIELD;
    vis_data.size = { 10.0f, 20.0f, 2.0f }; // width, depth, scale-height
    vis_data.hfield_data.nWidthSamples = num_width_samples;
    vis_data.hfield_data.nDepthSamples = num_depth_samples;
    vis_data.hfield_data.heights = create_hfield( num_width_samples, num_depth_samples );
    auto body_data = loco::TBodyData();
    body_data.collision = col_data;
    body_data.visual = vis_data;

    const auto body_name = "hfield_body";
    auto body_obj = std::make_unique<loco::TSingleBody>( body_name, body_data, loco::TVec3(), loco::TMat3() );

    scenario->AddSingleBody( std::move( body_obj ) );

    auto simulation = std::make_unique<loco::mujoco::TMujocoSimulation>( scenario.get() );
    simulation->Initialize();

    auto mjc_model = simulation->mjc_model();
    auto mjc_data = simulation->mjc_data();

    auto hfield_body = scenario->GetSingleBodyByName( body_name );
    ASSERT_TRUE( hfield_body != nullptr );
    auto mesh_collider = hfield_body->collider();
    ASSERT_TRUE( mesh_collider != nullptr );
    auto mjc_col_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyColliderAdapter*>( mesh_collider->collider_adapter() );
    ASSERT_TRUE( mjc_col_adapter != nullptr );

    const ssize_t mjc_geom_id = mjc_col_adapter->mjc_geom_id();
    const ssize_t mjc_geom_hfield_id = mjc_col_adapter->mjc_geom_hfield_id();
    const ssize_t mjc_geom_hfield_start_addr = mjc_col_adapter->mjc_geom_hfield_start_addr();
    ASSERT_TRUE( mjc_geom_id != -1 );
    ASSERT_TRUE( mjc_geom_hfield_id != -1 );
    ASSERT_TRUE( mjc_geom_hfield_start_addr != -1 );

    const float max_height = *std::max_element( col_data.hfield_data.heights.begin(), col_data.hfield_data.heights.end() );

    EXPECT_EQ( mjc_model->geom_type[mjc_geom_id], mjGEOM_HFIELD );
    EXPECT_EQ( mjc_model->hfield_nrow[mjc_geom_hfield_id], num_depth_samples );
    EXPECT_EQ( mjc_model->hfield_ncol[mjc_geom_hfield_id], num_width_samples );
    EXPECT_TRUE( std::abs( mjc_model->hfield_size[4 * mjc_geom_hfield_id + 0] - 5.0f ) < 1e-5 );
    EXPECT_TRUE( std::abs( mjc_model->hfield_size[4 * mjc_geom_hfield_id + 1] - 10.0f ) < 1e-5 );
    EXPECT_TRUE( std::abs( mjc_model->hfield_size[4 * mjc_geom_hfield_id + 2] - max_height * 2.0f ) < 1e-5 );
    EXPECT_TRUE( std::abs( mjc_model->hfield_size[4 * mjc_geom_hfield_id + 3] - loco::mujoco::LOCO_MUJOCO_HFIELD_BASE ) < 1e-5 );
    for ( size_t i = 0; i < num_depth_samples; i++ )
    {
        for ( size_t j = 0; j < num_width_samples; j++ )
        {
            const size_t buff_idx = i * num_width_samples + j;
            const float mujoco_height = mjc_model->hfield_data[mjc_geom_hfield_start_addr + buff_idx];
            const float expected_height = col_data.hfield_data.heights[buff_idx];
            ASSERT_TRUE( std::abs( mujoco_height * max_height - expected_height ) < 1e-6 );
        }
    }
    // @todo: enable when collision-groups are enabled
    //// EXPECT_EQ( mjc_model->geom_contype[mjc_geom_id], collider->collisionGroup() );
    //// EXPECT_EQ( mjc_model->geom_conaffinity[mjc_geom_id], collider->collisionMask() );
}