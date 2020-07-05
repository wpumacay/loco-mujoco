
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_mujoco.h>
#include <primitives/loco_single_body_constraint_adapter_mujoco.h>
#include <primitives/loco_single_body_adapter_mujoco.h>

TEST( TestLocoMujocoConstraintAdapter, TestLocoMujocoConstraintAdapterBuild )
{
    loco::InitUtils();

    auto scenario = std::make_unique<loco::TScenario>();

    // Revolute constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::primitives::TBox>( "pole_0", loco::TVec3( 0.2f, 0.2f, 1.0f ), loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::primitives::TSingleBodyRevoluteConstraint>( "pole_0_rev_const", loco::TMat4( loco::TMat3(), loco::TVec3( 0.0f, 0.0f, 0.5f ) ), loco::TVec3( 1.0f, 0.0f, 0.0f ) );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::REVOLUTE );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::primitives::TMujocoSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();


        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto mjc_constraint_adapter = dynamic_cast<loco::primitives::TMujocoSingleBodyRevoluteConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( mjc_constraint_adapter != nullptr );

        auto mjcf_elements = mjc_constraint_adapter->elements_resources();
        ASSERT_EQ( mjcf_elements.size(), 1 );
        auto mjcf_element = mjcf_elements[0];
        ASSERT_TRUE( mjcf_element != nullptr );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element->GetString( "name" ), "pole_0_rev_const" );
        EXPECT_EQ( mjcf_element->GetString( "type" ), "hinge" );
        EXPECT_EQ( mjcf_element->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element->GetVec3( "pos" ), { 0.0f, 0.0f, 0.5f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element->GetVec3( "axis" ), { 1.0f, 0.0f, 0.0f } ) );
    }

    // Prismatic constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::primitives::TBox>( "platform_0", loco::TVec3( 1.0f, 1.0f, 0.2f ), loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::primitives::TSingleBodyPrismaticConstraint>( "platform_0_prism_const", loco::TMat4(), loco::TVec3( 0.0f, 0.0f, 1.0f ) );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::PRISMATIC );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::primitives::TMujocoSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto mjc_constraint_adapter = dynamic_cast<loco::primitives::TMujocoSingleBodyPrismaticConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( mjc_constraint_adapter != nullptr );

        auto mjcf_elements = mjc_constraint_adapter->elements_resources();
        ASSERT_EQ( mjcf_elements.size(), 1 );
        auto mjcf_element = mjcf_elements[0];
        ASSERT_TRUE( mjcf_element != nullptr );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element->GetString( "name" ), "platform_0_prism_const" );
        EXPECT_EQ( mjcf_element->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element->GetVec3( "axis" ), { 0.0f, 0.0f, 1.0f } ) );
    }

    // Spherical constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TCapsule>( "rod_0", 0.1f, 1.0f, loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodySphericalConstraint>( "rod_0_spherical_const", loco::TMat4( loco::TMat3(), loco::TVec3( 0.0f, 0.0f, 0.5f ) ) );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::SPHERICAL );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto mjc_constraint_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodySphericalConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( mjc_constraint_adapter != nullptr );

        auto mjcf_elements = mjc_constraint_adapter->elements_resources();
        ASSERT_EQ( mjcf_elements.size(), 1 );
        auto mjcf_element = mjcf_elements[0];
        ASSERT_TRUE( mjcf_element != nullptr );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element->HasAttributeVec3( "pos" ) );
        EXPECT_EQ( mjcf_element->GetString( "name" ), "rod_0_spherical_const" );
        EXPECT_EQ( mjcf_element->GetString( "type" ), "ball" );
        EXPECT_EQ( mjcf_element->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element->GetVec3( "pos" ), { 0.0f, 0.0f, 0.5f } ) );
    }

    // Translational3d constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TSphere>( "sphere_0", 0.1f, loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodyTranslational3dConstraint>( "sphere_0_translational3d_const" );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::TRANSLATIONAL3D );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto mjc_constraint_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyTranslational3dConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( mjc_constraint_adapter != nullptr );

        auto mjcf_elements = mjc_constraint_adapter->elements_resources();
        ASSERT_EQ( mjcf_elements.size(), 3 );

        auto mjcf_element_trans_x = mjcf_elements[0];
        ASSERT_TRUE( mjcf_element_trans_x != nullptr );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "name" ), "sphere_0_translational3d_const_trans_x" );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_x->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_x->GetVec3( "axis" ), { 1.0f, 0.0f, 0.0f } ) );

        auto mjcf_element_trans_y = mjcf_elements[1];
        ASSERT_TRUE( mjcf_element_trans_y != nullptr );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_trans_y->GetString( "name" ), "sphere_0_translational3d_const_trans_y" );
        EXPECT_EQ( mjcf_element_trans_y->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element_trans_y->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_y->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_y->GetVec3( "axis" ), { 0.0f, 1.0f, 0.0f } ) );

        auto mjcf_element_trans_z = mjcf_elements[2];
        ASSERT_TRUE( mjcf_element_trans_z != nullptr );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "name" ), "sphere_0_translational3d_const_trans_z" );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_z->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_z->GetVec3( "axis" ), { 0.0f, 0.0f, 1.0f } ) );
    }

    // Universal3d constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TSphere>( "sphere_1", 0.1f, loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodyUniversal3dConstraint>( "sphere_1_universal3d_const" );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::UNIVERSAL3D );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto mjc_constraint_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyUniversal3dConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( mjc_constraint_adapter != nullptr );

        auto mjcf_elements = mjc_constraint_adapter->elements_resources();
        ASSERT_EQ( mjcf_elements.size(), 4 );

        auto mjcf_element_trans_x = mjcf_elements[0];
        ASSERT_TRUE( mjcf_element_trans_x != nullptr );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "name" ), "sphere_1_universal3d_const_trans_x" );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_x->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_x->GetVec3( "axis" ), { 1.0f, 0.0f, 0.0f } ) );

        auto mjcf_element_trans_y = mjcf_elements[1];
        ASSERT_TRUE( mjcf_element_trans_y != nullptr );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_trans_y->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_trans_y->GetString( "name" ), "sphere_1_universal3d_const_trans_y" );
        EXPECT_EQ( mjcf_element_trans_y->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element_trans_y->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_y->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_y->GetVec3( "axis" ), { 0.0f, 1.0f, 0.0f } ) );

        auto mjcf_element_trans_z = mjcf_elements[2];
        ASSERT_TRUE( mjcf_element_trans_z != nullptr );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "name" ), "sphere_1_universal3d_const_trans_z" );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_z->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_z->GetVec3( "axis" ), { 0.0f, 0.0f, 1.0f } ) );

        auto mjcf_element_rot_z = mjcf_elements[3];
        ASSERT_TRUE( mjcf_element_rot_z != nullptr );
        EXPECT_TRUE( mjcf_element_rot_z->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_rot_z->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_rot_z->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_rot_z->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_rot_z->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_rot_z->GetString( "name" ), "sphere_1_universal3d_const_rot_z" );
        EXPECT_EQ( mjcf_element_rot_z->GetString( "type" ), "hinge" );
        EXPECT_EQ( mjcf_element_rot_z->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_rot_z->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_rot_z->GetVec3( "axis" ), { 0.0f, 0.0f, 1.0f } ) );
    }

    // Planar constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TSphere>( "sphere_2", 0.1f, loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodyPlanarConstraint>( "sphere_2_planar_const" );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::PLANAR );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::mujoco::TMujocoSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto mjc_constraint_adapter = dynamic_cast<loco::mujoco::TMujocoSingleBodyPlanarConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( mjc_constraint_adapter != nullptr );

        auto mjcf_elements = mjc_constraint_adapter->elements_resources();
        ASSERT_EQ( mjcf_elements.size(), 3 );

        auto mjcf_element_trans_x = mjcf_elements[0];
        ASSERT_TRUE( mjcf_element_trans_x != nullptr );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_trans_x->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "name" ), "sphere_2_planar_const_trans_x" );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element_trans_x->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_x->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_x->GetVec3( "axis" ), { 1.0f, 0.0f, 0.0f } ) );

        auto mjcf_element_trans_z = mjcf_elements[1];
        ASSERT_TRUE( mjcf_element_trans_z != nullptr );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_trans_z->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "name" ), "sphere_2_planar_const_trans_z" );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "type" ), "slide" );
        EXPECT_EQ( mjcf_element_trans_z->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_z->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_trans_z->GetVec3( "axis" ), { 0.0f, 0.0f, 1.0f } ) );

        auto mjcf_element_rot_y = mjcf_elements[2];
        ASSERT_TRUE( mjcf_element_rot_y != nullptr );
        EXPECT_TRUE( mjcf_element_rot_y->HasAttributeString( "name" ) );
        EXPECT_TRUE( mjcf_element_rot_y->HasAttributeString( "type" ) );
        EXPECT_TRUE( mjcf_element_rot_y->HasAttributeString( "limited" ) );
        EXPECT_TRUE( mjcf_element_rot_y->HasAttributeVec3( "pos" ) );
        EXPECT_TRUE( mjcf_element_rot_y->HasAttributeVec3( "axis" ) );
        EXPECT_EQ( mjcf_element_rot_y->GetString( "name" ), "sphere_2_planar_const_rot_y" );
        EXPECT_EQ( mjcf_element_rot_y->GetString( "type" ), "hinge" );
        EXPECT_EQ( mjcf_element_rot_y->GetString( "limited" ), "false" );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_rot_y->GetVec3( "pos" ), { 0.0f, 0.0f, 0.0f } ) );
        EXPECT_TRUE( tinymath::allclose( mjcf_element_rot_y->GetVec3( "axis" ), { 0.0f, 1.0f, 0.0f } ) );
    }
}