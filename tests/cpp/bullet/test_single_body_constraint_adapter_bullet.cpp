
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_bullet.h>
#include <primitives/loco_single_body_constraint_adapter_bullet.h>
#include <primitives/loco_single_body_adapter_bullet.h>

TEST( TestLocoBulletConstraintAdapter, TestLocoBulletConstraintAdapterBuild )
{
    loco::InitUtils();

    auto scenario = std::make_unique<loco::TScenario>();

    // Revolute constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TBox>( "pole_0", loco::TVec3( 0.2f, 0.2f, 1.0f ), loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodyRevoluteConstraint>( "pole_0_rev_const", loco::TMat4( loco::TMat3(), loco::TVec3( 0.0f, 0.0f, 0.5f ) ), loco::TVec3( 1.0f, 0.0f, 0.0f ) );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::REVOLUTE );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::bullet::TBulletSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();


        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto bt_constraint_adapter = dynamic_cast<loco::bullet::TBulletSingleBodyRevoluteConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( bt_constraint_adapter != nullptr );
    }

    // Prismatic constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TBox>( "platform_0", loco::TVec3( 1.0f, 1.0f, 0.2f ), loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodyPrismaticConstraint>( "platform_0_prism_const", loco::TMat4(), loco::TVec3( 0.0f, 0.0f, 1.0f ) );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::PRISMATIC );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::bullet::TBulletSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto bt_constraint_adapter = dynamic_cast<loco::bullet::TBulletSingleBodyPrismaticConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( bt_constraint_adapter != nullptr );
    }

    // Spherical constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TCapsule>( "rod_0", 0.1f, 1.0f, loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodySphericalConstraint>( "rod_0_spherical_const", loco::TMat4( loco::TMat3(), loco::TVec3( 0.0f, 0.0f, 0.5f ) ) );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::SPHERICAL );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::bullet::TBulletSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto bt_constraint_adapter = dynamic_cast<loco::bullet::TBulletSingleBodySphericalConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( bt_constraint_adapter != nullptr );
    }

    // Translational3d constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TSphere>( "sphere_0", 0.1f, loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodyTranslational3dConstraint>( "sphere_0_translational3d_const" );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::TRANSLATIONAL3D );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::bullet::TBulletSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto bt_constraint_adapter = dynamic_cast<loco::bullet::TBulletSingleBodyTranslational3dConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( bt_constraint_adapter != nullptr );
    }

    // Universal3d constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TSphere>( "sphere_1", 0.1f, loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodyUniversal3dConstraint>( "sphere_1_universal3d_const" );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::UNIVERSAL3D );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::bullet::TBulletSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto bt_constraint_adapter = dynamic_cast<loco::bullet::TBulletSingleBodyUniversal3dConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( bt_constraint_adapter != nullptr );
    }

    // Planar constraint
    {
        auto body = scenario->AddSingleBody( std::make_unique<loco::TSphere>( "sphere_2", 0.1f, loco::TVec3(), loco::TMat3() ) );
        auto constraint = std::make_unique<loco::TSingleBodyPlanarConstraint>( "sphere_2_planar_const" );
        EXPECT_EQ( constraint->constraint_type(), loco::eConstraintType::PLANAR );
        body->SetConstraint( std::move( constraint ) );
        auto body_adapter = std::make_unique<loco::bullet::TBulletSingleBodyAdapter>( body );
        body->SetBodyAdapter( body_adapter.get() );
        body_adapter->Build();

        auto body_constraint = body->constraint();
        ASSERT_TRUE( body_constraint != nullptr );
        auto constraint_adapter = body_constraint->constraint_adapter();
        ASSERT_TRUE( constraint_adapter != nullptr );
        auto bt_constraint_adapter = dynamic_cast<loco::bullet::TBulletSingleBodyPlanarConstraintAdapter*>( constraint_adapter );
        ASSERT_TRUE( bt_constraint_adapter != nullptr );
    }
}