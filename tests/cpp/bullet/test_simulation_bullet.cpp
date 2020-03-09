
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_bullet.h>

TEST( TestLocoBulletSimulation, TestBulletSimulationFunctionality )
{
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

    auto simulation = std::make_unique<loco::bullet::TBulletSimulation>( scenario.get() );
    simulation->Initialize();
    simulation->Step();
    simulation->Reset();
    simulation->Pause();
    simulation->Resume();
    EXPECT_EQ( simulation->backendId(), "BULLET" );
    EXPECT_TRUE( simulation->bullet_world() != nullptr );
    EXPECT_TRUE( simulation->bullet_constraint_solver() != nullptr );
    EXPECT_TRUE( simulation->bullet_collision_dispatcher() != nullptr );
    EXPECT_TRUE( simulation->bullet_broadphase() != nullptr );
}