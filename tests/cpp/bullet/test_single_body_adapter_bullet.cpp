
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_bullet.h>
#include <primitives/loco_single_body_adapter_bullet.h>

struct BuildGroup
{
    std::unique_ptr<loco::TSingleBody> body;
    std::unique_ptr<loco::bullet::TBulletSingleBodyAdapter> body_adapter;
    std::unique_ptr<loco::bullet::TBulletSingleBodyColliderAdapter> collision_adapter;
};

BuildGroup build_body( const std::string& name,
                       const loco::eShapeType& shape,
                       const loco::TVec3& size,
                       const loco::eDynamicsType& dyntype,
                       const loco::TVec3& position,
                       const loco::TVec3& roteuler,
                       const loco::TInertialData inertia = loco::TInertialData(),
                       const std::string& mesh_filepath = "" )
{
    auto col_data = loco::TCollisionData();
    col_data.type = shape;
    col_data.size = size;
    col_data.mesh_data.filename = mesh_filepath;
    auto vis_data = loco::TVisualData();
    vis_data.type = shape;
    vis_data.size = size;
    vis_data.mesh_data.filename = mesh_filepath;
    auto body_data = loco::TBodyData();
    body_data.collision = col_data;
    body_data.visual = vis_data;
    body_data.inertia = inertia;
    body_data.dyntype = dyntype;

    auto body_obj = std::make_unique<loco::TSingleBody>( name, body_data, position, tinymath::rotation( roteuler ) );
    auto body_adapter = std::make_unique<loco::bullet::TBulletSingleBodyAdapter>( body_obj.get() );
    auto collision_ref = body_obj->collider();
    auto collision_adapter = std::make_unique<loco::bullet::TBulletSingleBodyColliderAdapter>( collision_ref );
    collision_ref->SetColliderAdapter( collision_adapter.get() );
    body_adapter->Build();

    return { std::move( body_obj ), std::move( body_adapter ), std::move( collision_adapter ) };
}

TEST( TestLocoBulletSingleBodyAdapter, TestLocoBulletSingleBodyAdapterBuild )
{
    loco::TLogger::Init();

    std::vector<std::string> vec_names = { "floor",
                                           "boxy",
                                           "box_obstacle",
                                           "monkey_head",
                                           "heavy_sphere" };
    std::vector<loco::TVec3> vec_sizes = { { 10.0, 10.0, 1.0 },
                                           { 0.1, 0.2, 0.3 },
                                           { 0.1, 0.2, 0.3 },
                                           { 0.2, 0.2, 0.2 },
                                           { 0.1, 0.1, 0.1 } };
    std::vector<loco::eShapeType> vec_shapes = { loco::eShapeType::PLANE,
                                                 loco::eShapeType::BOX,
                                                 loco::eShapeType::BOX,
                                                 loco::eShapeType::MESH,
                                                 loco::eShapeType::SPHERE };
    std::vector<loco::eDynamicsType> vec_dyntypes = { loco::eDynamicsType::STATIC,
                                                      loco::eDynamicsType::DYNAMIC,
                                                      loco::eDynamicsType::STATIC,
                                                      loco::eDynamicsType::DYNAMIC,
                                                      loco::eDynamicsType::DYNAMIC };
    std::vector<loco::TVec3> vec_positions = { { 0.0, 0.0, 0.0 },
                                               { 0.0, 0.0, 1.0 },
                                               { 0.0, 1.0, 1.0 },
                                               { 1.0, 0.0, 1.0 },
                                               { 1.0, 1.0, 1.0 } };
    std::vector<loco::TVec3> vec_eulers = { { 0.0, 0.0, 0.0 },
                                            { 0.2 * loco::PI, 0.2 * loco::PI, 0.2 * loco::PI },
                                            { 0.3 * loco::PI, 0.3 * loco::PI, 0.3 * loco::PI },
                                            { 0.4 * loco::PI, 0.4 * loco::PI, 0.4 * loco::PI },
                                            { 0.5 * loco::PI, 0.5 * loco::PI, 0.5 * loco::PI } };
    std::vector<double> vec_expected_masses = { 0.0, // static-dyntype
                                                0.1 * 0.2 * 0.3 * loco::DEFAULT_DENSITY,
                                                0.0, // static-dyntype
                                                0.0, // don't check (requires aabb)
                                                (4. / 3.) * loco::PI * 0.1 * 0.1 * 0.1 * loco::DEFAULT_DENSITY };
    const std::string monkey_filepath = loco::PATH_RESOURCES + "meshes/monkey.stl";
    auto scenario = std::make_unique<loco::TScenario>();
    for ( ssize_t i = 0; i < 5; i++ )
    {
        auto body_group = build_body( vec_names[i], vec_shapes[i], vec_sizes[i], vec_dyntypes[i],
                                      vec_positions[i], vec_eulers[i], loco::TInertialData(),
                                      ( vec_shapes[i] == loco::eShapeType::MESH ? monkey_filepath : "" ) );
        auto bt_rigid_body = body_group.body_adapter->rigid_body();
        ASSERT_TRUE( bt_rigid_body != nullptr );

        const auto tf = loco::bullet::mat4_from_bt( bt_rigid_body->getWorldTransform() );
        const auto expected_tf = body_group.body->tf();
        EXPECT_TRUE( tinymath::allclose( tf, expected_tf, 1e-2f ) );

        if ( vec_shapes[i] != loco::eShapeType::MESH )
        {
            const auto mass = bt_rigid_body->getMass();
            const auto expected_mass = vec_expected_masses[i];
            EXPECT_TRUE( std::abs( mass - expected_mass ) < 1e-5 );
        }

        scenario->AddSingleBody( std::move( body_group.body ) );
    }

    auto simulation = std::make_unique<loco::bullet::TBulletSimulation>( scenario.get() );
    simulation->Initialize();
}

