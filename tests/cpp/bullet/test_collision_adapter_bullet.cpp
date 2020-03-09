
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_bullet.h>
#include <adapters/loco_collision_adapter_bullet.h>

bool allclose_vec3( const btVector3& bt_vec_1, const btVector3& bt_vec_2, double tolerance = 1e-5 )
{
    return ( std::abs( bt_vec_1.x() - bt_vec_2.x() ) <= tolerance ) &&
           ( std::abs( bt_vec_1.y() - bt_vec_2.y() ) <= tolerance ) &&
           ( std::abs( bt_vec_1.z() - bt_vec_2.z() ) <= tolerance );
}

bool allclose_vec3( const btVector3& bt_vec, const loco::TVec3& vec, double tolerance = 1e-5 )
{
    return ( std::abs( bt_vec.x() - vec.x() ) <= tolerance ) &&
           ( std::abs( bt_vec.y() - vec.y() ) <= tolerance ) &&
           ( std::abs( bt_vec.z() - vec.z() ) <= tolerance );
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

TEST( TestLocoBulletCollisionAdapter, TestLocoBulletCollisionAdapterBuild )
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
    std::vector<std::unique_ptr<loco::bullet::TBulletCollisionAdapter>> vec_colliders_adapters;
    for ( size_t i = 0; i < vec_col_data.size(); i++ )
    {
        const auto collider_name = loco::ToString( vec_col_data[i].type ) + "_collider";
        auto col_obj = std::make_unique<loco::TCollision>( collider_name, vec_col_data[i] );
        auto col_adapter = std::make_unique<loco::bullet::TBulletCollisionAdapter>( col_obj.get() );
        col_adapter->Build();
        ASSERT_TRUE( col_adapter->collision_shape() != nullptr );
        vec_colliders.push_back( std::move( col_obj ) );
        vec_colliders_adapters.push_back( std::move( col_adapter ) );
    }

    std::vector<loco::TSizef> vec_expected_sizes = { { 0.05f, 0.1f, 0.15f },
                                                     { 0.1f },
                                                     { 5.0f, 5.0f, 1.0f },
                                                     { 0.2f, 0.4f },
                                                     { 0.2f, 0.4f },
                                                     { 0.2f, 0.3f, 0.4f } };
    std::vector<BroadphaseNativeTypes> vec_expected_types = { BroadphaseNativeTypes::BOX_SHAPE_PROXYTYPE,
                                                              BroadphaseNativeTypes::SPHERE_SHAPE_PROXYTYPE,
                                                            #ifdef LOCO_BULLET_SINGLE_BODIES_USE_STATIC_PLANE
                                                              BroadphaseNativeTypes::STATIC_PLANE_PROXYTYPE,
                                                            #else
                                                              BroadphaseNativeTypes::BOX_SHAPE_PROXYTYPE,
                                                            #endif /* LOCO_BULLET_SINGLE_BODIES_USE_STATIC_PLANE */
                                                              BroadphaseNativeTypes::CYLINDER_SHAPE_PROXYTYPE,
                                                              BroadphaseNativeTypes::CAPSULE_SHAPE_PROXYTYPE,
                                                              BroadphaseNativeTypes::CONVEX_HULL_SHAPE_PROXYTYPE };
    for ( size_t i = 0; i < vec_colliders_adapters.size(); i++ )
    {
        auto bt_collision_shape = vec_colliders_adapters[i]->collision_shape();
        EXPECT_EQ( bt_collision_shape->getShapeType(), vec_expected_types[i] );
        if ( auto box_shape = dynamic_cast<btBoxShape*>( bt_collision_shape ) )
        {
            const btVector3 bt_half_extents = box_shape->getHalfExtentsWithMargin();
            const loco::TVec3 expected_half_extensions = { 0.05f, 0.1f, 0.15f };
            EXPECT_TRUE( allclose_vec3( bt_half_extents, expected_half_extensions ) );
        }
        else if ( auto sphere_shape = dynamic_cast<btSphereShape*>( bt_collision_shape ) )
        {
            const btScalar bt_radius = sphere_shape->getRadius();
            const loco::TScalar expected_radius = 0.1;
            EXPECT_TRUE( std::abs( bt_radius - expected_radius ) < 1e-5 );
        }
        else if ( auto cylinder_shape = dynamic_cast<btCylinderShapeZ*>( bt_collision_shape ) )
        {
            const btVector3 bt_half_extents = cylinder_shape->getHalfExtentsWithMargin();
            const loco::TScalar expected_radius = 0.2;
            const loco::TScalar expected_half_height = 0.4;
            EXPECT_TRUE( std::abs( bt_half_extents.x() - expected_radius ) < 1e-5 );
            EXPECT_TRUE( std::abs( bt_half_extents.z() - expected_half_height ) < 1e-5 );
        }
        else if ( auto capsule_shape = dynamic_cast<btCapsuleShapeZ*>( bt_collision_shape ) )
        {
            const btScalar bt_radius = capsule_shape->getRadius();
            const btScalar bt_half_height = capsule_shape->getHalfHeight();
            const loco::TScalar expected_radius = 0.2;
            const loco::TScalar expected_half_height = 0.4;
            EXPECT_TRUE( std::abs( bt_radius - expected_radius ) < 1e-5 );
            EXPECT_TRUE( std::abs( bt_half_height - expected_half_height ) < 1e-5 );
        }
    }
}

TEST( TestLocoBulletCollisionAdapter, TestLocoBulletCollisionAdapterMeshBuild )
{
    loco::TLogger::Init();

    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::MESH;
    col_data.size = { 0.2f, 0.2f, 0.2f };
    col_data.mesh_data.filename = loco::PATH_RESOURCES + "meshes/monkey.stl";

    const auto collider_name = loco::ToString( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TCollision>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::bullet::TBulletCollisionAdapter>( col_obj.get() );
    col_adapter->Build();
    auto bt_collision_shape = col_adapter->collision_shape();
    ASSERT_TRUE( bt_collision_shape != nullptr );
    EXPECT_EQ( bt_collision_shape->getShapeType(), BroadphaseNativeTypes::CONVEX_HULL_SHAPE_PROXYTYPE );
    auto bt_cvhull_shape = dynamic_cast<btConvexHullShape*>( bt_collision_shape );
    ASSERT_TRUE( bt_cvhull_shape != nullptr );
}

TEST( TestLocoBulletCollisionAdapter, TestLocoBulletCollisionAdapterMeshUserBuild )
{
    loco::TLogger::Init();

    auto vertices_faces = create_mesh_tetrahedron();

    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::MESH;
    col_data.size = { 0.2f, 0.2f, 0.2f };
    col_data.mesh_data.vertices = vertices_faces.first;
    col_data.mesh_data.faces = vertices_faces.second;

    const auto collider_name = loco::ToString( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TCollision>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::bullet::TBulletCollisionAdapter>( col_obj.get() );
    col_adapter->Build();
    auto bt_collision_shape = col_adapter->collision_shape();
    ASSERT_TRUE( bt_collision_shape != nullptr );
    EXPECT_EQ( bt_collision_shape->getShapeType(), BroadphaseNativeTypes::CONVEX_HULL_SHAPE_PROXYTYPE );
    auto bt_cvhull_shape = dynamic_cast<btConvexHullShape*>( bt_collision_shape );
    ASSERT_TRUE( bt_cvhull_shape != nullptr );
    const ssize_t expected_num_tetrahedron_vertices = 4;
    EXPECT_EQ( bt_cvhull_shape->getNumVertices(), expected_num_tetrahedron_vertices );
}

TEST( TestLocoBulletCollisionAdapter, TestLocoBulletCollisionAdapterHfieldBuild )
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
                                        loco::bullet::LOCO_BULLET_HFIELD_BASE };

    const auto collider_name = loco::ToString( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TCollision>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::bullet::TBulletCollisionAdapter>( col_obj.get() );
    col_adapter->Build();
    auto bt_collision_shape = col_adapter->collision_shape();
    ASSERT_TRUE( bt_collision_shape != nullptr );
    EXPECT_EQ( bt_collision_shape->getShapeType(), BroadphaseNativeTypes::TERRAIN_SHAPE_PROXYTYPE );
    auto bt_hfield_shape = dynamic_cast<btHeightfieldTerrainShape*>( bt_collision_shape );
    ASSERT_TRUE( bt_hfield_shape != nullptr );
    const btVector3 bt_scaling = bt_hfield_shape->getLocalScaling();
    const loco::TScalar expected_delta_width = col_data.size.x() / ( num_width_samples - 1 );
    const loco::TScalar expected_delta_depth = col_data.size.y() / ( num_depth_samples - 1 );
    const loco::TScalar expected_scale_height = col_data.size.z();
    EXPECT_TRUE( std::abs( bt_scaling.x() - expected_delta_width ) < 1e-5 );
    EXPECT_TRUE( std::abs( bt_scaling.y() - expected_delta_depth ) < 1e-5 );
    EXPECT_TRUE( std::abs( bt_scaling.z() - expected_scale_height ) < 1e-5 );
}