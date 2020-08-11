
#include <loco_common_bullet.h>

namespace loco {
namespace bullet {

    btVector3 vec3_to_bt( const TVec3& vec )
    {
        return btVector3( vec.x(), vec.y(), vec.z() );
    }

    btMatrix3x3 mat3_to_bt( const TMat3& mat )
    {
        btMatrix3x3 bt_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                bt_mat[i][j] = mat( i, j );
        return bt_mat;
    }

    btTransform mat4_to_bt( const TMat4& mat )
    {
        return btTransform( mat3_to_bt( TMat3( mat ) ),
                            vec3_to_bt( TVec3( mat.col( 3 ) ) ) );
    }

    TVec3 vec3_from_bt( const btVector3& vec )
    {
        return TVec3( vec.x(), vec.y(), vec.z() );
    }

    TMat3 mat3_from_bt( const btMatrix3x3& mat )
    {
        TMat3 tm_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                tm_mat( i, j ) = mat[i][j];
        return tm_mat;
    }

    TMat4 mat4_from_bt( const btTransform& mat )
    {
        return TMat4( mat3_from_bt( mat.getBasis() ),
                      vec3_from_bt( mat.getOrigin() ) );
    }

    std::string bt_shape_enum_to_str( const BroadphaseNativeTypes& bt_shape_enum )
    {
        switch ( bt_shape_enum )
        {
            case BroadphaseNativeTypes::BOX_SHAPE_PROXYTYPE                 : return "bt_box_shape";
            case BroadphaseNativeTypes::TRIANGLE_SHAPE_PROXYTYPE            : return "bt_triangle_shape";
            case BroadphaseNativeTypes::TETRAHEDRAL_SHAPE_PROXYTYPE         : return "bt_tetrahedral_shape";
            case BroadphaseNativeTypes::CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE : return "bt_convex_traingle_mesh_shape";
            case BroadphaseNativeTypes::CONVEX_HULL_SHAPE_PROXYTYPE         : return "bt_convex_hull_shape";
            case BroadphaseNativeTypes::CUSTOM_POLYHEDRAL_SHAPE_TYPE        : return "bt_polyhedral_shape";
            case BroadphaseNativeTypes::SPHERE_SHAPE_PROXYTYPE              : return "bt_sphere_shape";
            case BroadphaseNativeTypes::MULTI_SPHERE_SHAPE_PROXYTYPE        : return "bt_multi_sphere_shape";
            case BroadphaseNativeTypes::CAPSULE_SHAPE_PROXYTYPE             : return "bt_capsule_shape";
            case BroadphaseNativeTypes::CONVEX_SHAPE_PROXYTYPE              : return "bt_convex_shape";
            case BroadphaseNativeTypes::CYLINDER_SHAPE_PROXYTYPE            : return "bt_cylinder_shape";
            case BroadphaseNativeTypes::TRIANGLE_MESH_SHAPE_PROXYTYPE       : return "bt_triangle_mesh_shape";
            case BroadphaseNativeTypes::TERRAIN_SHAPE_PROXYTYPE             : return "bt_terrain_shape";
            case BroadphaseNativeTypes::EMPTY_SHAPE_PROXYTYPE               : return "bt_empty_shape";
            case BroadphaseNativeTypes::STATIC_PLANE_PROXYTYPE              : return "bt_static_plane_shape";
            case BroadphaseNativeTypes::COMPOUND_SHAPE_PROXYTYPE            : return "bt_compound_shape";
        }

        LOCO_CORE_ERROR( "bt_shape_enum_to_str >>> shape enum {0} should not be in use (not supported)", bt_shape_enum );
        return "bt_INVALID_shape";
    }

    std::unique_ptr<btCollisionShape> CreateCollisionShape( const TShapeData& data )
    {
        switch ( data.type )
        {
            case eShapeType::PLANE :
                return std::make_unique<btStaticPlaneShape>( btVector3( 0.0, 0.0, 1.0 ), 0.0 );

            case eShapeType::BOX :
                return std::make_unique<btBoxShape>( vec3_to_bt( 0.5 * data.size ) );

            case eShapeType::SPHERE :
                return std::make_unique<btSphereShape>( data.size.x() );

            case eShapeType::CAPSULE :
                return std::make_unique<btCapsuleShapeZ>( data.size.x(), data.size.y() );

            case eShapeType::CYLINDER :
                return std::make_unique<btCylinderShapeZ>( btVector3( data.size.x(), data.size.x(), 0.5 * data.size.y() ) );

            case eShapeType::ELLIPSOID :
                return CreateConvexHull( CreateEllipsoidVertices( data ) );

            case eShapeType::CONVEX_MESH :
                return CreateConvexHull( CreateConvexMeshVertices( data ) );

            case eShapeType::TRIANGULAR_MESH :
            {
                std::vector<float> mesh_vertices;
                std::vector<int> mesh_faces;
                CreateTriangularMeshData( data, mesh_vertices, mesh_faces );
                return CreateTriangularMesh( mesh_vertices, mesh_faces );
            }

            case eShapeType::HEIGHTFIELD :
            {
                const auto& heights = data.hfield_data.heights;
                const ssize_t nwidth_samples = data.hfield_data.nWidthSamples;
                const ssize_t ndepth_samples = data.hfield_data.nDepthSamples;
                const int up_axis = 2;
                const float max_height = *std::max_element( heights.cbegin(), heights.cend() );

                auto bt_hfield_shape = std::make_unique<btHeightfieldTerrainShape>( nwidth_samples,
                                                                                    ndepth_samples,
                                                                                    heights.data(),
                                                                                    btScalar( 1.0 ),
                                                                                    btScalar( 0.0 ),
                                                                                    btScalar( max_height * data.size.z() ),
                                                                                    up_axis, PHY_FLOAT, false );
                bt_hfield_shape->setLocalScaling( vec3_to_bt( { data.size.x() / ( nwidth_samples - 1 ),
                                                                data.size.y() / ( ndepth_samples - 1 ),
                                                                data.size.z() } ) );
                return std::move( bt_hfield_shape );
            }

            case eShapeType::COMPOUND :
            {
                auto bt_compound_shape = std::make_unique<btCompoundShape>();
                auto children_data = data.children;
                auto children_tfs = data.children_tfs;
                for ( ssize_t i = 0; i < children_data.size(); i++ )
                    bt_compound_shape->addChildShape( mat4_to_bt( children_tfs[i] ),
                                                      CreateCollisionShape( children_data[i] ).release() );
                return std::move( bt_compound_shape );
            }
        }

        LOCO_CORE_ERROR( "CreateCollisionShape >>> unsupported shape: {0}", ToString( data.type ) );
        return nullptr;
    }

    double ComputeVolumeFromBtShape( const btCollisionShape* collision_shape )
    {
        LOCO_CORE_ASSERT( collision_shape, "ComputeVolumeFromBtShape >>> must pass a valid bullet collision-shape" );

        switch ( collision_shape->getShapeType() )
        {
            case BroadphaseNativeTypes::BOX_SHAPE_PROXYTYPE :
            {
                const auto box_shape = static_cast<const btBoxShape*>( collision_shape );
                const auto box_dimensions = box_shape->getHalfExtentsWithMargin();
                return 8.0 * box_dimensions.x() * box_dimensions.y() * box_dimensions.z();
            }
            case BroadphaseNativeTypes::SPHERE_SHAPE_PROXYTYPE :
            {
                const auto sphere_shape = static_cast<const btSphereShape*>( collision_shape );
                const auto sphere_radius = sphere_shape->getRadius();
                return (4. / 3.) * loco::PI * sphere_radius * sphere_radius * sphere_radius;
            }
            case BroadphaseNativeTypes::CYLINDER_SHAPE_PROXYTYPE :
            {
                const auto cylinder_shape = static_cast<const btCylinderShapeZ*>( collision_shape );
                const auto cylinder_radius = cylinder_shape->getRadius();
                const auto cylinder_up_axis = cylinder_shape->getUpAxis();
                const auto cylinder_half_extents = cylinder_shape->getHalfExtentsWithMargin();
                const auto cylinder_height = 2.0 * cylinder_half_extents[cylinder_up_axis];
                return loco::PI * cylinder_radius * cylinder_radius * cylinder_height;
            }
            case BroadphaseNativeTypes::CAPSULE_SHAPE_PROXYTYPE :
            {
                const auto capsule_shape = static_cast<const btCapsuleShapeZ*>( collision_shape );
                const auto capsule_radius = capsule_shape->getRadius();
                const auto capsule_height = 2.0 * capsule_shape->getHalfHeight();
                return loco::PI * capsule_radius * capsule_radius * capsule_height +
                       (4. / 3.) * loco::PI * capsule_radius * capsule_radius * capsule_radius;
            }
            case BroadphaseNativeTypes::CONVEX_HULL_SHAPE_PROXYTYPE :
            case BroadphaseNativeTypes::COMPOUND_SHAPE_PROXYTYPE :
            {
                btVector3 aabb_min, aabb_max;
                collision_shape->getAabb( btTransform::getIdentity(), aabb_min, aabb_max );
                return ( aabb_max.x() - aabb_min.x() ) *
                       ( aabb_max.y() - aabb_min.y() ) *
                       ( aabb_max.z() - aabb_min.z() );
            }
        }

        LOCO_CORE_ERROR( "ComputeVolumeFromBtShape >>> unsupported shape: {0}",
                         bt_shape_enum_to_str( (BroadphaseNativeTypes)collision_shape->getShapeType() ) );
        return 1.0;
    }

    std::unique_ptr<btConvexHullShape> CreateConvexHull( const std::vector<TVec3>& mesh_vertices )
    {
        auto convex_hull_shape = std::make_unique<btConvexHullShape>();
        const bool recalc_local_aabb = false;
        for ( ssize_t i = 0; i < mesh_vertices.size(); i++ )
            convex_hull_shape->addPoint( vec3_to_bt( mesh_vertices[i] ), recalc_local_aabb );

        convex_hull_shape->recalcLocalAabb();
        convex_hull_shape->optimizeConvexHull();
        return std::move( convex_hull_shape );
    }

    std::unique_ptr<btBvhTriangleMeshShape> CreateTriangularMesh( const std::vector<float>& mesh_vertices,
                                                                  const std::vector<int>& mesh_faces )
    {
        // Create some buffers to hold the data dynamically (will be attached to bullet's mesh-data struct)
        float* vertices_data = new float[mesh_vertices.size()];
        int* faces_data = new int[mesh_faces.size()];
        memcpy( vertices_data, mesh_vertices.data(), sizeof(float) * mesh_vertices.size() );
        memcpy( faces_data, mesh_faces.data(), sizeof(int) * mesh_faces.size() );
        //------------------------------------------------------------------------------------------

        btIndexedMesh bt_mesh_data;
        bt_mesh_data.m_vertexDataPtr = vertices_data; // keep the data internally
        bt_mesh_data.m_vertexBase = (const unsigned char*)vertices_data;
        bt_mesh_data.m_vertexStride = sizeof(float) * 3;
        bt_mesh_data.m_numVertices = mesh_vertices.size() / 3;
        bt_mesh_data.m_triangleDataPtr = faces_data; // keep the data internally
        bt_mesh_data.m_triangleIndexBase = (const unsigned char*)faces_data;
        bt_mesh_data.m_triangleIndexStride = sizeof(int) * 3;
        bt_mesh_data.m_numTriangles = mesh_faces.size() / 3;
        bt_mesh_data.m_indexType = PHY_INTEGER;

        auto mesh_interface = new btTriangleIndexVertexArray();
        mesh_interface->addIndexedMesh( bt_mesh_data, PHY_INTEGER );
        const bool use_quantized_aabb_compression = true;

        auto bvh_trimesh_shape = std::make_unique<btBvhTriangleMeshShape>( mesh_interface, use_quantized_aabb_compression );
        return std::move( bvh_trimesh_shape );
    }

    std::vector<TVec3> CreateEllipsoidVertices( const TShapeData& data )
    {
        const ssize_t ndiv_1 = 20;
        const ssize_t ndiv_2 = 20;
        const ssize_t num_vertices = ( ndiv_1 + 1 ) * ( ndiv_2 + 1 );
        std::vector<TVec3> vertices( num_vertices );

        for ( ssize_t i = 0; i <= ndiv_1; i++ )
        {
            for ( ssize_t j = 0; j <= ndiv_2; j++ )
            {
                float ang_theta = 2.0 * loco::PI * (float)i / ndiv_1;
                float ang_phi = -0.5 * loco::PI + loco::PI * (float)j / ndiv_2;
                vertices[i * ( ndiv_2 + 1 ) + j].x() = data.size.x() * std::cos( ang_theta ) * std::cos( ang_phi );
                vertices[i * ( ndiv_2 + 1 ) + j].y() = data.size.y() * std::sin( ang_theta ) * std::cos( ang_phi );
                vertices[i * ( ndiv_2 + 1 ) + j].z() = data.size.z() * std::sin( ang_phi );
            }
        }
        return vertices;
    }

    std::vector<TVec3> CreateConvexMeshVertices( const TShapeData& data )
    {
        std::vector<TVec3> mesh_vertices;
        if ( data.mesh_data.filename != "" )
            _CollectMeshVerticesFromFile( mesh_vertices, data );
        else if ( data.mesh_data.vertices.size() > 0 && data.mesh_data.faces.size() > 0 )
            _CollectMeshVerticesFromUser( mesh_vertices, data );
        else
            LOCO_CORE_ERROR( "CreateConvexMeshVertices >>> tried to construct a mesh without any data" );
        return mesh_vertices;
    }

    void _CollectMeshVerticesFromFile( std::vector<TVec3>& mesh_vertices, const TShapeData& data )
    {
        const std::string filepath = data.mesh_data.filename;
        auto assimp_scene = std::unique_ptr<const aiScene,aiSceneDeleter>( aiImportFile( filepath.c_str(),
                                                                                aiProcessPreset_TargetRealtime_MaxQuality |
                                                                                aiProcess_PreTransformVertices ) );
        if ( !assimp_scene )
        {
            LOCO_CORE_ERROR( "_CollectMeshVerticesFromFile >>> couldn't open model {0}", filepath );
            return;
        }

        std::stack<const aiNode*> dfs_traversal;
        dfs_traversal.push( assimp_scene->mRootNode );
        while ( !dfs_traversal.empty() )
        {
            auto assimp_node = dfs_traversal.top();
            dfs_traversal.pop();
            if ( !assimp_node )
                continue;

            // It's enough with the vertex data, as we'll only construct the convex-hull
            for ( ssize_t i = 0; i < assimp_node->mNumMeshes; i++ )
            {
                const aiMesh* assimp_mesh = assimp_scene->mMeshes[assimp_node->mMeshes[i]];
                for ( ssize_t v = 0; v < assimp_mesh->mNumVertices; v++ )
                    mesh_vertices.push_back( { data.size.x() * assimp_mesh->mVertices[v].x,
                                               data.size.y() * assimp_mesh->mVertices[v].y,
                                               data.size.z() * assimp_mesh->mVertices[v].z } );
            }

            for ( ssize_t i = 0; i < assimp_node->mNumChildren; i++ )
                dfs_traversal.push( assimp_node->mChildren[i] );
        }
    }

    void _CollectMeshVerticesFromUser( std::vector<TVec3>& mesh_vertices, const TShapeData& data )
    {
        // It's enough with the vertex data, as we'll only construct the convex-hull
        if ( data.mesh_data.vertices.size() % 3 != 0 )
            LOCO_CORE_ERROR( "_CollectMeshVerticesFromUser >>> there must be 3 elements per vertex" );

        const std::vector<float>& vertices = data.mesh_data.vertices;
        const ssize_t num_vertices = vertices.size() / 3;
        mesh_vertices.reserve( num_vertices );
        for ( ssize_t i = 0; i < num_vertices; i++ )
            mesh_vertices.push_back( { data.size.x() * vertices[3 * i + 0],
                                       data.size.y() * vertices[3 * i + 1],
                                       data.size.z() * vertices[3 * i + 2] } );
    }

    void CreateTriangularMeshData( const TShapeData& data, std::vector<float>& dst_vertices, std::vector<int>& dst_faces )
    {
        auto& mesh_data = data.mesh_data;
        /**/ if ( mesh_data.filename != "" )
            _CollectTrimeshDataFromFile( dst_vertices, dst_faces, data );
        else if ( mesh_data.vertices.size() > 0 && mesh_data.faces.size() > 0 )
            _CollectTrimeshDataFromUser( dst_vertices, dst_faces, data );
        else
            LOCO_CORE_ERROR( "CreateTriangularMeshData >>> tried to construct a tri-mesh without any data" );
    }

    void _CollectTrimeshDataFromFile( std::vector<float>& dst_vertices, std::vector<int>& dst_faces, const TShapeData& data )
    {
        const std::string filepath = data.mesh_data.filename;
        auto assimp_scene = std::unique_ptr<const aiScene, aiSceneDeleter>( aiImportFile( 
                                                                                filepath.c_str(),
                                                                                aiProcessPreset_TargetRealtime_MaxQuality |
                                                                                aiProcess_PreTransformVertices ) );
        if ( !assimp_scene )
        {
            LOCO_CORE_ERROR( "_CollectTrimeshDataFromFile >>> couldn't open model {0}", filepath );
            return;
        }

        std::stack<const aiNode*> dfs_traversal;
        dfs_traversal.push( assimp_scene->mRootNode );
        while( !dfs_traversal.empty() )
        {
            auto assimp_node = dfs_traversal.top();
            dfs_traversal.pop();
            if ( !assimp_node )
                continue;

            for ( ssize_t i = 0; i < assimp_node->mNumMeshes; i++ )
            {
                auto assimp_mesh = assimp_scene->mMeshes[assimp_node->mMeshes[i]];
                for ( ssize_t v = 0; v < assimp_mesh->mNumVertices; v++ )
                {
                    dst_vertices.push_back( data.size.x() * assimp_mesh->mVertices[v].x );
                    dst_vertices.push_back( data.size.y() * assimp_mesh->mVertices[v].y );
                    dst_vertices.push_back( data.size.z() * assimp_mesh->mVertices[v].z );
                }
                for ( ssize_t f = 0; f < assimp_mesh->mNumFaces; f++ )
                {
                    auto assimp_face = assimp_mesh->mFaces[f];
                    dst_faces.push_back( assimp_face.mIndices[0] );
                    dst_faces.push_back( assimp_face.mIndices[1] );
                    dst_faces.push_back( assimp_face.mIndices[2] );
                }
            }

            for ( ssize_t i = 0; i < assimp_node->mNumChildren; i++ )
                dfs_traversal.push( assimp_node->mChildren[i] );
        }
    }

    void _CollectTrimeshDataFromUser( std::vector<float>& dst_vertices, std::vector<int>& dst_faces, const TShapeData& data )
    {
        const auto& mesh_data = data.mesh_data;
        const auto mesh_scale = data.size;
        if ( mesh_data.vertices.size() % 3 != 0 )
            LOCO_CORE_ERROR( "_CollectTrimeshDataFromUser >>> there must be 3 elements per vertex" );
        if ( mesh_data.faces.size() % 3 != 0 )
            LOCO_CORE_ERROR( "_CollectTrimeshDataFromUser >>> there must be 3 indices per face" );

        // The data is already in the format we require, so just apply the scale to the vertices
        const ssize_t num_vertices = mesh_data.vertices.size() / 3;
        for ( ssize_t v = 0; v < num_vertices; v++ )
        {
            dst_vertices.push_back( mesh_scale.x() * mesh_data.vertices[3 * v + 0] );
            dst_vertices.push_back( mesh_scale.y() * mesh_data.vertices[3 * v + 1] );
            dst_vertices.push_back( mesh_scale.z() * mesh_data.vertices[3 * v + 2] );
        }
        const ssize_t num_faces = mesh_data.faces.size() /3 ;
        for ( ssize_t f = 0; f < num_faces; f++ )
        {
            dst_faces.push_back( mesh_data.faces[3 * f + 0] );
            dst_faces.push_back( mesh_data.faces[3 * f + 1] );
            dst_faces.push_back( mesh_data.faces[3 * f + 2] );
        }
    }

    void aiSceneDeleter::operator() ( const aiScene* assimp_scene ) const
    {
        aiReleaseImport( assimp_scene );
    }
}}