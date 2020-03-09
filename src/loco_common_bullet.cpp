
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

            case eShapeType::MESH :
                return CreateConvexHull( CreateMeshVertices( data ) );

            case eShapeType::HFIELD :
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
        }

        LOCO_CORE_ERROR( "CreateCollisionShape >>> unsupported shape: {0}", ToString( data.type ) );
        return nullptr;
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

    std::vector<TVec3> CreateMeshVertices( const TShapeData& data )
    {
        std::vector<TVec3> mesh_vertices;
        if ( data.mesh_data.filename != "" )
            _CollectMeshVerticesFromFile( mesh_vertices, data );
        else if ( data.mesh_data.vertices.size() > 0 && data.mesh_data.faces.size() > 0 )
            _CollectMeshVerticesFromUser( mesh_vertices, data );
        else
            LOCO_CORE_ERROR( "CreateMeshVertices >>> tried to construct a mesh without any data" );
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
            mesh_vertices[i] = { data.size.x() * vertices[3 * i + 0],
                                 data.size.y() * vertices[3 * i + 1],
                                 data.size.z() * vertices[3 * i + 2] };
    }

    void aiSceneDeleter::operator() ( const aiScene* assimp_scene ) const
    {
        aiReleaseImport( assimp_scene );
    }

}}