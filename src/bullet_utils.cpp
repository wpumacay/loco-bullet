
#include <bullet_utils.h>

namespace tysoc {
namespace bullet {
namespace utils {


    btVector3 toBtVec3( const TVec3& vec )
    {
        return btVector3( vec.x, vec.y, vec.z );
    }

    TVec3 fromBtVec3( const btVector3& vec )
    {
        return TVec3( vec.x(), vec.y(), vec.z() );
    }

    btMatrix3x3 toBtMat3( const TMat3& mat )
    {
        btMatrix3x3 _res;

        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res[i][j] = mat.buff[i + 3 * j];

        return _res;
    }

    TMat3 fromBtMat3( const btMatrix3x3& mat )
    {
        TMat3 _res;

        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res.buff[i + 3 * j] = mat[i][j];

        return _res;
    }

    void loadMesh( const std::string& filePath, TMeshObject& mesh )
    {
        auto _assimpScenePtr = aiImportFile( filePath.c_str(),
                                             aiProcessPreset_TargetRealtime_MaxQuality );

        if ( !_assimpScenePtr )
        {
            return;
        }

        // recursively copy the data from assimp to our data structure
        _processAssimpNode( _assimpScenePtr, _assimpScenePtr->mRootNode, mesh );

        // make sure we release the assimp resources
        aiReleaseImport( _assimpScenePtr );
    }

    void _processAssimpNode( const aiScene* assimpScenePtr,
                             aiNode* assimpNodePtr,
                             TMeshObject& mesh )
    {
        for ( size_t i = 0; i < assimpNodePtr->mNumMeshes; i++ )
        {
            aiMesh* _assimpMeshPtr = assimpScenePtr->mMeshes[ assimpNodePtr->mMeshes[i] ];
            _processAssimpMesh( _assimpMeshPtr, mesh );
        }

        for ( size_t i = 0; i < assimpNodePtr->mNumChildren; i++ )
        {
            _processAssimpNode( assimpScenePtr,
                                assimpNodePtr->mChildren[i], 
                                mesh );
        }
    }

    void _processAssimpMesh( aiMesh* assimpMeshPtr, TMeshObject& mesh )
    {
        std::vector< TVec3 > _vertices;

        for ( size_t i = 0; i < assimpMeshPtr->mNumVertices; i++ )
        {
            _vertices.push_back( TVec3( assimpMeshPtr->mVertices[i].x,
                                        assimpMeshPtr->mVertices[i].y,
                                        assimpMeshPtr->mVertices[i].z ) );
        }

        for ( size_t i = 0; i < assimpMeshPtr->mNumFaces; i++ )
        {
            aiFace _assimpFace = assimpMeshPtr->mFaces[i];
            // grab only in tris. we are assuming it comes this way
            // @TODO: Check this part as may have to support quads
            for ( size_t j = 0; j < _assimpFace.mNumIndices / 3; j++ )
            {
                auto _p1 = _vertices[ _assimpFace.mIndices[ 3 * j + 0 ] ];
                auto _p2 = _vertices[ _assimpFace.mIndices[ 3 * j + 1 ] ];
                auto _p3 = _vertices[ _assimpFace.mIndices[ 3 * j + 2 ] ];

                mesh.vertices.push_back( _p1 );
                mesh.vertices.push_back( _p2 );
                mesh.vertices.push_back( _p3 );
            }
        }
    }

    TMat4 fromBtTransform( const btTransform& tf )
    {
        auto _pos = fromBtVec3( tf.getOrigin() );
        auto _rot = fromBtMat3( tf.getBasis() );

        return TMat4::fromPositionAndRotation( _pos, _rot );
    }

    btTransform toBtTransform( const TMat4& mat )
    {
        auto _origin = toBtVec3( mat.getPosition() );
        auto _basis = toBtMat3( mat.getRotation() );

        return btTransform( _basis, _origin );
    }

}}}