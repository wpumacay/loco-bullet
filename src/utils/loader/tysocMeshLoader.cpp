
#include <utils/loader/tysocMeshLoader.h>



namespace loader
{

    void loadMesh( const std::string& filePath, TMeshObject& mesh )
    {
        auto _assimpScenePtr = aiImportFile( filePath.c_str(),
                                             aiProcessPreset_TargetRealtime_MaxQuality );

        if ( !_assimpScenePtr )
        {
            return;
        }

        // recursively copy the data from assimp to our data structure
        _processAssimpNode( _assimpScenePtr->mRootNode, _assimpScenePtr, mesh );

        // make sure we release the assimp resources
        aiReleaseImport( _assimpScenePtr );
    }

    void _processAssimpNode( aiNode* assimpNodePtr, 
                             const aiScene* assimpScenePtr,
                             TMeshObject& mesh )
    {
        for ( size_t i = 0; i < assimpNodePtr->mNumMeshes; i++ )
        {
            aiMesh* _assimpMeshPtr = assimpScenePtr->mMeshes[ assimpNodePtr->mMeshes[i] ];
            _processAssimpMesh( _assimpMeshPtr, mesh );
        }

        for ( size_t i = 0; i < assimpNodePtr->mNumChildren; i++ )
        {
            _processAssimpNode( assimpNodePtr->mChildren[i],
                                assimpScenePtr, mesh );
        }
    }

    void _processAssimpMesh( aiMesh* assimpMeshPtr, TMeshObject& mesh )
    {
        std::vector< TMeshVertex > _vertices;

        for ( size_t i = 0; i < assimpMeshPtr->mNumVertices; i++ )
        {
            _vertices.push_back( TMeshVertex( assimpMeshPtr->mVertices[i].x,
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

}