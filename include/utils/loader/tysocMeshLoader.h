
#pragma once

#include <vector>
#include <string>

// Assimp helper functionality
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace loader
{

    struct TMeshVertex
    {
        float x;
        float y;
        float z;

        TMeshVertex()
        {
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }

        TMeshVertex( float x, float y, float z )
        {
            this->x = x;
            this->y = y;
            this->z = z;
        }
    };

    struct TMeshObject
    {
        std::vector< TMeshVertex > vertices;
    };

    void loadMesh( const std::string& filePath, TMeshObject& mesh );
    void _processAssimpNode( aiNode* assimpNodePtr, 
                             const aiScene* assimpScenePtr,
                             TMeshObject& mesh );
    void _processAssimpMesh( aiMesh* assimpMeshPtr, TMeshObject& mesh );
}