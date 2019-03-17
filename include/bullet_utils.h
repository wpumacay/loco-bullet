
#pragma once

#include <bullet_common.h>

// Assimp helper functionality
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace tysoc {
namespace bullet {
namespace utils {

    /**
    *   Mesh structure for non-primitive collision shapes
    */
    struct TMeshObject
    {
        std::vector< TVec3 > vertices;
    };

    /**
    *   Creates a bullet btVector3 from a TVec3
    *
    *   @param vec  TVec3 to convert
    */
    btVector3 toBtVec3( const TVec3& vec );

    /**
    *   Creates a TVec3 from a bullet btVector3
    *
    *   @param vec  btVector3 to convert
    */
    TVec3 fromBtVec3( const btVector3& vec );

    /**
    *   Creates a bullet btMatrix3x3 from a TMat3
    *
    *   @param mat  TMat3 to convert
    */
    btMatrix3x3 toBtMat3( const TMat3& mat );

    /**
    *   Creates a TMat3 from a bullet btMatrix3x3
    *
    *   @param mat  btMatrix3x3 to convert
    */
    TMat3 fromBtMat3( const btMatrix3x3& mat );

    /**
    *   Creates a TMat4 from a bullet btTransform
    *
    *   @param tf   btTransform to convert
    */
    TMat4 fromBtTransform( const btTransform& tf );

    /**
    *   Creates a btTransform from a bullet TMat4
    *
    *   @param mat  TMat4 to convert
    */
    btTransform toBtTransform( const TMat4& mat );

}}}