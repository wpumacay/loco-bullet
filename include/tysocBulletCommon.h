
#pragma once

// main bullet API
#include <btBulletDynamicsCommon.h>

// some helper functions
#include <map>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <iostream>

#ifndef TYSOCBULLET_RESOURCES_PATH
    #define TYSOCBULLET_RESOURCES_PATH "../res"
#endif

namespace tysocBullet
{

    void createBtVec3( float* srcVec, btVector3& outVec );
    void getVec3Array( const btVector3& srcVec, float* outVec );

    void createBtMat3( float* srcMat, btMatrix3x3& outMat );
    void getMat3Array( const btMatrix3x3& srcMat, float* outMat );
}