
#pragma once

// main bullet API
#include <btBulletDynamicsCommon.h>

// some helper functions
#include <map>
#include <vector>
#include <queue>
#include <cmath>
#include <random>

#ifndef TYSOCBULLET_RESOURCES_PATH
    #define TYSOCBULLET_RESOURCES_PATH "../res"
#endif

namespace tysocBullet
{

    // @TODO: Here there be dragons

    void createBtMat3( float* srcMat, btMatrix3x3& outMat );
    void getBtMat3( const btMatrix3x3& srcMat, float* outMat );
}