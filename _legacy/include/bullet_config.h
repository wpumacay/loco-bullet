
#pragma once

#include <string>

#ifndef TYSOC_BACKEND_PHYSICS_BULLET
    #define TYSOC_BACKEND_PHYSICS_BULLET "../libtysocPhysicsBullet.so"
#endif

namespace tysoc {
namespace config {

    namespace physics
    {
        const std::string BULLET = std::string( TYSOC_BACKEND_PHYSICS_BULLET );
    }

}}