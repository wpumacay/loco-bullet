#pragma once

#include <loco_common.h>
// Main Bullet-API
#include <btBulletDynamicsCommon.h>
// Heightfield-terrain functionality
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
// Multibody bullet functionality
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodySliderConstraint.h>

namespace loco {
namespace bullet {

    // Conversions from and to bullet-math types and tinymath-math types
    btVector3 vec3_to_bt( const TVec3& vec );
    btMatrix3x3 mat3_to_bt( const TMat3& mat );
    btTransform mat4_to_bt( const TMat4& mat );
    TVec3 vec3_from_bt( const btVector3& vec );
    TMat3 mat3_from_bt( const btMatrix3x3& mat );
    TMat4 mat4_from_bt( const btTransform& mat );

}}