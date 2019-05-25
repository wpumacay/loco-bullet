
#pragma once

// main bullet API
#include <btBulletDynamicsCommon.h>
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

// extra helper functionality from tysocCore
#include <tysoc_common.h>
#include <utils/parsers/mjcf/mjcf.h>
#include <utils/parsers/rlsim/rlsim.h>
#include <utils/parsers/urdf/urdf.h>

// some helper functions
#include <map>
#include <vector>
#include <stack>
#include <queue>
#include <cmath>
#include <random>
#include <iostream>

// and some configurations
#include <bullet_config.h>

namespace tysoc {
namespace bullet {

    // @TODO: Here there be dragons

}}