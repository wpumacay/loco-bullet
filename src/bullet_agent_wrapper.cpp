
#include <bullet_agent_wrapper.h>

namespace tysoc {
namespace bullet {

    TBtKinTreeAgentWrapper::TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                                    const std::string& workingDir )
        : TKinTreeAgentWrapper( kinTreeAgentPtr, workingDir )
    {

    }

    TBtKinTreeAgentWrapper::~TBtKinTreeAgentWrapper()
    {
        m_btWorldPtr = NULL;
    }

    void TBtKinTreeAgentWrapper::setBtWorld( btDiscreteDynamicsWorld* btWorldPtr )
    {
        m_btWorldPtr = btWorldPtr;
    }

    void TBtKinTreeAgentWrapper::_initializeInternal()
    {
        _createBtResourcesFromKinTree();
    }

    void TBtKinTreeAgentWrapper::_resetInternal()
    {
        if ( m_kinTreeAgentPtr )
            m_kinTreeAgentPtr->reset();
    }

    void TBtKinTreeAgentWrapper::_preStepInternal()
    {
        // @WIP|@MUST: should set actuator ctrls
    }

    void TBtKinTreeAgentWrapper::_postStepInternal()
    {
        auto _kinBodies = m_kinTreeAgentPtr->getKinTreeBodies();
        for ( size_t i = 0; i < _kinBodies.size(); i++ )
        {
            if ( m_btBodies.find( _kinBodies[i]->name ) == m_btBodies.end() )
                continue;

            // extract the world transform from the simulated body
            auto _btRigidBodyPtr = m_btBodies[ _kinBodies[i]->name ];
            auto _rbTransform = _btRigidBodyPtr->getWorldTransform();

            // set this transform as the worldtransform of the wrapped body
            _kinBodies[i]->worldTransform = utils::fromBtTransform( _rbTransform );
        }
    }

    void TBtKinTreeAgentWrapper::_createBtResourcesFromKinTree()
    {
        if ( !m_kinTreeAgentPtr )
            return;

        auto _rootBodyPtr = m_kinTreeAgentPtr->getRootBody();
        _createBtResourcesFromBodyNode( _rootBodyPtr, NULL );
    }

    void TBtKinTreeAgentWrapper::_createBtResourcesFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                 btRigidBody* parentBtBodyPtr )
    {
        // construct a Bullet body out of the kintree body data
        auto _btBodyPtr = _createBtRigidBody( kinTreeBodyPtr );
        // construct a Bullet constraint out of the kintree body data (dofs inside)
        auto _btConstraint = _createBtConstraint( kinTreeBodyPtr, 
                                                  _btBodyPtr, 
                                                  parentBtBodyPtr );

        if ( _btBodyPtr )
        {
            m_btBodies[ kinTreeBodyPtr->name ] = _btBodyPtr;
            m_btWorldPtr->addRigidBody( _btBodyPtr );
        }

        if ( _btConstraint )
        {
            m_btConstraints[ kinTreeBodyPtr->name ] = _btConstraint;
            m_btWorldPtr->addConstraint( _btConstraint, 
                                         true ); // disable collision between linked bodies
        }

        if ( _btBodyPtr )
        {
            for ( size_t i = 0; i < kinTreeBodyPtr->childBodies.size(); i++ )
                _createBtResourcesFromBodyNode( kinTreeBodyPtr->childBodies[i], _btBodyPtr );
        }
    }

    btRigidBody* TBtKinTreeAgentWrapper::_createBtRigidBody( agent::TKinTreeBody* kinTreeBodyPtr )
    {
        // Variables to store inertial props calculations
        btTransform _tfInertialLocalFrame;
        btVector3 _inertiaDiag;
        btScalar _inertiaMass;

        // compute the inertial properties from the data given by the user.
        bool _hasInertialProps = _computeInertialProperties( kinTreeBodyPtr, 
                                                             _tfInertialLocalFrame, 
                                                             _inertiaDiag, 
                                                             _inertiaMass );

        // create a collision shape out of all collision in the body
        auto _collisionShapePtr = _createBtCollisionShape( kinTreeBodyPtr,
                                                           _hasInertialProps,
                                                           _tfInertialLocalFrame );

        if ( !_collisionShapePtr )
            return NULL;

        // create initial pos/rot from kintree body
        auto _rbTransform = utils::toBtTransform( kinTreeBodyPtr->worldTransform );
        auto _rbMotionState = new btDefaultMotionState( _rbTransform );

        // in case no inertia props computed (non given by user), then ...
        // compute default props from collision shape. Mass is initialize ...
        // in the previous method from the inertial props. If non given, then ...
        // compute mass from shape volume and default density.
        if ( !_hasInertialProps )
        {
            if ( _inertiaMass == 0.0f )
            {
                // @TODO: Change volume computation to a more accurate calculation ...
                // using the actual shapes instead of the overall AABB
                btVector3 _aabbMin;
                btVector3 _aabbMax;

                _collisionShapePtr->getAabb( _rbTransform, _aabbMin, _aabbMax );

                auto _vmin2max = _aabbMax - _aabbMin;
                auto _dx = _vmin2max.dot( _rbTransform.getBasis().getColumn( 0 ) );
                auto _dy = _vmin2max.dot( _rbTransform.getBasis().getColumn( 1 ) );
                auto _dz = _vmin2max.dot( _rbTransform.getBasis().getColumn( 2 ) );

                auto _volume = btFabs( _dx * _dy * _dz );

                _inertiaMass = TYSOC_DEFAULT_DENSITY * _volume;
            }

            _collisionShapePtr->calculateLocalInertia( _inertiaMass, _inertiaDiag );
        }

        // assemble all previous info into the struct used for rigid body creation
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo(
                                                            _inertiaMass,
                                                            _rbMotionState,
                                                            _collisionShapePtr,
                                                            _inertiaDiag );

        // construct the rigid body for this kintree body object
        auto _rigidBodyPtr = new btRigidBody( _rbConstructionInfo );

        // @DEBUG: just to check some issues with the ant
        _rigidBodyPtr->setRestitution( 0.1 );
        _rigidBodyPtr->setFriction( 1.0 );

        // make sure the object is going to be simulated by forcing activation
        _rigidBodyPtr->forceActivationState( DISABLE_DEACTIVATION );

        // // @DEBUG: set initial velocity to see if everything is simulating (no torques jet)
        // _rigidBodyPtr->setLinearVelocity( utils::toBtVec3( { 0.2, 0.2, 0.2 } ) );

        return _rigidBodyPtr;
    }

    bool TBtKinTreeAgentWrapper::_computeInertialProperties( agent::TKinTreeBody* kinTreeBodyPtr,
                                                             btTransform& inertialFrame,
                                                             btVector3& inertiaDiag,
                                                             btScalar& inertiaMass )
    {
        inertialFrame.setIdentity();
        inertiaMass = 0.0f;

        // @WIP

        return false;
    }

    btCollisionShape* TBtKinTreeAgentWrapper::_createBtCollisionShape( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                       bool compensateLocalInertialFrame,
                                                                       const btTransform& inertialLocalFrame )
    {
        btCollisionShape* _btCollisionShape = NULL;

        auto _kinTreeCollisions = kinTreeBodyPtr->childCollisions;

        /* @DOCS */
        // create a single collision shape out of all collisions attached to the body
        auto _compoundCollisionShape = new btCompoundShape();
        for ( size_t i = 0; i < _kinTreeCollisions.size(); i++ )
        {
            auto _collisionShape = _createBtCollisionShapeSingle( _kinTreeCollisions[i] );
            if ( !_collisionShape )
                continue;

            if ( compensateLocalInertialFrame )
            {
                /* @DOCS */
                // construct the transform from the collision frame to the ...
                // inertial frame, which is not aligned to the body frame, as ...
                // the inertial frame had to be compensated to be axis-aligned ...
                // with the principal axes of inertia (given by user inertia props)
                auto _tfCollisionToBody = utils::toBtTransform( _kinTreeCollisions[i]->relTransform );
                auto _tfCollisionToInertial = inertialLocalFrame.inverse() * _tfCollisionToBody;
                // and add the shape to the compound according to this transform
                _compoundCollisionShape->addChildShape( _tfCollisionToInertial,
                                                        _collisionShape );
            }
            else
            {
                /* @DOCS */
                // construct the transform from the collision frame to the ...
                // inertial frame, which in this case is the same as the body ...
                // frame, so the transform is the same
                auto _tfCollisionToInertial = utils::toBtTransform( _kinTreeCollisions[i]->relTransform );
                // and add the shape to the compound according to this transform
                _compoundCollisionShape->addChildShape( _tfCollisionToInertial,
                                                        _collisionShape );
            }
        }

        return _compoundCollisionShape;
    }

    btCollisionShape* TBtKinTreeAgentWrapper::_createBtCollisionShapeSingle( agent::TKinTreeCollision* kinTreeCollisionPtr )
    {
        if ( !kinTreeCollisionPtr )
            return NULL;

        btCollisionShape* _collisionShapePtr = NULL;

        auto _type = kinTreeCollisionPtr->geometry.type;
        auto _size = kinTreeCollisionPtr->geometry.size;

        if ( _type == "box" )
        {
            _collisionShapePtr = new btBoxShape( btVector3( 0.5, 0.5, 0.5 ) );
        }
        else if ( _type == "sphere" )
        {
            _collisionShapePtr = new btSphereShape( 1.0 );
        }
        else if ( _type == "capsule" )
        {
            _size = { _size.z, _size.x, _size.y };
            _collisionShapePtr = new btCapsuleShapeZ( 1.0, 1.0 );
        }
        else if ( _type == "cylinder" )
        {
            _size = { _size.x, _size.x, _size.y };
            _collisionShapePtr = new btCylinderShapeZ( btVector3( 1.0, 1.0, 1.0 ) );
        }

        if ( !_collisionShapePtr )
            std::cout << "ERROR> could not create shape of type: " << _type << std::endl;
        else
            _collisionShapePtr->setLocalScaling( utils::toBtVec3( _size ) );

        return _collisionShapePtr;
    }

    btTypedConstraint* TBtKinTreeAgentWrapper::_createBtConstraint( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                    btRigidBody* currentBtBodyPtr,
                                                                    btRigidBody* parentBtBodyPtr )
    {
        btTypedConstraint* _btConstraint = NULL;

        if ( kinTreeBodyPtr->childJoints.size() == 0 )
        {
            // create a fixed joint, as the related body has no dofs
            _btConstraint = _createFixedConstraint( kinTreeBodyPtr, currentBtBodyPtr, parentBtBodyPtr );
        }
        else if ( kinTreeBodyPtr->childJoints.size() == 1 )
        {
            // check the type, and create one constraint according to the type
            auto _kinTreeJointPtr = kinTreeBodyPtr->childJoints[0];
            auto _type = _kinTreeJointPtr->type;

            if ( !_kinTreeJointPtr )
            {
                std::cout << "ERROR> the joint for body: " << kinTreeBodyPtr->name << " is NULL" << std::endl;
                return NULL;
            }

            if ( _type == "world" || _type == "fixed" )
            {
                // attached to world, so fixed
                if ( parentBtBodyPtr )
                    std::cout << "WARNING> body: " << kinTreeBodyPtr->name 
                              << " is attached to world, but it has a parent???" << std::endl;

                _btConstraint = _createFixedConstraint( kinTreeBodyPtr, currentBtBodyPtr, NULL );
            }
            else if ( _type == "free" || _type == "floating" )
            {
                // no constraint should be created
                _btConstraint = NULL;
            }
            else if ( _type == "hinge" || _type == "continuous" || _type == "revolute" )
            {
                _btConstraint = _createHingeConstraint( _kinTreeJointPtr, currentBtBodyPtr, parentBtBodyPtr );
            }
            else if ( _type == "slide" || _type == "prismatic" )
            {
                _btConstraint = _createSliderConstraint( _kinTreeJointPtr, currentBtBodyPtr, parentBtBodyPtr );
            }
            else if ( _type == "ball" || _type == "spheric" || _type == "spherical" )
            {
                _btConstraint = _createPoint2PointConstraint( _kinTreeJointPtr, currentBtBodyPtr, parentBtBodyPtr );
            }
            else
            {
                std::cout << "ERROR> joint type: " << _type << " not supported" << std::endl;
            }
        }
        else
        {
            // create a single generic constraint with limits|dofs configured appropriately
            _btConstraint = _createGenericConstraintFromJoints( kinTreeBodyPtr, currentBtBodyPtr, parentBtBodyPtr );
        }

        return _btConstraint;
    }

    btGeneric6DofConstraint* TBtKinTreeAgentWrapper::_createFixedConstraint( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                             btRigidBody* currentBtBodyPtr,
                                                                             btRigidBody* parentBtBodyPtr )
    {
        btGeneric6DofConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyB''' for fixed-constraint creation
            _btConstraint = new btGeneric6DofConstraint( *currentBtBodyPtr,
                                                         utils::toBtTransform( tysoc::TMat4() ),
                                                         true );

            // Lock all 6 axes
            _btConstraint->setLimit( 0, 0, 0 );
            _btConstraint->setLimit( 1, 0, 0 );
            _btConstraint->setLimit( 2, 0, 0 );
            _btConstraint->setLimit( 3, 0, 0 );
            _btConstraint->setLimit( 4, 0, 0 );
            _btConstraint->setLimit( 5, 0, 0 );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for fixed-constraint creation

            auto _frameInA = utils::toBtTransform( tysoc::TMat4() ); // Identity, fixed in own body frame
            auto _frameInB = utils::toBtTransform( kinTreeBodyPtr->relTransform );// Relative transform of body to parent

            _btConstraint = new btGeneric6DofConstraint( *currentBtBodyPtr,
                                                         *parentBtBodyPtr,
                                                         _frameInA,
                                                         _frameInB,
                                                         true );

            // Lock all 6 axes
            _btConstraint->setLimit( 0, 0, 0 );
            _btConstraint->setLimit( 1, 0, 0 );
            _btConstraint->setLimit( 2, 0, 0 );
            _btConstraint->setLimit( 3, 0, 0 );
            _btConstraint->setLimit( 4, 0, 0 );
            _btConstraint->setLimit( 5, 0, 0 );
        }

        return _btConstraint;
    }

    btHingeConstraint* TBtKinTreeAgentWrapper::_createHingeConstraint( agent::TKinTreeJoint* kinTreeJointPtr,
                                                                       btRigidBody* currentBtBodyPtr,
                                                                       btRigidBody* parentBtBodyPtr )
    {
        btHingeConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyA''' for hinge-constraint creation
            _btConstraint = new btHingeConstraint( *currentBtBodyPtr, 
                                                   utils::toBtVec3( kinTreeJointPtr->relTransform.getPosition() ),
                                                   utils::toBtVec3( kinTreeJointPtr->axis ) );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for hinge-constraint creation

            auto _kinTreeBodyPtr = kinTreeJointPtr->parentBodyPtr;
            if ( !_kinTreeBodyPtr )
            {
                std::cout << "WARNING> It seems you left a joint: " 
                          << kinTreeJointPtr->name << " without its parent" << std::endl;
                return NULL;
            }

            auto _axisInA = kinTreeJointPtr->axis;
            auto _pivotInA = kinTreeJointPtr->relTransform.getPosition();

            auto _axisInB = _kinTreeBodyPtr->relTransform.getRotation() * _axisInA;
            auto _pivotInB = _kinTreeBodyPtr->relTransform.getRotation() * _pivotInA +
                             _kinTreeBodyPtr->relTransform.getPosition();

            _btConstraint = new btHingeConstraint( *currentBtBodyPtr, 
                                                   *parentBtBodyPtr,
                                                   utils::toBtVec3( _pivotInA ), 
                                                   utils::toBtVec3( _pivotInB ),
                                                   utils::toBtVec3( _axisInA ), 
                                                   utils::toBtVec3( _axisInB ) );
        }

        _btConstraint->setLimit( kinTreeJointPtr->lowerLimit * TYSOC_PI / 180.0, 
                                 kinTreeJointPtr->upperLimit * TYSOC_PI / 180.0 );

        return _btConstraint;
    }

    btSliderConstraint* TBtKinTreeAgentWrapper::_createSliderConstraint( agent::TKinTreeJoint* kinTreeJointPtr,
                                                                         btRigidBody* currentBtBodyPtr,
                                                                         btRigidBody* parentBtBodyPtr )
    {
        btSliderConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyB''' for slider constraint creation
            // By looking at btSliderConstraint::calculateTransforms (line 153) ...
            // it seems that the transform implicitly defines the sliding axis

            auto _transform = tysoc::TMat4();
            _transform.setPosition( kinTreeJointPtr->relTransform.getPosition() );
            _transform.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, kinTreeJointPtr->axis ) );

            _btConstraint = new btSliderConstraint( *currentBtBodyPtr,
                                                    utils::toBtTransform( _transform ),
                                                    true );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for slider-constraint creation

            auto _kinTreeBodyPtr = kinTreeJointPtr->parentBodyPtr;
            if ( !_kinTreeBodyPtr )
            {
                std::cout << "WARNING> It seems you left a joint: " 
                          << kinTreeJointPtr->name << " without its parent" << std::endl;
                return NULL;
            }

            auto _axisInA = kinTreeJointPtr->axis;
            auto _pivotInA = kinTreeJointPtr->relTransform.getPosition();
            auto _frameInA = tysoc::TMat4();
            _frameInA.setPosition( _pivotInA );
            _frameInA.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, _axisInA ) );

            auto _axisInB = _kinTreeBodyPtr->relTransform.getRotation() * _axisInA;
            auto _pivotInB = _kinTreeBodyPtr->relTransform.getRotation() * _pivotInA +
                             _kinTreeBodyPtr->relTransform.getPosition();
            auto _frameInB = tysoc::TMat4();
            _frameInB.setPosition( _pivotInB );
            _frameInB.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, _axisInB ) );

            _btConstraint = new btSliderConstraint( *currentBtBodyPtr,
                                                    *parentBtBodyPtr,
                                                    utils::toBtTransform( _frameInA ),
                                                    utils::toBtTransform( _frameInB ), 
                                                    true );
        }

        _btConstraint->setLowerLinLimit( kinTreeJointPtr->lowerLimit );
        _btConstraint->setUpperLinLimit( kinTreeJointPtr->upperLimit );
        _btConstraint->setLowerAngLimit( 0 );
        _btConstraint->setUpperAngLimit( 0 );

        return _btConstraint;
    }

    btPoint2PointConstraint* TBtKinTreeAgentWrapper::_createPoint2PointConstraint( agent::TKinTreeJoint* kinTreeJointPtr,
                                                                                   btRigidBody* currentBtBodyPtr,
                                                                                   btRigidBody* parentBtBodyPtr )
    {
        // @TODO: Replace with btGeneric6DofSpring2Constraint, as in :
        // https://github.com/xbpeng/DeepLoco/blob/c4e2db93fefcd49ee7a2481918e2f7db1c1da733/sim/World.cpp#L755

        btPoint2PointConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyA''' for hinge-constraint creation
            _btConstraint = new btPoint2PointConstraint( *currentBtBodyPtr, 
                                                         utils::toBtVec3( kinTreeJointPtr->relTransform.getPosition() ) );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for hinge-constraint creation

            auto _kinTreeBodyPtr = kinTreeJointPtr->parentBodyPtr;
            if ( !_kinTreeBodyPtr )
            {
                std::cout << "WARNING> It seems you left a joint: " 
                          << kinTreeJointPtr->name << " without its parent" << std::endl;
                return NULL;
            }

            auto _pivotInA = kinTreeJointPtr->relTransform.getPosition();
            auto _pivotInB = _kinTreeBodyPtr->relTransform.getRotation() * _pivotInA +
                             _kinTreeBodyPtr->relTransform.getPosition();

            _btConstraint = new btPoint2PointConstraint( *currentBtBodyPtr, 
                                                         *parentBtBodyPtr,
                                                         utils::toBtVec3( _pivotInA ), 
                                                         utils::toBtVec3( _pivotInB ) );
        }

        return _btConstraint;
    }

    btGeneric6DofConstraint* TBtKinTreeAgentWrapper::_createGenericConstraintFromJoints( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                                         btRigidBody* currentBtBodyPtr,
                                                                                         btRigidBody* parentBtBodyPtr )
    {
        btGeneric6DofConstraint* _btConstraint = NULL;

        // Grab only the DOFs whose axes map to basis vectors, only ...
        // those that consist of 'slide' or 'hinge' joints, and max 6 joints
        auto _joints = kinTreeBodyPtr->childJoints;
        std::vector< int > _dofIndices;
        std::vector< tysoc::TVec2 > _dofLimits;
        for ( size_t q = 0; q < _joints.size(); q++ )
        {
            if ( q > 5 )
            {
                std::cout << "WARNING> trying to configure more than 6 joints " 
                          << "for generic constraint" << std::endl;
                break;
            }

            if ( _joints[q]->type != "hinge" && _joints[q]->type != "slide" )
            {
                std::cout << "WARNING> tried to configure " << _joints[q]->type
                          << " for generic joint, which is not supported (only slide and hinge)" << std::endl;
                continue;
            }

            // map joint AXIS to DOF index
            if ( _joints[q]->type == "slide" )
            {
                if ( std::fabs( _joints[q]->axis.x ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "lin-x" << std::endl;
                    _dofIndices.push_back( 0 );
                    _dofLimits.push_back( { _joints[q]->lowerLimit, _joints[q]->upperLimit } );
                }
                else if ( std::fabs( _joints[q]->axis.y ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "lin-y" << std::endl;
                    _dofIndices.push_back( 1 );
                    _dofLimits.push_back( { _joints[q]->lowerLimit, _joints[q]->upperLimit } );
                }
                else if ( std::fabs( _joints[q]->axis.z ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "lin-z" << std::endl;
                    _dofIndices.push_back( 2 );
                    _dofLimits.push_back( { _joints[q]->lowerLimit, _joints[q]->upperLimit } );
                }
            }
            else
            {
                if ( std::fabs( _joints[q]->axis.x ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "ang-x" << std::endl;
                    _dofIndices.push_back( 3 );
                    _dofLimits.push_back( { _joints[q]->lowerLimit, _joints[q]->upperLimit } );
                }
                else if ( std::fabs( _joints[q]->axis.y ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "ang-y" << std::endl;
                    _dofIndices.push_back( 4 );
                    _dofLimits.push_back( { _joints[q]->lowerLimit, _joints[q]->upperLimit } );
                }
                else if ( std::fabs( _joints[q]->axis.z ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "ang-z" << std::endl;
                    _dofIndices.push_back( 5 );
                    _dofLimits.push_back( { _joints[q]->lowerLimit, _joints[q]->upperLimit } );
                }
            }
        }

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyB''' for fixed-constraint creation
            _btConstraint = new btGeneric6DofConstraint( *currentBtBodyPtr,
                                                         utils::toBtTransform( tysoc::TMat4() ),
                                                         true );

            // Lock all 6 axes
            _btConstraint->setLimit( 0, 0, 0 );
            _btConstraint->setLimit( 1, 0, 0 );
            _btConstraint->setLimit( 2, 0, 0 );
            _btConstraint->setLimit( 3, 0, 0 );
            _btConstraint->setLimit( 4, 0, 0 );
            _btConstraint->setLimit( 5, 0, 0 );

            // Free only the axes that have DOFs
            for ( size_t q = 0; q < _dofIndices.size(); q++ )
            {
                // std::cout << "LOG> Setting dof: " << _dofIndices[q] << std::endl;
                _btConstraint->setLimit( _dofIndices[q], _dofLimits[q].x, _dofLimits[q].y );
            }
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for fixed-constraint creation

            auto _frameInA = utils::toBtTransform( tysoc::TMat4() ); // Identity, fixed in own body frame
            auto _frameInB = utils::toBtTransform( kinTreeBodyPtr->relTransform );// Relative transform of body to parent

            _btConstraint = new btGeneric6DofConstraint( *currentBtBodyPtr,
                                                         *parentBtBodyPtr,
                                                         _frameInA,
                                                         _frameInB,
                                                         true );

            // Lock all 6 axes
            _btConstraint->setLimit( 0, 0, 0 );
            _btConstraint->setLimit( 1, 0, 0 );
            _btConstraint->setLimit( 2, 0, 0 );
            _btConstraint->setLimit( 3, 0, 0 );
            _btConstraint->setLimit( 4, 0, 0 );
            _btConstraint->setLimit( 5, 0, 0 );

            // Free only the axes that have DOFs
            for ( size_t q = 0; q < _dofIndices.size(); q++ )
            {
                // std::cout << "LOG> Setting dof: " << _dofIndices[q] << std::endl;
                _btConstraint->setLimit( _dofIndices[q], _dofLimits[q].x, _dofLimits[q].y );
            }
        }

        return _btConstraint;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromAbstract( agent::TAgentKinTree* kinTreeAgentPtr,
                                                               const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromFile( const std::string& name,
                                                           const std::string& filename,
                                                           const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromId( const std::string& name,
                                                         const std::string& format,
                                                         const std::string& id,
                                                         const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }


}}