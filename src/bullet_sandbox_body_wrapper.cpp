
#include <bullet_sandbox_body_wrapper.h>


namespace tysoc {
namespace bullet {


    TBtBodyWrapper::TBtBodyWrapper( sandbox::TBody* bodyPtr,
                                    const std::string& workingDir )
        : TBodyWrapper( bodyPtr, workingDir )
    {
        m_btWorldPtr = NULL;
        m_btCollisionShapePtr = NULL;
        m_btRigidBodyPtr = NULL;
    }

    TBtBodyWrapper::~TBtBodyWrapper()
    {
        // @WIP: see method "exitPhysics" in bullet example "CommonRigidBodyBase.h"

        m_btWorldPtr = NULL;
    }

    void TBtBodyWrapper::setBtWorld( btDiscreteDynamicsWorld* btWorldPtr )
    {
        m_btWorldPtr = btWorldPtr;
    }

    void TBtBodyWrapper::_initializeInternal()
    {
        _createBtResourcesFromBody( m_bodyPtr, NULL );

        if ( ( m_btRigidBodyPtr == NULL ) || ( m_btCollisionShapePtr == NULL ) )
            std::cout << "WARNING> did not set root body properly" << std::endl;
    }

    void TBtBodyWrapper::_initializeWorldTransformsInternal()
    {
        if ( !m_bodyPtr )
            return;

        _initBodyRecursively( m_bodyPtr );
    }

    void TBtBodyWrapper::_resetInternal()
    {
        if ( !m_bodyPtr )
            return;

        if ( !m_btRigidBodyPtr )
            return;

        m_bodyPtr->worldTransform.setPosition( m_posStart );
        m_bodyPtr->worldTransform.setRotation( m_rotStart );

        m_btRigidBodyPtr->setLinearVelocity( btVector3( 0, 0, 0 ) );
        m_btRigidBodyPtr->setAngularVelocity( btVector3( 0, 0, 0 ) );
        m_btRigidBodyPtr->setWorldTransform( utils::toBtTransform( m_bodyPtr->worldTransform ) );
        m_btRigidBodyPtr->forceActivationState( DISABLE_DEACTIVATION );
    }

    void TBtBodyWrapper::_preStepInternal()
    {
        // Nothing to do here
    }

    void TBtBodyWrapper::_postStepInternal()
    {
        if ( !m_bodyPtr )
            return;

        _updateBodyRecursively( m_bodyPtr );
    }

    void TBtBodyWrapper::_changePositionInternal()
    {
        if ( !m_bodyPtr )
            return;

        if ( !m_btRigidBodyPtr )
            return;

        m_btRigidBodyPtr->setWorldTransform( utils::toBtTransform( m_bodyPtr->worldTransform ) );
    }

    void TBtBodyWrapper::_changeRotationInternal()
    {
        if ( !m_bodyPtr )
            return;

        if ( !m_btRigidBodyPtr )
            return;

        m_btRigidBodyPtr->setWorldTransform( utils::toBtTransform( m_bodyPtr->worldTransform ) );
    }

    void TBtBodyWrapper::_changeSizeInternal()
    {
        if ( !m_bodyPtr )
            return;

        if ( !m_btCollisionShapePtr )
            return;

        if ( !m_btRigidBodyPtr )
            return;

        // change shape dimensions
        m_btCollisionShapePtr->setLocalScaling( utils::toBtVec3( m_bodyPtr->size ) );

        // recalculate the mass and inertia props
        btScalar _rbMass = m_bodyPtr->mass;
        btVector3 _rbInertia = btVector3( 0, 0, 0 );
        if ( _rbMass != 0.0f )
            m_btCollisionShapePtr->calculateLocalInertia( _rbMass, _rbInertia );

        // and then change the mass props of the rigid body
        m_btRigidBodyPtr->setMassProps( _rbMass, _rbInertia );
    }

    void TBtBodyWrapper::_createBtResourcesFromBody( sandbox::TBody* bodyPtr, 
                                                     btRigidBody* parentBtBodyPtr )
    {
        if ( !m_btWorldPtr )
            return;

        if ( !bodyPtr )
            return;

        auto _btBodyPtr = _createBtRigidBody( bodyPtr );
        auto _btConstraint = _createBtConstraint( bodyPtr, 
                                                  _btBodyPtr,
                                                  parentBtBodyPtr );

        if ( _btBodyPtr )
        {
            m_btBodies[ bodyPtr->name ] = _btBodyPtr;
            m_btWorldPtr->addRigidBody( _btBodyPtr );

            // check if root (root of chain, or single body)
            if ( !bodyPtr->parentBodyPtr )
                m_btRigidBodyPtr = _btBodyPtr;
        }

        if ( _btConstraint )
        {
            m_btConstraints[ bodyPtr->name ] = _btConstraint;
            m_btWorldPtr->addConstraint( _btConstraint, CONSTRAINT_DISABLE_COLLISION_CONNECTED_BODIES );
        }

        if ( _btBodyPtr )
        {
            for ( size_t q = 0; q < bodyPtr->bodies.size(); q++ )
                _createBtResourcesFromBody( bodyPtr->bodies[q], _btBodyPtr );
        }
    }

    btRigidBody* TBtBodyWrapper::_createBtRigidBody( sandbox::TBody* bodyPtr )
    {
        auto _collisionShapePtr = _createBtCollisionShape( bodyPtr );

        if ( !_collisionShapePtr )
            return NULL;

        if ( !bodyPtr->parentBodyPtr )
            m_btCollisionShapePtr = _collisionShapePtr;

        // create initial pos/rot from wrapped body
        auto _rbTransform = utils::toBtTransform( bodyPtr->worldTransform );
        auto _rbMotionState = new btDefaultMotionState( _rbTransform );

        // initialize inertial properties
        btScalar _rbMass = bodyPtr->mass;
        btVector3 _rbInertia = btVector3( 0, 0, 0 );
        if ( _rbMass != 0.0f )
            _collisionShapePtr->calculateLocalInertia( _rbMass, _rbInertia );

        // assemble the struct used for rigid body creation
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo(
                                                            _rbMass,
                                                            _rbMotionState,
                                                            _collisionShapePtr,
                                                            _rbInertia );

        // construct the rigid body with the previous struct
        auto _rigidBodyPtr = new btRigidBody( _rbConstructionInfo );

        // grab some extra info from the wrapped body
        _rigidBodyPtr->setRestitution( 0.1 );
        _rigidBodyPtr->setFriction( bodyPtr->friction.x );

        // make sure the object is going to be simulated by forcing activation
        _rigidBodyPtr->forceActivationState( DISABLE_DEACTIVATION );

        // and set the initial velocity
        _rigidBodyPtr->setLinearVelocity( utils::toBtVec3( bodyPtr->vel ) );

        return _rigidBodyPtr;
    }

    btCollisionShape* TBtBodyWrapper::_createBtCollisionShape( sandbox::TBody* bodyPtr )
    {
        if ( !bodyPtr )
            return NULL;

        btCollisionShape* _collisionShapePtr = NULL;

        auto _type = bodyPtr->type;
        auto _size = bodyPtr->size;

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
        else if ( _type == "plane" )
        {
            _collisionShapePtr = new btStaticPlaneShape( btVector3( 0, 0, 1 ), 0 );
        }

        if ( !_collisionShapePtr )
            std::cout << "ERROR> could not create shape of type: " << _type << std::endl;
        else if ( _type != "plane" )
            _collisionShapePtr->setLocalScaling( utils::toBtVec3( _size ) );

        return _collisionShapePtr;
    }

    btTypedConstraint* TBtBodyWrapper::_createBtConstraint( sandbox::TBody* bodyPtr,
                                                            btRigidBody* currentBtBodyPtr,
                                                            btRigidBody* parentBtBodyPtr )
    {
        btTypedConstraint* _btConstraint = NULL;

        if ( bodyPtr->joints.size() == 0 )
        {
            // create a fixed joint, as the related body has no dofs
            _btConstraint = _createFixedConstraint( bodyPtr, currentBtBodyPtr, parentBtBodyPtr );
        }
        else if ( bodyPtr->joints.size() == 1 )
        {
            // check the type, and create one constraint according to the type
            auto _joint = bodyPtr->joints[0];
            auto _type = _joint->type;

            if ( _type == "free" )
            {
                // no constraint should be created
                _btConstraint = NULL;
            }
            else if ( _type == "hinge" )
            {
                _btConstraint = _createHingeConstraint( _joint, currentBtBodyPtr, parentBtBodyPtr );
            }
            else if ( _type == "slide" )
            {
                _btConstraint = _createSliderConstraint( _joint, currentBtBodyPtr, parentBtBodyPtr );
            }
            else if ( _type == "ball" )
            {
                _btConstraint = _createPoint2PointConstraint( _joint, currentBtBodyPtr, parentBtBodyPtr );
            }
            else
            {
                std::cout << "ERROR> joint type: " << _type << " not supported" << std::endl;
            }
        }
        else
        {
            // create a single generic constraint with limits|dofs configured appropriately
            _btConstraint = _createGenericConstraintFromJoints( bodyPtr, currentBtBodyPtr, parentBtBodyPtr );
        }

        return _btConstraint;
    }

    btGeneric6DofConstraint* TBtBodyWrapper::_createFixedConstraint( sandbox::TBody* bodyPtr,
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
            auto _frameInB = utils::toBtTransform( bodyPtr->relTransform );// Relative transform of body to parent

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

    btHingeConstraint* TBtBodyWrapper::_createHingeConstraint( sandbox::TJoint* jointPtr,
                                                               btRigidBody* currentBtBodyPtr,
                                                               btRigidBody* parentBtBodyPtr )
    {
        btHingeConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyA''' for hinge-constraint creation
            _btConstraint = new btHingeConstraint( *currentBtBodyPtr, 
                                                   utils::toBtVec3( jointPtr->relTransform.getPosition() ),
                                                   utils::toBtVec3( jointPtr->axis ) );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for hinge-constraint creation

            auto _bodyPtr = jointPtr->parentBodyPtr;
            if ( !_bodyPtr )
            {
                std::cout << "WARNING> It seems you left a joint: " 
                          << jointPtr->name << " without its parent" << std::endl;
                return NULL;
            }

            auto _axisInA = jointPtr->axis;
            auto _pivotInA = jointPtr->relTransform.getPosition();

            auto _axisInB = _bodyPtr->relTransform.getRotation() * _axisInA;
            auto _pivotInB = _bodyPtr->relTransform.getRotation() * _pivotInA +
                             _bodyPtr->relTransform.getPosition();

            _btConstraint = new btHingeConstraint( *currentBtBodyPtr, 
                                                   *parentBtBodyPtr,
                                                   utils::toBtVec3( _pivotInA ), 
                                                   utils::toBtVec3( _pivotInB ),
                                                   utils::toBtVec3( _axisInA ), 
                                                   utils::toBtVec3( _axisInB ) );
        }

        _btConstraint->setLimit( jointPtr->limits.x, jointPtr->limits.y );

        return _btConstraint;
    }

    btSliderConstraint* TBtBodyWrapper::_createSliderConstraint( sandbox::TJoint* jointPtr,
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
            _transform.setPosition( jointPtr->relTransform.getPosition() );
            _transform.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, jointPtr->axis ) );

            _btConstraint = new btSliderConstraint( *currentBtBodyPtr,
                                                    utils::toBtTransform( _transform ),
                                                    true );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for slider-constraint creation

            auto _bodyPtr = jointPtr->parentBodyPtr;
            if ( !_bodyPtr )
            {
                std::cout << "WARNING> It seems you left a joint: " 
                          << jointPtr->name << " without its parent" << std::endl;
                return NULL;
            }

            auto _axisInA = jointPtr->axis;
            auto _pivotInA = jointPtr->relTransform.getPosition();
            auto _frameInA = tysoc::TMat4();
            _frameInA.setPosition( _pivotInA );
            _frameInA.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, _axisInA ) );

            auto _axisInB = _bodyPtr->relTransform.getRotation() * _axisInA;
            auto _pivotInB = _bodyPtr->relTransform.getRotation() * _pivotInA +
                             _bodyPtr->relTransform.getPosition();
            auto _frameInB = tysoc::TMat4();
            _frameInB.setPosition( _pivotInB );
            _frameInB.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, _axisInB ) );

            _btConstraint = new btSliderConstraint( *currentBtBodyPtr,
                                                    *parentBtBodyPtr,
                                                    utils::toBtTransform( _frameInA ),
                                                    utils::toBtTransform( _frameInB ), 
                                                    true );
        }

        _btConstraint->setLowerLinLimit( jointPtr->limits.x );
        _btConstraint->setUpperLinLimit( jointPtr->limits.y );
        _btConstraint->setLowerAngLimit( 0 );
        _btConstraint->setUpperAngLimit( 0 );

        return _btConstraint;
    }

    btPoint2PointConstraint* TBtBodyWrapper::_createPoint2PointConstraint( sandbox::TJoint* jointPtr,
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
                                                         utils::toBtVec3( jointPtr->relTransform.getPosition() ) );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for hinge-constraint creation

            auto _bodyPtr = jointPtr->parentBodyPtr;
            if ( !_bodyPtr )
            {
                std::cout << "WARNING> It seems you left a joint: " 
                          << jointPtr->name << " without its parent" << std::endl;
                return NULL;
            }

            auto _pivotInA = jointPtr->relTransform.getPosition();
            auto _pivotInB = _bodyPtr->relTransform.getRotation() * _pivotInA +
                             _bodyPtr->relTransform.getPosition();

            _btConstraint = new btPoint2PointConstraint( *currentBtBodyPtr, 
                                                         *parentBtBodyPtr,
                                                         utils::toBtVec3( _pivotInA ), 
                                                         utils::toBtVec3( _pivotInB ) );
        }

        return _btConstraint;
    }

    btGeneric6DofConstraint* TBtBodyWrapper::_createGenericConstraintFromJoints( sandbox::TBody* bodyPtr,
                                                                                 btRigidBody* currentBtBodyPtr,
                                                                                 btRigidBody* parentBtBodyPtr )
    {
        btGeneric6DofConstraint* _btConstraint = NULL;

        // Grab only the DOFs whose axes map to basis vectors, only ...
        // those that consist of 'slide' or 'hinge' joints, and max 6 joints
        auto _joints = bodyPtr->joints;
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
                    std::cout << "LOG> Setting dof: " << "lin-x" << std::endl;
                    _dofIndices.push_back( 0 );
                    _dofLimits.push_back( _joints[q]->limits );
                }
                else if ( std::fabs( _joints[q]->axis.y ) > 0 )
                {
                    std::cout << "LOG> Setting dof: " << "lin-y" << std::endl;
                    _dofIndices.push_back( 1 );
                    _dofLimits.push_back( _joints[q]->limits );
                }
                else if ( std::fabs( _joints[q]->axis.z ) > 0 )
                {
                    std::cout << "LOG> Setting dof: " << "lin-z" << std::endl;
                    _dofIndices.push_back( 2 );
                    _dofLimits.push_back( _joints[q]->limits );
                }
            }
            else
            {
                if ( std::fabs( _joints[q]->axis.x ) > 0 )
                {
                    std::cout << "LOG> Setting dof: " << "ang-x" << std::endl;
                    _dofIndices.push_back( 3 );
                    _dofLimits.push_back( _joints[q]->limits );
                }
                else if ( std::fabs( _joints[q]->axis.y ) > 0 )
                {
                    std::cout << "LOG> Setting dof: " << "ang-y" << std::endl;
                    _dofIndices.push_back( 4 );
                    _dofLimits.push_back( _joints[q]->limits );
                }
                else if ( std::fabs( _joints[q]->axis.z ) > 0 )
                {
                    std::cout << "LOG> Setting dof: " << "ang-z" << std::endl;
                    _dofIndices.push_back( 5 );
                    _dofLimits.push_back( _joints[q]->limits );
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
                std::cout << "LOG> Setting dof: " << _dofIndices[q] << std::endl;
                _btConstraint->setLimit( _dofIndices[q], _dofLimits[q].x, _dofLimits[q].y );
            }
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for fixed-constraint creation

            auto _frameInA = utils::toBtTransform( tysoc::TMat4() ); // Identity, fixed in own body frame
            auto _frameInB = utils::toBtTransform( bodyPtr->relTransform );// Relative transform of body to parent

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
                std::cout << "LOG> Setting dof: " << _dofIndices[q] << std::endl;
                _btConstraint->setLimit( _dofIndices[q], _dofLimits[q].x, _dofLimits[q].y );
            }
        }

        return _btConstraint;
    }

    void TBtBodyWrapper::_updateBodyRecursively( sandbox::TBody* bodyPtr )
    {
        if ( !bodyPtr )
            return;

        if ( m_btBodies.find( bodyPtr->name ) != m_btBodies.end() )
        {
            // extract the world transform of the simulated body
            auto _btRigidBodyPtr = m_btBodies[ bodyPtr->name ];
            auto _rbTransform = _btRigidBodyPtr->getWorldTransform();

            // set it to the world transform of the wrapped body
            bodyPtr->worldTransform = utils::fromBtTransform( _rbTransform );

            // update the child joints
            for ( size_t q = 0; q < bodyPtr->joints.size(); q++ )
            {
                auto _joint = bodyPtr->joints[q];
                _joint->worldTransform = bodyPtr->worldTransform * _joint->relTransform;
            }

            // and the child bodies as well
            for ( size_t q = 0; q < bodyPtr->bodies.size(); q++ )
                _updateBodyRecursively( bodyPtr->bodies[q] );
        }
        else
        {
            std::cout << "WARNING> body with name: " << bodyPtr->name << " not initialized" << std::endl;
        }
    }

    void TBtBodyWrapper::_initBodyRecursively( sandbox::TBody* bodyPtr )
    {
        if ( m_btBodies.find( bodyPtr->name ) != m_btBodies.end() )
        {
            auto _btRigidBodyPtr = m_btBodies[ bodyPtr->name ];
            auto _rbTransform = utils::toBtTransform( bodyPtr->worldTransform );
            
            _btRigidBodyPtr->setWorldTransform( _rbTransform );

            // and the child bodies as well
            for ( size_t q = 0; q < bodyPtr->bodies.size(); q++ )
                _initBodyRecursively( bodyPtr->bodies[q] );
        }
    }

}}