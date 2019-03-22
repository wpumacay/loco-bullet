
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
        auto _btConstraint = _createBtConstraint( bodyPtr->joints, 
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
            std::cout << "LOG> added constraint" << std::endl;
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
        _rigidBodyPtr->setRestitution( 0.5f );
        _rigidBodyPtr->setFriction( bodyPtr->friction.x );

        // make sure the object is going to be simulated by forcing activation
        _rigidBodyPtr->forceActivationState( DISABLE_DEACTIVATION );

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
        else
            _collisionShapePtr->setLocalScaling( utils::toBtVec3( _size ) );

        return _collisionShapePtr;
    }

    btTypedConstraint* TBtBodyWrapper::_createBtConstraint( const std::vector< sandbox::TJoint* >& joints,
                                                            btRigidBody* currentBtBodyPtr,
                                                            btRigidBody* parentBtBodyPtr )
    {
        btTypedConstraint* _btConstraint = NULL;

        if ( joints.size() == 0 )
        {
            // create a fixed joint, as the related body has no dofs
            _btConstraint = _createFixedConstraint( currentBtBodyPtr, parentBtBodyPtr );
        }
        else if ( joints.size() == 1 )
        {
            // check the type, and create one constraint according to the type
            auto _joint = joints[0];
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
            _btConstraint = _createGenericConstraintFromJoints( joints, currentBtBodyPtr, parentBtBodyPtr );
        }

        return _btConstraint;
    }

    btFixedConstraint* TBtBodyWrapper::_createFixedConstraint( btRigidBody* currentBtBodyPtr,
                                                               btRigidBody* parentBtBodyPtr )
    {
        return NULL;
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

            _btConstraint->setLimit( 1.f, -1.f );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyA''' and '''currentBtBodyPtr=bodyB''' ...
            // for hinge-constraint creation

        }

        return _btConstraint;
    }

    btSliderConstraint* TBtBodyWrapper::_createSliderConstraint( sandbox::TJoint* jointPtr,
                                                                 btRigidBody* currentBtBodyPtr,
                                                                 btRigidBody* parentBtBodyPtr )
    {
        return NULL;
    }

    btPoint2PointConstraint* TBtBodyWrapper::_createPoint2PointConstraint( sandbox::TJoint* jointPtr,
                                                                           btRigidBody* currentBtBodyPtr,
                                                                           btRigidBody* parentBtBodyPtr )
    {
        return NULL;
    }

    btGeneric6DofSpring2Constraint* TBtBodyWrapper::_createGenericConstraintFromJoints( const std::vector< sandbox::TJoint* >& joints,
                                                                                        btRigidBody* currentBtBodyPtr,
                                                                                        btRigidBody* parentBtBodyPtr )
    {
        return NULL;
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

}}