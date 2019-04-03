
#include <bullet_agent_wrapper.h>

namespace tysoc {
namespace bullet {

    TBtKinTreeAgentWrapper::TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                                    const std::string& workingDir )
        : TKinTreeAgentWrapper( kinTreeAgentPtr, workingDir )
    {
        m_rootCompound = NULL;
        m_btWorldPtr = NULL;
    }

    TBtKinTreeAgentWrapper::~TBtKinTreeAgentWrapper()
    {
        m_btWorldPtr = NULL;
        m_rootCompound = NULL;

        // @TODO|@CHECK: Define the proper functionality for the destructor
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
            if ( m_bodyCompounds.find( _kinBodies[i]->name ) == m_bodyCompounds.end() )
                continue;

            // extract the world transform from the simulated body
            auto _compound = m_bodyCompounds[ _kinBodies[i]->name ];
            auto _btStartBody = _compound->btStartBody;
            auto _baseToStartBodyTransform = _compound->baseToStartBodyTransform;
            auto _startBodyWorldTransform = utils::fromBtTransform( _btStartBody->getWorldTransform() );
            auto _baseWorldTransform = _startBodyWorldTransform * _baseToStartBodyTransform;

            // set this transform as the worldTransform of the wrapped body
            _kinBodies[i]->worldTransform = _baseWorldTransform;
        }

        // @DEBUG: draw AABB for all bodies
        if ( !m_simulationPtr )
            return;

        auto _visualizerPtr = m_simulationPtr->getVisualizer();

        if ( !_visualizerPtr )
            return;

        for ( auto it = m_bodyCompounds.begin();
                   it != m_bodyCompounds.end();
                   it++ )
        {
            auto _compound = it->second;

            for ( size_t i = 0; i < _compound->btBodiesInChain.size(); i++ )
            {
                auto _btBodyInChain = _compound->btBodiesInChain[i];

                // Extract the aabb from the bullet body
                btTransform _aabbWorldTransform;
                btVector3 _aabbMin, _aabbMax;

                _btBodyInChain->getAabb( _aabbMin, _aabbMax );

                // auto _diff = _aabbMax - _aabbMin;
                // auto _norm = _diff.length();

                // std::cout << "diag: " << _norm << std::endl;

                // _aabbWorldTransform = _btBodyInChain->getWorldTransform();

                // request the visualizer to draw the aabb
                // _visualizerPtr->drawAABB( utils::fromBtVec3( _aabbMin ),
                //                           utils::fromBtVec3( _aabbMax ),
                //                           utils::fromBtTransform( _aabbWorldTransform ),
                //                           { 0.8, 0.1, 0.1 } );

                // Taken from getAABB.py example from pybullet. It seems that ...
                // the AABB info should be used in a different way to what I thought :(.
                _visualizerPtr->drawLine( { _aabbMin.x(), _aabbMin.y(), _aabbMin.z() },
                                          { _aabbMax.x(), _aabbMin.y(), _aabbMin.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMin.x(), _aabbMin.y(), _aabbMin.z() },
                                          { _aabbMin.x(), _aabbMax.y(), _aabbMin.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMin.x(), _aabbMin.y(), _aabbMin.z() },
                                          { _aabbMin.x(), _aabbMin.y(), _aabbMax.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMin.x(), _aabbMin.y(), _aabbMax.z() },
                                          { _aabbMin.x(), _aabbMax.y(), _aabbMax.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMin.x(), _aabbMin.y(), _aabbMax.z() },
                                          { _aabbMax.x(), _aabbMin.y(), _aabbMax.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMax.x(), _aabbMin.y(), _aabbMin.z() },
                                          { _aabbMax.x(), _aabbMin.y(), _aabbMax.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMax.x(), _aabbMin.y(), _aabbMin.z() },
                                          { _aabbMax.x(), _aabbMax.y(), _aabbMin.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMax.x(), _aabbMax.y(), _aabbMin.z() },
                                          { _aabbMin.x(), _aabbMax.y(), _aabbMin.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMin.x(), _aabbMax.y(), _aabbMin.z() },
                                          { _aabbMin.x(), _aabbMax.y(), _aabbMax.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMax.x(), _aabbMax.y(), _aabbMax.z() },
                                          { _aabbMin.x(), _aabbMax.y(), _aabbMax.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMax.x(), _aabbMax.y(), _aabbMax.z() },
                                          { _aabbMax.x(), _aabbMin.y(), _aabbMax.z() },
                                          { 0.1, 0.1, 0.8 } );

                _visualizerPtr->drawLine( { _aabbMax.x(), _aabbMax.y(), _aabbMax.z() },
                                          { _aabbMax.x(), _aabbMax.y(), _aabbMin.z() },
                                          { 0.1, 0.1, 0.8 } );
            }
        }
    }

    void TBtKinTreeAgentWrapper::_createBtResourcesFromKinTree()
    {
        if ( !m_kinTreeAgentPtr )
            return;

        if ( TYSOC_BULLET_USE_MULTIBODY )
            std::cout << "ERROR> can't use bullet's multibody feature yet" << std::endl;
        else
            m_rootCompound = _createBtCompoundFromBodyNode( m_kinTreeAgentPtr->getRootBody(), NULL );
    }

    TBtBodyCompound* TBtKinTreeAgentWrapper::_createBtCompoundFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                            TBtBodyCompound* parentBodyCompound )
    {
        if ( kinTreeBodyPtr->childCollisions.size() < 1 )
        {
            std::cout << "ERROR> tried to create a compound with no collisions "
                      << "for body node: " << kinTreeBodyPtr->name << std::endl;
            return NULL;
        }

        auto _bodyCompound = new TBtBodyCompound();
        _bodyCompound->kinTreeBodyPtr = kinTreeBodyPtr;
        m_bodyCompounds[ kinTreeBodyPtr->name ] = _bodyCompound;

        /* Create the resources for this compound ******************************/
        auto _kinCollisions = kinTreeBodyPtr->childCollisions;
        auto _firstCollision = _kinCollisions.front();

        // build the data for the first collision (wrt to core kintree body)
        _bodyCompound->startBodyToBaseTransform = _firstCollision->relTransform;
        _bodyCompound->baseToStartBodyTransform = _bodyCompound->startBodyToBaseTransform.inverse();
        _bodyCompound->btStartBody = _createBtRigidBodyForChain( _firstCollision );
        _bodyCompound->btBodiesInChain.push_back( _bodyCompound->btStartBody );
        _bodyCompound->btConstraintsInChain.push_back( NULL );// save a place for the constraint defined by the joints
        _bodyCompound->relTransformsInChain.push_back( _bodyCompound->startBodyToBaseTransform );

        if ( !_bodyCompound->btStartBody )
        {
            std::cout << "ERROR> could not create compound for body: "
                      << kinTreeBodyPtr->name << ". First body for collision: "
                      << _firstCollision->name << " returned NULL" << std::endl;

            return NULL;
        }

        // add front of the chain to the world
        m_btWorldPtr->addRigidBody( _bodyCompound->btStartBody );

        // create all other rigid bodies in the chain
        auto _previousBtBody = _bodyCompound->btStartBody;
        auto _previousToBaseTransform = _bodyCompound->startBodyToBaseTransform;
        for ( size_t i = 1; i < _kinCollisions.size(); i++ )
        {
            // create the current body in the chain
            auto _currentBtBody = _createBtRigidBodyForChain( _kinCollisions[i] );
            // and the relative transform to the base (kintree body node)
            auto _currentToBaseTransform = _kinCollisions[i]->relTransform;

            /*
            *  Compute the transform of this current body in the chain, wrt the ...
            *  previous body in the chain, by using the following computations:
            *
            *   ___       base
            *  |   |          T
            *  |___|---------  i-1
            *               |
            *   |           |
            *   |i-1        _
            *   |   T      |_| -> base frame (of kintree body node)
            *   |    i
            *   |           |
            *   ___         |
            *  |   |        | base
            *  |___|---------     T
            *                      i
            *
            *
            * base     i        base          i-1      base  -1    base     
            *     T  x  T      =    T    ->       T  =     T     x     T    
            *      i      i-1        i-1           i        i-1         i
            */
            auto _currentToPreviousTransform = _previousToBaseTransform.inverse() * 
                                               _currentToBaseTransform;

            // create a fixed constraint between the previous and the current bodies
            auto _fixedConstraint = _createFixedConstraint( _currentBtBody,
                                                            _previousBtBody,
                                                            _currentToPreviousTransform );

            // save the resources in the compound
            _bodyCompound->btBodiesInChain.push_back( _currentBtBody );
            _bodyCompound->btConstraintsInChain.push_back( _fixedConstraint );
            _bodyCompound->relTransformsInChain.push_back( _currentToPreviousTransform );

            // register the bt resources in the world
            m_btWorldPtr->addRigidBody( _currentBtBody );
            m_btWorldPtr->addConstraint( _fixedConstraint, true );// disable collision between linked bodies

            // and prepare for the next iteration
            _previousBtBody = _currentBtBody;
            _previousToBaseTransform = _currentToBaseTransform;
        }

        // and set the references related to the last body of the chain
        _bodyCompound->btEndBody = _bodyCompound->btBodiesInChain.back();
        _bodyCompound->endBodyToBaseTransform = _kinCollisions.back()->relTransform;

        /***********************************************************************/

        /* Create the constraint from the joints of this kintree body **********/

        auto _nodeConstraint = _createNodeBtConstraintFromCompound( kinTreeBodyPtr,
                                                                    _bodyCompound,
                                                                    parentBodyCompound );

        // if ( _nodeConstraint )
        //     m_btWorldPtr->addConstraint( _nodeConstraint, true );

        /***********************************************************************/

        /* Recurse for the other bodies (children of this body) ****************/

        for ( size_t i = 0; i < kinTreeBodyPtr->childBodies.size(); i++ )
            _createBtCompoundFromBodyNode( kinTreeBodyPtr->childBodies[i], _bodyCompound );

        /***********************************************************************/

        return _bodyCompound;
    }

    btRigidBody* TBtKinTreeAgentWrapper::_createBtRigidBodyForChain( agent::TKinTreeCollision* kinTreeCollisionPtr )
    {
        auto _btCollisionShapePtr = _createBtCollisionShapeSingle( kinTreeCollisionPtr );

        // create initial pos/rot from kintree collision (at initialization every ...
        // body, collision and visual are set to the corresponding pos/rot)
        auto _rbTransform = utils::toBtTransform( kinTreeCollisionPtr->geometry.worldTransform );
        auto _rbMotionState = new btDefaultMotionState( _rbTransform );

        // @TODO: Allow the user define the density for the objects (or mass as well), ...
        // and depending of the specified data by the user, do the required computations.

        // create initial properties, and compute them from the shape and density (default density for now)
        btVector3 _inertiaDiag;
        btScalar _inertiaMass;
        btScalar _shapeVolume;

        // compute volume from the given collision shape, and mass from volume and density
        _shapeVolume = _computeVolumeFromShape( _btCollisionShapePtr, _rbTransform );
        _inertiaMass = TYSOC_DEFAULT_DENSITY * _shapeVolume;

        // compute inertia props for the given shape using the mass computed above
        _btCollisionShapePtr->calculateLocalInertia( _inertiaMass, _inertiaDiag );

        // assemble all previous info into the struct used for rigid body creation
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo(
                                                            _inertiaMass,
                                                            _rbMotionState,
                                                            _btCollisionShapePtr,
                                                            _inertiaDiag );

        // construct the rigid body for this kintree body object
        auto _rigidBodyPtr = new btRigidBody( _rbConstructionInfo );

        // @DEBUG: just to check some issues with the ant
        _rigidBodyPtr->setRestitution( 0.1 );
        _rigidBodyPtr->setFriction( 1.0 );

        // make sure the object is going to be simulated by forcing activation
        _rigidBodyPtr->forceActivationState( DISABLE_DEACTIVATION );

        return _rigidBodyPtr;
    }

    btScalar TBtKinTreeAgentWrapper::_computeVolumeFromShape( btCollisionShape* collisionShapePtr,
                                                              const btTransform& frameTransform )
    {
        // @TODO: Change volume computation to a more accurate calculation ...
        // using the actual shapes instead of the overall AABB
        btVector3 _aabbMin;
        btVector3 _aabbMax;

        collisionShapePtr->getAabb( frameTransform, _aabbMin, _aabbMax );

        auto _vmin2max = _aabbMax - _aabbMin;
        auto _dx = _vmin2max.dot( frameTransform.getBasis().getColumn( 0 ) );
        auto _dy = _vmin2max.dot( frameTransform.getBasis().getColumn( 1 ) );
        auto _dz = _vmin2max.dot( frameTransform.getBasis().getColumn( 2 ) );

        return btFabs( _dx * _dy * _dz );
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

    btTypedConstraint* TBtKinTreeAgentWrapper::_createNodeBtConstraintFromCompound( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                                    TBtBodyCompound* currentBodyCompound,
                                                                                    TBtBodyCompound* parentBodyCompound )
    {
        btTypedConstraint* _btConstraint = NULL;

        // Declare some variables we will use in all cases
        TMat4 _firstInCurrentToLastInParentTransform = TMat4();
        btRigidBody* _currentFirstBtBodyPtr = currentBodyCompound->btStartBody;
        btRigidBody* _parentEndBtBodyPtr = NULL;

        // compute the transform from the first body in the current chain (current ...
        // compound) to the end body of the parent chain (parent compound). If no ...
        // parent, just pass an identity matrix, and NULL as parent
        if ( parentBodyCompound )
        {
            _parentEndBtBodyPtr = parentBodyCompound->btEndBody;

            auto _baseToParentTransform = kinTreeBodyPtr->relTransform;
            auto _startBodyToBaseTransform = currentBodyCompound->startBodyToBaseTransform;
            auto _startBodyToParentTransform = _baseToParentTransform * _startBodyToBaseTransform;

            auto _parentEndBodyToParentBaseTransform = parentBodyCompound->endBodyToBaseTransform;
            auto _parentBaseToParentEndBodyTransform = _parentEndBodyToParentBaseTransform.inverse();

            auto _firstInCurrentToLastInParentTransform = _parentBaseToParentEndBodyTransform * _startBodyToParentTransform;
        }

        if ( kinTreeBodyPtr->childJoints.size() == 0 )
        {
            
            // Create a fixed joint, as the related body has no dofs 
            _btConstraint = _createFixedConstraint( _currentFirstBtBodyPtr, 
                                                    _parentEndBtBodyPtr, 
                                                    _firstInCurrentToLastInParentTransform );
        }
        else if ( kinTreeBodyPtr->childJoints.size() == 1 )
        {
            // check the type, and create one constraint according to the type
            auto _kinTreeJointPtr = kinTreeBodyPtr->childJoints[0];
            auto _type = _kinTreeJointPtr->type;
            // auto _type = std::string( "ball" );

            if ( !_kinTreeJointPtr )
            {
                std::cout << "ERROR> the joint for body: " << kinTreeBodyPtr->name << " is NULL" << std::endl;
                return NULL;
            }

            auto _pivotInBase = _kinTreeJointPtr->relTransform.getPosition();
            auto _axisInBase = _kinTreeJointPtr->axis;

            auto _baseToStartBodyTransform = currentBodyCompound->baseToStartBodyTransform;
            auto _pivotInCurrentStartBody = _baseToStartBodyTransform.getRotation() * _pivotInBase +
                                            _baseToStartBodyTransform.getPosition();
            auto _axisInCurrentStartBody = _baseToStartBodyTransform.getRotation() *
                                           _axisInBase;

            if ( _type == "world" || _type == "fixed" )
            {
                // attached to world, so fixed
                if ( parentBodyCompound )
                    std::cout << "WARNING> body: " << kinTreeBodyPtr->name 
                              << " is attached to world, but it has a parent???" << std::endl;

                _btConstraint = _createFixedConstraint( _currentFirstBtBodyPtr, NULL, TMat4() );
            }
            else if ( _type == "free" || _type == "floating" )
            {
                // no constraint should be created
                _btConstraint = NULL;
            }
            else if ( _type == "hinge" || _type == "continuous" || _type == "revolute" )
            {
                _btConstraint = _createHingeConstraint( _currentFirstBtBodyPtr, 
                                                        _parentEndBtBodyPtr, 
                                                        _pivotInCurrentStartBody,
                                                        _axisInCurrentStartBody,
                                                        _firstInCurrentToLastInParentTransform,
                                                        { _kinTreeJointPtr->lowerLimit,
                                                          _kinTreeJointPtr->upperLimit } );
            }
            else if ( _type == "slide" || _type == "prismatic" )
            {
                _btConstraint = _createSliderConstraint( _currentFirstBtBodyPtr, 
                                                         _parentEndBtBodyPtr,
                                                         _pivotInCurrentStartBody,
                                                         _axisInCurrentStartBody,
                                                         _firstInCurrentToLastInParentTransform,
                                                         { _kinTreeJointPtr->lowerLimit,
                                                           _kinTreeJointPtr->upperLimit } );
            }
            else if ( _type == "ball" || _type == "spheric" || _type == "spherical" )
            {
                _btConstraint = _createPoint2PointConstraint( _currentFirstBtBodyPtr,
                                                              _parentEndBtBodyPtr,
                                                              _pivotInCurrentStartBody,
                                                              _firstInCurrentToLastInParentTransform );
            }
            else
            {
                std::cout << "ERROR> joint type: " << _type << " not supported" << std::endl;
            }
        }
        else
        {
            // create a single generic constraint with limits|dofs configured appropriately
            _btConstraint = _createGenericConstraintFromJoints( _currentFirstBtBodyPtr, 
                                                                _parentEndBtBodyPtr,
                                                                kinTreeBodyPtr->childJoints,
                                                                _firstInCurrentToLastInParentTransform );
        }

        return _btConstraint;
    }

    btGeneric6DofConstraint* TBtKinTreeAgentWrapper::_createFixedConstraint( btRigidBody* currentBtBodyPtr,
                                                                             btRigidBody* parentBtBodyPtr,
                                                                             const TMat4& currentToParentTransform )
    {
        btGeneric6DofConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyB''' for fixed-constraint creation
            _btConstraint = new btGeneric6DofConstraint( *currentBtBodyPtr,
                                                         utils::toBtTransform( currentToParentTransform ),
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
            auto _frameInB = utils::toBtTransform( currentToParentTransform );// Relative transform of body to parent

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

    btHingeConstraint* TBtKinTreeAgentWrapper::_createHingeConstraint( btRigidBody* currentBtBodyPtr,
                                                                       btRigidBody* parentBtBodyPtr,
                                                                       const TVec3& pivotInCurrent,
                                                                       const TVec3& axisInCurrent,
                                                                       const TMat4& currentToParentTransform,
                                                                       const TVec2& limits )
    {
        btHingeConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyA''' for hinge-constraint creation
            _btConstraint = new btHingeConstraint( *currentBtBodyPtr, 
                                                   utils::toBtVec3( pivotInCurrent ),
                                                   utils::toBtVec3( axisInCurrent ) );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for hinge-constraint creation

            auto _axisInA = axisInCurrent;
            auto _pivotInA = pivotInCurrent;

            auto _axisInB = currentToParentTransform.getRotation() * axisInCurrent;
            auto _pivotInB = currentToParentTransform.getRotation() * pivotInCurrent +
                                  currentToParentTransform.getPosition();

            _btConstraint = new btHingeConstraint( *currentBtBodyPtr, 
                                                   *parentBtBodyPtr,
                                                   utils::toBtVec3( _pivotInA ), 
                                                   utils::toBtVec3( _pivotInB ),
                                                   utils::toBtVec3( _axisInA ), 
                                                   utils::toBtVec3( _axisInB ) );
        }

        _btConstraint->setLimit( limits.x * TYSOC_PI / 180.0, limits.y * TYSOC_PI / 180.0 );

        return _btConstraint;
    }

    btSliderConstraint* TBtKinTreeAgentWrapper::_createSliderConstraint( btRigidBody* currentBtBodyPtr,
                                                                         btRigidBody* parentBtBodyPtr,
                                                                         const TVec3& pivotInCurrent,
                                                                         const TVec3& axisInCurrent,
                                                                         const TMat4& currentToParentTransform,
                                                                         const TVec2& limits )
    {
        btSliderConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyB''' for slider constraint creation
            // By looking at btSliderConstraint::calculateTransforms (line 153) ...
            // it seems that the transform implicitly defines the sliding axis

            auto _transform = tysoc::TMat4();
            _transform.setPosition( pivotInCurrent );
            _transform.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, axisInCurrent ) );

            _btConstraint = new btSliderConstraint( *currentBtBodyPtr,
                                                    utils::toBtTransform( _transform ),
                                                    true );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for slider-constraint creation

            auto _axisInA = axisInCurrent;
            auto _pivotInA = pivotInCurrent;

            auto _frameInA = tysoc::TMat4();
            _frameInA.setPosition( _pivotInA );
            _frameInA.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, _axisInA ) );

            auto _axisInB = currentToParentTransform.getRotation() * _axisInA;
            auto _pivotInB = currentToParentTransform.getRotation() * _pivotInA +
                             currentToParentTransform.getPosition();
            auto _frameInB = tysoc::TMat4();
            _frameInB.setPosition( _pivotInB );
            _frameInB.setRotation( tysoc::shortestArcQuat( { 1, 0, 0 }, _axisInB ) );

            _btConstraint = new btSliderConstraint( *currentBtBodyPtr,
                                                    *parentBtBodyPtr,
                                                    utils::toBtTransform( _frameInA ),
                                                    utils::toBtTransform( _frameInB ), 
                                                    true );
        }

        _btConstraint->setLowerLinLimit( limits.x );
        _btConstraint->setUpperLinLimit( limits.y );
        _btConstraint->setLowerAngLimit( 0 );
        _btConstraint->setUpperAngLimit( 0 );

        return _btConstraint;
    }

    btPoint2PointConstraint* TBtKinTreeAgentWrapper::_createPoint2PointConstraint( btRigidBody* currentBtBodyPtr,
                                                                                   btRigidBody* parentBtBodyPtr,
                                                                                   const TVec3& pivotInCurrent,
                                                                                   const TMat4& currentToParentTransform )
    {
        // @TODO: Replace with btGeneric6DofSpring2Constraint, as in :
        // https://github.com/xbpeng/DeepLoco/blob/c4e2db93fefcd49ee7a2481918e2f7db1c1da733/sim/World.cpp#L755

        btPoint2PointConstraint* _btConstraint = NULL;

        if ( !parentBtBodyPtr )
        {
            // Just use '''currentBtBodyPtr=bodyA''' for hinge-constraint creation
            _btConstraint = new btPoint2PointConstraint( *currentBtBodyPtr, 
                                                         utils::toBtVec3( pivotInCurrent ) );
        }
        else
        {
            // Use '''parentBtBodyPtr=bodyB''' and '''currentBtBodyPtr=bodyA''' ...
            // for hinge-constraint creation

            auto _pivotInA = pivotInCurrent;
            auto _pivotInB = currentToParentTransform.getRotation() * _pivotInA +
                             currentToParentTransform.getPosition();

            _btConstraint = new btPoint2PointConstraint( *currentBtBodyPtr, 
                                                         *parentBtBodyPtr,
                                                         utils::toBtVec3( _pivotInA ), 
                                                         utils::toBtVec3( _pivotInB ) );
        }

        return _btConstraint;
    }

    btGeneric6DofConstraint* TBtKinTreeAgentWrapper::_createGenericConstraintFromJoints( btRigidBody* currentBtBodyPtr,
                                                                                         btRigidBody* parentBtBodyPtr,
                                                                                         const std::vector< agent::TKinTreeJoint* >& joints,
                                                                                         const TMat4& currentToParentTransform )
    {
        btGeneric6DofConstraint* _btConstraint = NULL;

        // Grab only the DOFs whose axes map to basis vectors, only ...
        // those that consist of 'slide' or 'hinge' joints, and max 6 joints
        std::vector< int > _dofIndices;
        std::vector< tysoc::TVec2 > _dofLimits;
        for ( size_t q = 0; q < joints.size(); q++ )
        {
            if ( q > 5 )
            {
                std::cout << "WARNING> trying to configure more than 6 joints " 
                          << "for generic constraint" << std::endl;
                break;
            }

            if ( joints[q]->type != "hinge" && joints[q]->type != "slide" )
            {
                std::cout << "WARNING> tried to configure " << joints[q]->type
                          << " for generic joint, which is not supported (only slide and hinge)" << std::endl;
                continue;
            }

            // map joint AXIS to DOF index
            if ( joints[q]->type == "slide" )
            {
                if ( std::fabs( joints[q]->axis.x ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "lin-x" << std::endl;
                    _dofIndices.push_back( 0 );
                    _dofLimits.push_back( { joints[q]->lowerLimit, joints[q]->upperLimit } );
                }
                else if ( std::fabs( joints[q]->axis.y ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "lin-y" << std::endl;
                    _dofIndices.push_back( 1 );
                    _dofLimits.push_back( { joints[q]->lowerLimit, joints[q]->upperLimit } );
                }
                else if ( std::fabs( joints[q]->axis.z ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "lin-z" << std::endl;
                    _dofIndices.push_back( 2 );
                    _dofLimits.push_back( { joints[q]->lowerLimit, joints[q]->upperLimit } );
                }
            }
            else
            {
                if ( std::fabs( joints[q]->axis.x ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "ang-x" << std::endl;
                    _dofIndices.push_back( 3 );
                    _dofLimits.push_back( { joints[q]->lowerLimit, joints[q]->upperLimit } );
                }
                else if ( std::fabs( joints[q]->axis.y ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "ang-y" << std::endl;
                    _dofIndices.push_back( 4 );
                    _dofLimits.push_back( { joints[q]->lowerLimit, joints[q]->upperLimit } );
                }
                else if ( std::fabs( joints[q]->axis.z ) > 0 )
                {
                    // std::cout << "LOG> Setting dof: " << "ang-z" << std::endl;
                    _dofIndices.push_back( 5 );
                    _dofLimits.push_back( { joints[q]->lowerLimit, joints[q]->upperLimit } );
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
            auto _frameInB = utils::toBtTransform( currentToParentTransform );// Relative transform of body to parent

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