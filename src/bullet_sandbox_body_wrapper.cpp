
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
        _createBtCollisionShape();
        _createBtRigidBody();
    }

    void TBtBodyWrapper::_resetInternal()
    {
        if ( !m_bodyPtr )
            return;

        if ( !m_btCollisionShapePtr )
            return;

        if ( !m_btRigidBodyPtr )
            return;

        m_btCollisionShapePtr->setLocalScaling( utils::toBtVec3( m_bodyPtr->size ) );

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

        if ( !m_btRigidBodyPtr )
            return;

        // extract the world transform of the simulated body
        auto _rbTransform = m_btRigidBodyPtr->getWorldTransform();
        // and set it to the world transform of the wrapped body
        m_bodyPtr->worldTransform = utils::fromBtTransform( _rbTransform );
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

    void TBtBodyWrapper::_createBtCollisionShape()
    {
        if ( !m_btWorldPtr )
            return;

        if ( !m_bodyPtr )
            return;

        auto _type = m_bodyPtr->type;
        auto _size = m_bodyPtr->size;

        if ( _type == "box" )
        {
            m_btCollisionShapePtr = new btBoxShape( btVector3( 0.5, 0.5, 0.5 ) );
        }
        else if ( _type == "sphere" )
        {
            m_btCollisionShapePtr = new btSphereShape( 1.0 );
        }
        else if ( _type == "capsule" )
        {
            _size = { _size.z, _size.x, _size.y };
            m_btCollisionShapePtr = new btCapsuleShapeZ( 1.0, 1.0 );
        }
        else if ( _type == "cylinder" )
        {
            _size = { _size.x, _size.x, _size.y };
            m_btCollisionShapePtr = new btCylinderShapeZ( btVector3( 1.0, 1.0, 1.0 ) );
        }
        else if ( _type == "plane" )
        {
            m_btCollisionShapePtr = new btStaticPlaneShape( btVector3( 0, 0, 1 ), 0 );
        }

        if ( !m_btCollisionShapePtr )
            std::cout << "WARNING> could not create shape of type: " << _type << std::endl;
        else
            m_btCollisionShapePtr->setLocalScaling( utils::toBtVec3( _size ) );
    }

    void TBtBodyWrapper::_createBtRigidBody()
    {
        if ( !m_btWorldPtr )
            return;

        if ( !m_bodyPtr )
            return;

        if ( !m_btCollisionShapePtr )
            return;

        // create initial pos/rot from wrapped body
        auto _rbTransform = utils::toBtTransform( m_bodyPtr->worldTransform );
        auto _rbMotionState = new btDefaultMotionState( _rbTransform );

        // initialize inertial properties
        btScalar _rbMass = m_bodyPtr->mass;
        btVector3 _rbInertia = btVector3( 0, 0, 0 );
        if ( _rbMass != 0.0f )
            m_btCollisionShapePtr->calculateLocalInertia( _rbMass, _rbInertia );

        // assemble the struct used for rigid body creation
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo(
                                                            _rbMass,
                                                            _rbMotionState,
                                                            m_btCollisionShapePtr,
                                                            _rbInertia );

        // construct the rigid body with the previous struct
        m_btRigidBodyPtr = new btRigidBody( _rbConstructionInfo );

        // grab some extra info from the wrapped body
        m_btRigidBodyPtr->setRestitution( 0.5f );
        m_btRigidBodyPtr->setFriction( m_bodyPtr->friction.x );

        // make sure the object is going to be simulated by forcing activation
        m_btRigidBodyPtr->forceActivationState( DISABLE_DEACTIVATION );

        // and finally add it to the dynamics world
        m_btWorldPtr->addRigidBody( m_btRigidBodyPtr );
    }



}}