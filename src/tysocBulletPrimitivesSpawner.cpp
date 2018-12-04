
#include <tysocBulletPrimitivesSpawner.h>


namespace tysocBullet
{


    TBulletPrimitivesSpawner::TBulletPrimitivesSpawner( btDynamicsWorld* worldPtr )
        : tysocUtils::TPrimitivesSpawner()
    {
        m_bulletWorldPtr = worldPtr;
    }

    TBulletPrimitivesSpawner::~TBulletPrimitivesSpawner()
    {
        m_bulletWorldPtr = NULL;
    }

    btRigidBody* TBulletPrimitivesSpawner::_createRigidBody( const std::string& shape,
                                                             float sx, float sy, float sz,
                                                             float x, float y, float z )
    {
        // @CHECK: I've already done this in two/three places, perhaps could be ...
        // generalized a bit, because for each type of new handler I'm repeating the ...
        // code that creates rigid bodies

        btCollisionShape* _rbCollisionShapePtr = NULL;
        if ( shape == "box" )
        {
            _rbCollisionShapePtr = new btBoxShape( btVector3( 0.5f, 0.5f, 0.5f ) );
        }
        else if ( shape == "sphere" )
        {
            _rbCollisionShapePtr = new btSphereShape( 1.0f );
        }
        else if ( shape == "capsule" )
        {
            _rbCollisionShapePtr = new btCapsuleShapeZ( 1.0f, 0.5f );
        }
        else if ( shape == "cylinder" )
        {
            _rbCollisionShapePtr = new btCylinderShapeZ( btVector3( 1.0f, 1.0f, 1.0f ) );
        }

        if ( !_rbCollisionShapePtr )
        {
            // @TODO: Change logs with more information of the site they are being called
            std::cout << "WARNING> could not create shape of type: " << shape << std::endl;
            return NULL;
        }

        _rbCollisionShapePtr->setLocalScaling( btVector3( sx, sy, sz ) );

        btDefaultMotionState* _rbMotionStatePtr = NULL;
        {
            btTransform _rbTransform;
            _rbTransform.setIdentity();
            _rbTransform.setOrigin( btVector3( x, y, z ) );

            _rbMotionStatePtr = new btDefaultMotionState( _rbTransform );
        }

        btScalar _rbMass = 1.0f;
        btVector3 _rbInertia( 0, 0, 0 );
        if ( _rbMass != 0.0f )
        {
            _rbCollisionShapePtr->calculateLocalInertia( _rbMass, _rbInertia );
        }

        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo( 
                                                            _rbMass,
                                                            _rbMotionStatePtr,
                                                            _rbCollisionShapePtr,
                                                            _rbInertia );

        auto _rbodyPtr = new btRigidBody( _rbConstructionInfo );
        _rbodyPtr->setRestitution( 0.5f );
        _rbodyPtr->setFriction( 0.8f );

        _rbodyPtr->forceActivationState( WANTS_DEACTIVATION );

        m_bulletWorldPtr->addRigidBody( _rbodyPtr );

        return _rbodyPtr;
    }

    tysocUtils::TDebugPrimitive* TBulletPrimitivesSpawner::_createPrimitiveInternal( 
                                                            const std::string& type,
                                                            float sx, float sy, float sz,
                                                            float x, float y, float z )
    {
        auto _bulletPrimitivePtr = new TBulletDebugPrimitive();

        _bulletPrimitivePtr->rigidBody = _createRigidBody( type,
                                                           sx, sy, sz,
                                                           x, y, z );

        return _bulletPrimitivePtr;
    }
    
    void TBulletPrimitivesSpawner::_recyclePrimitiveInternal( tysocUtils::TDebugPrimitive* primitivePtr )
    {
        auto _rbodyPtr = reinterpret_cast< TBulletDebugPrimitive* >( primitivePtr )->rigidBody;
        _rbodyPtr->forceActivationState( WANTS_DEACTIVATION );
    }
    
    void TBulletPrimitivesSpawner::_activatePrimitiveInternal( 
                                        tysocUtils::TDebugPrimitive* primitivePtr,
                                        float sx, float sy, float sz,
                                        float x, float y, float z )
    {
        auto _rbodyPtr = reinterpret_cast< TBulletDebugPrimitive* >( primitivePtr )->rigidBody;
        auto _rbCollisionShapePtr = _rbodyPtr->getCollisionShape();

        _rbCollisionShapePtr->setLocalScaling( btVector3( sx, sy, sz ) );

        btTransform _rbTransform;
        _rbTransform.setIdentity();
        _rbTransform.setOrigin( btVector3( x, y, z ) );

        _rbodyPtr->setLinearVelocity( btVector3( 0, 0, 0 ) );
        _rbodyPtr->setAngularVelocity( btVector3( 0, 0, 0 ) );
        _rbodyPtr->setWorldTransform( _rbTransform );
        _rbodyPtr->forceActivationState( DISABLE_DEACTIVATION );
    }

    void TBulletPrimitivesSpawner::_updatePrimitiveInternal( tysocUtils::TDebugPrimitive* primitivePtr )
    {
        auto _rbodyPtr = reinterpret_cast< TBulletDebugPrimitive* >( primitivePtr )->rigidBody;
        auto _rbTransform = _rbodyPtr->getWorldTransform();

        auto _rbPosition = _rbTransform.getOrigin();
        auto _rbOrientation = _rbTransform.getBasis();

        primitivePtr->position.x = _rbPosition.x();
        primitivePtr->position.y = _rbPosition.y();
        primitivePtr->position.z = _rbPosition.z();

        float _rotmat[9];
        getMat3Array( _rbOrientation, _rotmat );

        primitivePtr->rotmat[0] = _rotmat[0];
        primitivePtr->rotmat[1] = _rotmat[1];
        primitivePtr->rotmat[2] = _rotmat[2];
        primitivePtr->rotmat[3] = _rotmat[3];
        primitivePtr->rotmat[4] = _rotmat[4];
        primitivePtr->rotmat[5] = _rotmat[5];
        primitivePtr->rotmat[6] = _rotmat[6];
        primitivePtr->rotmat[7] = _rotmat[7];
        primitivePtr->rotmat[8] = _rotmat[8];

    }
}