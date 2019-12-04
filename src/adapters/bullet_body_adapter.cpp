
#include <adapters/bullet_body_adapter.h>

using namespace tysoc::bullet;

namespace tysoc {

    TBtBodyAdapter::TBtBodyAdapter( TBody* bodyPtr )
        : TIBodyAdapter( bodyPtr )
    {
        m_btRigidBody = nullptr;
        m_btHfieldTfCompensation.setIdentity();
        m_btHfieldTfCompensationInv.setIdentity();
        m_isHeightfield = false;
    }

    TBtBodyAdapter::~TBtBodyAdapter()
    {
        m_btRigidBody = nullptr;
    }

    void TBtBodyAdapter::build()
    {
        if ( !m_bodyPtr )
        {
            std::cout << "ERROR> tried to create bullet resources for a null body" << std::endl;
            return;
        }

        /* create a compound shape for the colliders (even for a single-collider, as it might
           have relative-transform to the parent body) ********************************************/

        auto _collider = m_bodyPtr->collision();
        if ( !_collider )
        {
            TYSOC_CORE_ERROR( "Bullet body-adapter >>> body {0} has no collider", m_bodyPtr->name() );
            return;
        }

        auto _colliderAdapter = _collider->adapter();
        if ( !_colliderAdapter )
        {
            TYSOC_CORE_ERROR( "Bullet body-adapter >>> body {0} has collider with no adapter", m_bodyPtr->name() );
            return;
        }

        auto _collisionShape = dynamic_cast< TBtCollisionAdapter* >( _colliderAdapter )->collisionShape();
        if ( !_collisionShape )
        {
            TYSOC_CORE_ERROR( "Bullet body-adapter >>> body {0} has collider without valid btCollisionShape", m_bodyPtr->name() );
            return;
        }

        // create compensation transform if the collider is a hfield shape
        if ( _collider->shape() == eShapeType::HFIELD )
        {
            m_isHeightfield = true;
            m_btHfieldTfCompensation.setOrigin( utils::toBtVec3( { 0.0f, 0.0f, 0.5f * _collider->data().size.z * _collider->data().size.z } ) );
            m_btHfieldTfCompensationInv = m_btHfieldTfCompensation.inverse();
        }

        /* create rigid body (using maximal-coordinates bullet API) *******************************/

        // grab rigid-body data given by the user
        auto _data = m_bodyPtr->data();

        // rigid-body start configuration. @todo: check if local when adding support for joints and linkages
        btDefaultMotionState* _rbMotionState = nullptr;
        if ( _collider->shape() == eShapeType::HFIELD ) 
            _rbMotionState = new btDefaultMotionState( utils::toBtTransform( m_bodyPtr->tf0() ) * m_btHfieldTfCompensation );
        else
            _rbMotionState = new btDefaultMotionState( utils::toBtTransform( m_bodyPtr->tf0() ) );

        // inertial properties (compute only if dynamic). @todo: compound uses aabb approximation, should handle compound differently
        btScalar _rbMass = 0.0f;
        btVector3 _rbInertiaDiag;
        if ( m_bodyPtr->dyntype() == eDynamicsType::DYNAMIC )
        {
            if ( _data.inertialData.mass != 0.0f )
                _rbMass = _data.inertialData.mass;
            else
                _rbMass = utils::computeVolumeFromShape( _collisionShape ) * TYSOC_DEFAULT_DENSITY;

            if ( _rbMass != 0.0f )
                _collisionShape->calculateLocalInertia( _rbMass, _rbInertiaDiag );
        }

        // finally, create the bullet-rigid-body (maximal-coordinates)
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo( _rbMass, 
                                                                      _rbMotionState, 
                                                                      _collisionShape, 
                                                                      _rbInertiaDiag );
        m_btRigidBody = std::unique_ptr< btRigidBody >( new btRigidBody( _rbConstructionInfo ) );
        m_btRigidBody->forceActivationState( DISABLE_DEACTIVATION );
    }

    void TBtBodyAdapter::reset()
    {
        if ( !m_btRigidBody )
            return;

        m_btRigidBody->setWorldTransform( utils::toBtTransform( m_bodyPtr->tf0() ) * m_btHfieldTfCompensation );
        m_btRigidBody->setLinearVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
        m_btRigidBody->setAngularVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
        m_btRigidBody->forceActivationState( DISABLE_DEACTIVATION );
    }

    void TBtBodyAdapter::update()
    {
        // do nothing for now, because so far we only need to use the overriden methods
    }

    void TBtBodyAdapter::setPosition( const TVec3& position )
    {
        if ( !m_btRigidBody )
            return;

        m_btRigidBody->getWorldTransform().setOrigin( utils::toBtVec3( position ) );

        if ( m_isHeightfield )
            m_btRigidBody->setWorldTransform( m_btRigidBody->getWorldTransform() * m_btHfieldTfCompensation );
    }

    void TBtBodyAdapter::setRotation( const TMat3& rotation )
    {
        if ( !m_btRigidBody )
            return;

        m_btRigidBody->getWorldTransform().setBasis( utils::toBtMat3( rotation ) );

        if ( m_isHeightfield )
            m_btRigidBody->setWorldTransform( m_btRigidBody->getWorldTransform() * m_btHfieldTfCompensation );
    }

    void TBtBodyAdapter::setTransform( const TMat4& transform )
    {
        if ( !m_btRigidBody )
            return;

        m_btRigidBody->setWorldTransform( utils::toBtTransform( transform ) );

        if ( m_isHeightfield )
            m_btRigidBody->setWorldTransform( m_btRigidBody->getWorldTransform() * m_btHfieldTfCompensation );
    }

    void TBtBodyAdapter::getPosition( TVec3& dstPosition )
    {
        if ( !m_btRigidBody )
            return;

        if ( m_isHeightfield )
            dstPosition = utils::fromBtVec3( ( m_btRigidBody->getWorldTransform() * m_btHfieldTfCompensationInv ).getOrigin() );
        else
            dstPosition = utils::fromBtVec3( ( m_btRigidBody->getWorldTransform() ).getOrigin() );
    }

    void TBtBodyAdapter::getRotation( TMat3& dstRotation )
    {
        if ( !m_btRigidBody )
            return;

        if ( m_isHeightfield )
            dstRotation = utils::fromBtMat3( ( m_btRigidBody->getWorldTransform() * m_btHfieldTfCompensationInv ).getBasis() );
        else
            dstRotation = utils::fromBtMat3( ( m_btRigidBody->getWorldTransform() ).getBasis() );
    }

    void TBtBodyAdapter::getTransform( TMat4& dstTransform )
    {
        if ( !m_btRigidBody )
            return;

        if ( m_isHeightfield )
            dstTransform = utils::fromBtTransform( ( m_btRigidBody->getWorldTransform() * m_btHfieldTfCompensationInv ) );
        else
            dstTransform = utils::fromBtTransform( ( m_btRigidBody->getWorldTransform() ) );
    }

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr )
    {
        return new TBtBodyAdapter( bodyPtr );
    }

}