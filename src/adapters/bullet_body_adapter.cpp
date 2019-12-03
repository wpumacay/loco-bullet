
#include <adapters/bullet_body_adapter.h>

using namespace tysoc::bullet;

namespace tysoc {

    TBtBodyAdapter::TBtBodyAdapter( TBody* bodyPtr )
        : TIBodyAdapter( bodyPtr )
    {
        m_btCompoundShape = nullptr;
        m_btRigidBody = nullptr;
    }

    TBtBodyAdapter::~TBtBodyAdapter()
    {
        m_btCompoundShape = nullptr;
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

        m_btCompoundShape = std::unique_ptr< btCompoundShape >( new btCompoundShape() );

        auto _collider = m_bodyPtr->collision();
        if ( _collider && _collider->adapter() )
        {
            auto _collisionAdapter = dynamic_cast< TBtCollisionAdapter* >( _collider->adapter() );
            _collisionAdapter->setCompoundShapeRef( m_btCompoundShape.get() );
            _collisionAdapter->setIndexInCompoundShape( m_btCompoundShape->getNumChildShapes() );

            if ( _collisionAdapter->collisionShape() )
            {
                m_btCompoundShape->addChildShape( _collisionAdapter->collisionLocalTf(), 
                                                  _collisionAdapter->collisionShape() );
            }
        }

        /* create rigid body (using maximal-coordinates bullet API) *******************************/

        // grab rigid-body data given by the user
        auto _data = m_bodyPtr->data();

        // rigid-body start configuration. @todo: check if local when adding support for joints and linkages
        auto _rbMotionState = new btDefaultMotionState( utils::toBtTransform( m_bodyPtr->tf0() ) );

        // inertial properties (compute only if dynamic). @todo: compound uses aabb approximation, should handle compound differently
        btScalar _rbInertiaMass = 0.0f;
        btVector3 _rbInertiaDiag;
        if ( m_bodyPtr->dyntype() == eDynamicsType::DYNAMIC )
        {
            if ( _data.inertialData.mass != 0.0f )
                _rbInertiaMass = _data.inertialData.mass;
            else
                _rbInertiaMass = utils::computeVolumeFromShape( m_btCompoundShape.get() ) * TYSOC_DEFAULT_DENSITY;

            if ( _rbInertiaMass != 0.0f )
                m_btCompoundShape->calculateLocalInertia( _rbInertiaMass, _rbInertiaDiag );
        }

        // finally, create the bullet-rigid-body (maximal-coordinates)
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo( _rbInertiaMass, 
                                                                      _rbMotionState, 
                                                                      m_btCompoundShape.get(), 
                                                                      _rbInertiaDiag );
        m_btRigidBody = std::unique_ptr< btRigidBody >( new btRigidBody( _rbConstructionInfo ) );
        m_btRigidBody->forceActivationState( DISABLE_DEACTIVATION );
    }

    void TBtBodyAdapter::reset()
    {
        assert( m_bodyPtr && m_btRigidBody );

        m_btRigidBody->setWorldTransform( utils::toBtTransform( m_bodyPtr->tf0() ) );
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
        assert( m_bodyPtr && m_btRigidBody );

        // @todo: validate for parent body when supporting multi-links
        m_btRigidBody->getWorldTransform().setOrigin( utils::toBtVec3( position ) );
    }

    void TBtBodyAdapter::setRotation( const TMat3& rotation )
    {
        assert( m_bodyPtr && m_btRigidBody );

        // @todo: validate for parent body when supporting multi-links
        m_btRigidBody->getWorldTransform().setBasis( utils::toBtMat3( rotation ) );
    }

    void TBtBodyAdapter::setTransform( const TMat4& transform )
    {
        assert( m_bodyPtr && m_btRigidBody );

        // @todo: validate for parent body when supporting multi-links
        m_btRigidBody->setWorldTransform( utils::toBtTransform( transform ) );
    }

    void TBtBodyAdapter::getPosition( TVec3& dstPosition )
    {
        assert( m_bodyPtr && m_btRigidBody );

        dstPosition = utils::fromBtVec3( m_btRigidBody->getWorldTransform().getOrigin() );
    }

    void TBtBodyAdapter::getRotation( TMat3& dstRotation )
    {
        assert( m_bodyPtr && m_btRigidBody );

        dstRotation = utils::fromBtMat3( m_btRigidBody->getWorldTransform().getBasis() );
    }

    void TBtBodyAdapter::getTransform( TMat4& dstTransform )
    {
        assert( m_bodyPtr && m_btRigidBody );

        dstTransform = utils::fromBtTransform( m_btRigidBody->getWorldTransform() );
    }

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr )
    {
        return new TBtBodyAdapter( bodyPtr );
    }

}