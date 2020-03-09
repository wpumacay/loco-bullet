
#include <adapters/loco_single_body_adapter_bullet.h>

namespace loco {
namespace bullet {

    TBulletSingleBodyAdapter::TBulletSingleBodyAdapter( TIBody* bodyRef )
        : TIBodyAdapter( bodyRef )
    {
        LOCO_CORE_ASSERT( bodyRef, "TBulletSingleBodyAdapter >>> adaptee (body) should be a valid \
                          refernce (nullptr given)" );
        LOCO_CORE_ASSERT( bodyRef->classType() == loco::eBodyClassType::SINGLE_BODY,
                          "TBulletSingleBodyAdapter >>> body {0} is not of class-type single-body", bodyRef->name() );
        LOCO_CORE_ASSERT( bodyRef->collision(), "TBulletSingleBodyAdapter >>> body {0} doesn't have \
                          a valid collider (found nullptr)", bodyRef->name() );

        m_bulletRigidBody = nullptr;
        m_bulletWorldRef = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        const std::string name = ( m_bodyRef ) ? m_bodyRef->name() : "undefined";
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TBulletSingleBodyAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TBulletSingleBodyAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TBulletSingleBodyAdapter::~TBulletSingleBodyAdapter()
    {
        m_bulletRigidBody = nullptr;
        m_bulletWorldRef = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        const std::string name = ( m_bodyRef ) ? m_bodyRef->name() : "undefined";
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TBulletSingleBodyAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TBulletSingleBodyAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TBulletSingleBodyAdapter::Build()
    {
        auto collision = m_bodyRef->collision();
        if ( auto collision_adapter = dynamic_cast<TBulletCollisionAdapter*>( collision->adapter() ) )
        {
            collision_adapter->Build();
            auto bt_collision_shape = collision_adapter->collision_shape();
            LOCO_CORE_ASSERT( bt_collision_shape, "TBulletSingleBodyAdapter::Build >>> single-body {0}'s \
                              collider (called {1}) doesn't have a valid btCollisionShape", m_bodyRef->name(), collision->name() );
            // Create rigid body (using maximal-coordinates bullet API) ****************************
            btScalar bt_mass = 0.0;
            btVector3 bt_inertia_diag;
            if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
            {
                if ( collision->shape() != eShapeType::HFIELD && collision->shape() != eShapeType::PLANE )
                {
                    if ( m_bodyRef->data().inertia.mass > loco::EPS )
                        bt_mass = m_bodyRef->data().inertia.mass;
                    else
                        bt_mass = ComputeVolumeFromBtShape( bt_collision_shape ) * loco::DEFAULT_DENSITY;

                    if ( bt_mass > loco::EPS )
                        bt_collision_shape->calculateLocalInertia( bt_mass, bt_inertia_diag );
                }
                else
                {
                    LOCO_CORE_WARN( "TBulletSingleBodyAdapter::Build >>> single-body {0} has collider of \
                                     type {1}, but it can't be dynamic", m_bodyRef->name(), ToString( collision->shape() ) );
                }
            }

            btRigidBody::btRigidBodyConstructionInfo bt_construction_info( bt_mass, nullptr, bt_collision_shape, bt_inertia_diag );
            bt_construction_info.m_startWorldTransform = mat4_to_bt( m_bodyRef->tf0() );

            m_bulletRigidBody = std::make_unique<btRigidBody>( bt_construction_info );
            m_bulletRigidBody->forceActivationState( DISABLE_DEACTIVATION );
        }
    }

    void TBulletSingleBodyAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_bulletWorldRef, "TBulletSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid bullet-world reference", m_bodyRef->name() );
        LOCO_CORE_ASSERT( m_bulletRigidBody, "TBulletSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        auto collision = m_bodyRef->collision();
        if ( auto collision_adapter = dynamic_cast<TBulletCollisionAdapter*>( collision->adapter() ) )
        {
            collision_adapter->SetBulletWorld( m_bulletWorldRef );
            collision_adapter->SetBulletRigidBody( m_bulletRigidBody.get() );
            collision_adapter->Initialize();
        }

        const ssize_t collision_group = collision->collisionGroup();
        const ssize_t collision_mask = collision->collisionMask();
        m_bulletWorldRef->addRigidBody( m_bulletRigidBody.get(), collision_group, collision_mask );
    }

    void TBulletSingleBodyAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_bulletRigidBody, "TBulletSingleBodyAdapter::Reset >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        m_bulletRigidBody->setWorldTransform( mat4_to_bt( m_bodyRef->tf0() ) );
        m_bulletRigidBody->setLinearVelocity( btVector3( 0.0, 0.0, 0.0 ) );
        m_bulletRigidBody->setAngularVelocity( btVector3( 0.0, 0.0, 0.0 ) );
        m_bulletRigidBody->forceActivationState( DISABLE_DEACTIVATION );
    }

    void TBulletSingleBodyAdapter::PreStep()
    {
        // Nothing to prepare before to a simulation step
    }

    void TBulletSingleBodyAdapter::PostStep()
    {
        // Nothing to process after to a simulation step
    }

    void TBulletSingleBodyAdapter::SetPosition( const TVec3& position )
    {
        LOCO_CORE_ASSERT( m_bulletRigidBody, "TBulletSingleBodyAdapter::SetPosition >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        m_bulletRigidBody->getWorldTransform().setOrigin( vec3_to_bt( position ) );
    }

    void TBulletSingleBodyAdapter::SetRotation( const TMat3& rotation )
    {
        LOCO_CORE_ASSERT( m_bulletRigidBody, "TBulletSingleBodyAdapter::SetRotation >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        m_bulletRigidBody->getWorldTransform().setBasis( mat3_to_bt( rotation ) );
    }

    void TBulletSingleBodyAdapter::SetTransform( const TMat4& transform )
    {
        LOCO_CORE_ASSERT( m_bulletRigidBody, "TBulletSingleBodyAdapter::SetTransform >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        m_bulletRigidBody->setWorldTransform( mat4_to_bt( transform ) );
    }

    void TBulletSingleBodyAdapter::GetPosition( TVec3& dstPosition )
    {
        LOCO_CORE_ASSERT( m_bulletRigidBody, "TBulletSingleBodyAdapter::GetPosition >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        dstPosition = vec3_from_bt( m_bulletRigidBody->getWorldTransform().getOrigin() );
    }

    void TBulletSingleBodyAdapter::GetRotation( TMat3& dstRotation )
    {
        LOCO_CORE_ASSERT( m_bulletRigidBody, "TBulletSingleBodyAdapter::GetRotation >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        dstRotation = mat3_from_bt( m_bulletRigidBody->getWorldTransform().getBasis() );
    }

    void TBulletSingleBodyAdapter::GetTransform( TMat4& dstTransform )
    {
        LOCO_CORE_ASSERT( m_bulletRigidBody, "TBulletSingleBodyAdapter::GetTransform >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        dstTransform = mat4_from_bt( m_bulletRigidBody->getWorldTransform() );
    }

    void TBulletSingleBodyAdapter::SetLocalPosition( const TVec3& position )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TBulletSingleBodyAdapter::SetLocalRotation( const TMat3& rotation )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TBulletSingleBodyAdapter::SetLocalTransform( const TMat4& transform )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TBulletSingleBodyAdapter::GetLocalPosition( TVec3& dstPosition )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TBulletSingleBodyAdapter::GetLocalRotation( TMat3& dstRotation )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

    void TBulletSingleBodyAdapter::GetLocalTransform( TMat4& dstTransform )
    {
        // Single-bodies don't have this functionality (not part of a compound|kintree)
    }

}}