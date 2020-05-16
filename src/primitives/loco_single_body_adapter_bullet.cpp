
#include <primitives/loco_single_body_adapter_bullet.h>

namespace loco {
namespace bullet {

    TBulletSingleBodyAdapter::TBulletSingleBodyAdapter( TSingleBody* body_ref )
        : TISingleBodyAdapter( body_ref )
    {
        LOCO_CORE_ASSERT( body_ref, "TBulletSingleBodyAdapter >>> adaptee (body) should be a valid \
                          refernce (nullptr given)" );
        LOCO_CORE_ASSERT( body_ref->collider(), "TBulletSingleBodyAdapter >>> body {0} doesn't have \
                          a valid collider (found nullptr)", body_ref->name() );

        m_BulletRigidBody = nullptr;
        m_BulletWorldRef = nullptr;
        m_HfieldTfCompensation.setIdentity();
        m_HfieldTfCompensationInv.setIdentity();

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        const std::string name = ( m_BodyRef ) ? m_BodyRef->name() : "undefined";
        if ( tinyutils::Logger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TBulletSingleBodyAdapter {0} @ {1}", name, tinyutils::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TBulletSingleBodyAdapter " << name << " @ " << tinyutils::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TBulletSingleBodyAdapter::~TBulletSingleBodyAdapter()
    {
        m_ColliderAdapter = nullptr;
        m_ConstraintAdapter = nullptr;

        m_BulletRigidBody = nullptr;
        m_BulletWorldRef = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        const std::string name = ( m_BodyRef ) ? m_BodyRef->name() : "undefined";
        if ( tinyutils::Logger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TBulletSingleBodyAdapter {0} @ {1}", name, tinyutils::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TBulletSingleBodyAdapter " << name << " @ " << tinyutils::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TBulletSingleBodyAdapter::Build()
    {
        auto collider = m_BodyRef->collider();
        LOCO_CORE_ASSERT( collider, "TBulletSingleBodyAdapter::Build >>> single-body {0} doesn't have \
                          a valid collider (nullptr)", m_BodyRef->name() );

        m_ColliderAdapter = std::make_unique<TBulletSingleBodyColliderAdapter>( collider );
        collider->SetColliderAdapter( m_ColliderAdapter.get() );

        auto bt_collider_adapter = static_cast<TBulletSingleBodyColliderAdapter*>( m_ColliderAdapter.get() );
        bt_collider_adapter->Build();

        auto bt_collision_shape = bt_collider_adapter->collision_shape();
        LOCO_CORE_ASSERT( bt_collision_shape, "TBulletSingleBodyAdapter::Build >>> single-body {0}'s \
                          collider (called {1}) doesn't have a valid btCollisionShape", m_BodyRef->name(), collider->name() );
        // Create rigid body (using maximal-coordinates bullet API) ****************************
        btScalar bt_mass = 0.0;
        btVector3 bt_inertia_diag( 0.0, 0.0, 0.0 );
        if ( m_BodyRef->dyntype() == eDynamicsType::DYNAMIC )
        {
            if ( collider->shape() != eShapeType::HFIELD && collider->shape() != eShapeType::PLANE )
            {
                if ( m_BodyRef->data().inertia.mass > loco::EPS )
                    bt_mass = m_BodyRef->data().inertia.mass;
                else
                    bt_mass = ComputeVolumeFromBtShape( bt_collision_shape ) * loco::DEFAULT_DENSITY;

                if ( bt_mass > loco::EPS )
                    bt_collision_shape->calculateLocalInertia( bt_mass, bt_inertia_diag );
            }
            else
            {
                LOCO_CORE_WARN( "TBulletSingleBodyAdapter::Build >>> single-body {0} has collider of \
                                 type {1}, but it can't be dynamic", m_BodyRef->name(), ToString( collider->shape() ) );
            }
        }

        if ( collider->shape() == eShapeType::HFIELD )
        {
            const auto& heights = collider->data().hfield_data.heights;
            const float z_max = *std::max_element( heights.cbegin(), heights.cend() );
            const float z_scale = collider->size().z();
            const float z_offset = z_max / 2.0f * z_scale * z_scale;
            m_HfieldTfCompensation.setOrigin( vec3_to_bt( { 0.0f, 0.0f, z_offset } ) );
            m_HfieldTfCompensationInv = m_HfieldTfCompensation.inverse();
        }

        btRigidBody::btRigidBodyConstructionInfo bt_construction_info( bt_mass, nullptr, bt_collision_shape, bt_inertia_diag );
        bt_construction_info.m_startWorldTransform = mat4_to_bt( m_BodyRef->tf0() ) * m_HfieldTfCompensation;

        m_BulletRigidBody = std::make_unique<btRigidBody>( bt_construction_info );
        m_BulletRigidBody->forceActivationState( DISABLE_DEACTIVATION );
        m_BulletRigidBody->setUserIndex( (int)eObjectType::SINGLE_BODY_COLLIDER );
        m_BulletRigidBody->setUserPointer( (void*) m_BodyRef->collider() );

        if ( auto constraint = m_BodyRef->constraint() )
        {
            const eConstraintType constraint_type = constraint->constraint_type();
            if ( constraint_type == eConstraintType::REVOLUTE )
            {
                m_ConstraintAdapter = std::make_unique<TBulletSingleBodyRevoluteConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto bt_constraint_adapter = dynamic_cast<TBulletSingleBodyRevoluteConstraintAdapter*>( m_ConstraintAdapter.get() );
                bt_constraint_adapter->SetBulletRigidBody( m_BulletRigidBody.get() );
                bt_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::PRISMATIC )
            {
                m_ConstraintAdapter = std::make_unique<TBulletSingleBodyPrismaticConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto bt_constraint_adapter = dynamic_cast<TBulletSingleBodyPrismaticConstraintAdapter*>( m_ConstraintAdapter.get() );
                bt_constraint_adapter->SetBulletRigidBody( m_BulletRigidBody.get() );
                bt_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::SPHERICAL )
            {
                m_ConstraintAdapter = std::make_unique<TBulletSingleBodySphericalConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto bt_constraint_adapter = dynamic_cast<TBulletSingleBodySphericalConstraintAdapter*>( m_ConstraintAdapter.get() );
                bt_constraint_adapter->SetBulletRigidBody( m_BulletRigidBody.get() );
                bt_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::TRANSLATIONAL3D )
            {
                m_ConstraintAdapter = std::make_unique<TBulletSingleBodyTranslational3dConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto bt_constraint_adapter = dynamic_cast<TBulletSingleBodyTranslational3dConstraintAdapter*>( m_ConstraintAdapter.get() );
                bt_constraint_adapter->SetBulletRigidBody( m_BulletRigidBody.get() );
                bt_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::UNIVERSAL3D )
            {
                m_ConstraintAdapter = std::make_unique<TBulletSingleBodyUniversal3dConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto bt_constraint_adapter = dynamic_cast<TBulletSingleBodyUniversal3dConstraintAdapter*>( m_ConstraintAdapter.get() );
                bt_constraint_adapter->SetBulletRigidBody( m_BulletRigidBody.get() );
                bt_constraint_adapter->Build();
            }
            else if ( constraint_type == eConstraintType::PLANAR )
            {
                m_ConstraintAdapter = std::make_unique<TBulletSingleBodyPlanarConstraintAdapter>( constraint );
                constraint->SetConstraintAdapter( m_ConstraintAdapter.get() );

                auto bt_constraint_adapter = dynamic_cast<TBulletSingleBodyPlanarConstraintAdapter*>( m_ConstraintAdapter.get() );
                bt_constraint_adapter->SetBulletRigidBody( m_BulletRigidBody.get() );
                bt_constraint_adapter->Build();
            }
            else
            {
                LOCO_CORE_ERROR( "TBulletSingleBodyAdapter::Build >>> constraint type {0} not supported", ToString( constraint_type ) );
            }
        }
    }

    void TBulletSingleBodyAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_BulletWorldRef, "TBulletSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid bullet-world reference", m_BodyRef->name() );
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        LOCO_CORE_ASSERT( m_ColliderAdapter, "TBulletSingleBodyAdapter::Initialize >>> body {0} must have \
                          a related bt-collider-adapter for its collider. Perhaps forgot to call ->Build()?", m_BodyRef->name() );

        auto collider = m_BodyRef->collider();
        if ( !collider )
            return;

        auto bt_collider_adapter = static_cast<TBulletSingleBodyColliderAdapter*>( m_ColliderAdapter.get() );
        bt_collider_adapter->SetBulletRigidBody( m_BulletRigidBody.get() );
        bt_collider_adapter->Initialize();

        const ssize_t collision_group = collider->collisionGroup();
        const ssize_t collision_mask = collider->collisionMask();
        m_BulletWorldRef->addRigidBody( m_BulletRigidBody.get(), collision_group, collision_mask );

        if ( m_BodyRef->constraint() )
        {
            m_ConstraintAdapter->Initialize();
        }
        else
        {
            m_BulletRigidBody->setLinearVelocity( vec3_to_bt( m_BodyRef->linear_vel0() ) );
            m_BulletRigidBody->setAngularVelocity( vec3_to_bt( m_BodyRef->angular_vel0() ) );
        }
    }

    void TBulletSingleBodyAdapter::Reset()
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::Reset >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        m_BulletRigidBody->setWorldTransform( mat4_to_bt( m_BodyRef->tf0() ) * m_HfieldTfCompensation );
        m_BulletRigidBody->forceActivationState( DISABLE_DEACTIVATION );
        if ( m_BodyRef->constraint() )
        {
            m_ConstraintAdapter->Reset();
        }
        else
        {
            m_BulletRigidBody->setLinearVelocity( vec3_to_bt( m_BodyRef->linear_vel0() ) );
            m_BulletRigidBody->setAngularVelocity( vec3_to_bt( m_BodyRef->angular_vel0() ) );
        }
    }

    void TBulletSingleBodyAdapter::SetTransform( const TMat4& transform )
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::SetTransform >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        m_BulletRigidBody->setWorldTransform( mat4_to_bt( transform ) * m_HfieldTfCompensation );
    }

    void TBulletSingleBodyAdapter::SetLinearVelocity( const TVec3& linear_vel )
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::SetLinearVelocity >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        m_BulletRigidBody->setLinearVelocity( vec3_to_bt( linear_vel ) );
    }

    void TBulletSingleBodyAdapter::SetAngularVelocity( const TVec3& angular_vel )
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::SetAngularVelocity >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        m_BulletRigidBody->setAngularVelocity( vec3_to_bt( angular_vel ) );
    }

    void TBulletSingleBodyAdapter::SetForceCOM( const TVec3& force_com )
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::SetForceCOM >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        m_BulletRigidBody->applyCentralForce( vec3_to_bt( force_com ) );
    }

    void TBulletSingleBodyAdapter::SetTorqueCOM( const TVec3& torque_com )
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::SetTorqueCOM >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        m_BulletRigidBody->applyTorque( vec3_to_bt( torque_com ) );
    }

    void TBulletSingleBodyAdapter::GetTransform( TMat4& dst_transform ) /* const */
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::GetTransform >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        dst_transform = mat4_from_bt( m_BulletRigidBody->getWorldTransform() * m_HfieldTfCompensationInv );
    }

    void TBulletSingleBodyAdapter::GetLinearVelocity( TVec3& dst_linear_vel ) /* const */
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::GetLinearVelocity >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        dst_linear_vel = vec3_from_bt( m_BulletRigidBody->getLinearVelocity() );
    }

    void TBulletSingleBodyAdapter::GetAngularVelocity( TVec3& dst_angular_vel ) /* const */
    {
        LOCO_CORE_ASSERT( m_BulletRigidBody, "TBulletSingleBodyAdapter::GetAngularVelocity >>> body {0} must have \
                          a valid bullet-rigid-body. Perhaps missing call to ->Build()?", m_BodyRef->name() );

        dst_angular_vel = vec3_from_bt( m_BulletRigidBody->getAngularVelocity() );
    }

    void TBulletSingleBodyAdapter::SetBulletWorld( btDynamicsWorld* world )
    {
        m_BulletWorldRef = world;
        if ( auto bt_collider_adapter = dynamic_cast<TBulletSingleBodyColliderAdapter*>( m_ColliderAdapter.get() ) )
            bt_collider_adapter->SetBulletWorld( world );

        if ( auto bt_constraint_adapter = dynamic_cast<TIBulletSingleBodyConstraintAdapter*>( m_ConstraintAdapter.get() ) )
            bt_constraint_adapter->SetBulletWorld( world );
    }
}}