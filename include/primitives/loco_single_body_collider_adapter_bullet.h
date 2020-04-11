#pragma once

#include <loco_common_bullet.h>
#include <primitives/loco_single_body_collider_adapter.h>

namespace loco {
    class TSingleBodyCollider;
}

namespace loco {
namespace bullet {

    const float LOCO_BULLET_HFIELD_BASE = 1.0f;

    class TBulletSingleBodyColliderAdapter : public TISingleBodyColliderAdapter
    {
    public :

        TBulletSingleBodyColliderAdapter( TSingleBodyCollider* collision_ref );

        TBulletSingleBodyColliderAdapter( const TBulletSingleBodyColliderAdapter& other ) = delete;

        TBulletSingleBodyColliderAdapter& operator= ( const TBulletSingleBodyColliderAdapter& other ) = delete;

        ~TBulletSingleBodyColliderAdapter();

        void Build() override;

        void Initialize() override;

        void ChangeSize( const TVec3& new_size ) override;

        void ChangeElevationData( const std::vector<float>& heights ) override;

        void ChangeCollisionGroup( int collisionGroup ) override;

        void ChangeCollisionMask( int collisionMask ) override;

        void SetBulletRigidBody( btRigidBody* rigid_body ) { m_BulletRigidBodyRef = rigid_body; }

        void SetBulletWorld( btDynamicsWorld* world ) { m_BulletWorldRef = world; }

        btCollisionShape* collision_shape() { return m_BulletCollisionShape.get(); }

        const btCollisionShape* collision_shape() const { return m_BulletCollisionShape.get(); }

        btRigidBody* rigid_body() { return m_BulletRigidBodyRef; }

        const btRigidBody* rigid_body() const { return m_BulletRigidBodyRef; }

        btDynamicsWorld* world() { return m_BulletWorldRef; }

        const btDynamicsWorld* world() const { return m_BulletWorldRef; }

    private :

        // Current size (used for scaling when changing size of collider)
        TVec3 m_Size;
        // Initial size (used for scaling when changing size of collider)
        TVec3 m_Size0;
        // Current scale (used for scaling when changing size of collider)
        TVec3 m_Scale;
        // Bullet collision-shape object (handle to backend collider)
        std::unique_ptr<btCollisionShape> m_BulletCollisionShape;
        // Reference to the bullet rigid-body that contains this collider
        btRigidBody* m_BulletRigidBodyRef;
        // Reference to the bullet world used for the simulation
        btDynamicsWorld* m_BulletWorldRef;
    };

}}