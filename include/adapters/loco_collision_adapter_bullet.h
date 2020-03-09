#pragma once

#include <loco_common_bullet.h>
#include <adapters/loco_collision_adapter.h>

namespace loco {
    class TCollision;
}

namespace loco {
namespace bullet {

    const float LOCO_BULLET_HFIELD_BASE = 1.0f;

    class TBulletCollisionAdapter : public TICollisionAdapter
    {
    public :

        TBulletCollisionAdapter( TCollision* collisionRef );

        TBulletCollisionAdapter( const TBulletCollisionAdapter& other ) = delete;

        TBulletCollisionAdapter& operator= ( const TBulletCollisionAdapter& other ) = delete;

        ~TBulletCollisionAdapter();

        void Build() override;

        void Initialize() override;

        void PreStep() override;

        void PostStep() override;

        void Reset() override;

        void SetLocalPosition( const TVec3& position ) override;

        void SetLocalRotation( const TMat3& rotation ) override;

        void SetLocalTransform( const TMat4& transform ) override;

        void ChangeSize( const TVec3& newSize ) override;

        void ChangeElevationData( const std::vector<float>& heights ) override;

        void ChangeCollisionGroup( int collisionGroup ) override;

        void ChangeCollisionMask( int collisionMask ) override;

        void SetBulletRigidBody( btRigidBody* rigid_body ) { m_bulletRigidBodyRef = rigid_body; }

        void SetBulletWorld( btDynamicsWorld* world ) { m_bulletWorldRef = world; }

        btCollisionShape* collision_shape() { return m_bulletCollisionShape.get(); }

        const btCollisionShape* collision_shape() const { return m_bulletCollisionShape.get(); }

        btRigidBody* rigid_body() { return m_bulletRigidBodyRef; }

        const btRigidBody* rigid_body() const { return m_bulletRigidBodyRef; }

        btDynamicsWorld* world() { return m_bulletWorldRef; }

        const btDynamicsWorld* world() const { return m_bulletWorldRef; }

    private :

        // Current size (used for scaling when changing size of collider)
        TVec3 m_size;
        // Initial size (used for scaling when changing size of collider)
        TVec3 m_size0;
        // Current scale (used for scaling when changing size of collider)
        TVec3 m_scale;
        // Bullet collision-shape object (handle to backend collider)
        std::unique_ptr<btCollisionShape> m_bulletCollisionShape;
        // Reference to the bullet rigid-body that contains this collider
        btRigidBody* m_bulletRigidBodyRef;
        // Reference to the bullet world used for the simulation
        btDynamicsWorld* m_bulletWorldRef;
    };

}}