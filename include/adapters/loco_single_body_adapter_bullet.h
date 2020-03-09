#pragma once

#include <loco_common_bullet.h>
#include <adapters/loco_body_adapter.h>
#include <adapters/loco_collision_adapter_bullet.h>

namespace loco {
    class TIBody;
}

namespace loco {
namespace bullet {



    class TBulletSingleBodyAdapter : public TIBodyAdapter
    {
    public :

        TBulletSingleBodyAdapter( TIBody* bodyRef );

        TBulletSingleBodyAdapter( const TBulletSingleBodyAdapter& other ) = delete;

        TBulletSingleBodyAdapter& operator= ( const TBulletSingleBodyAdapter& other ) = delete;

        ~TBulletSingleBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void PreStep() override;

        void PostStep() override;

        void SetPosition( const TVec3& position ) override;

        void SetRotation( const TMat3& rotation ) override;

        void SetTransform( const TMat4& transform ) override;

        void GetPosition( TVec3& dstPosition ) /* const */ override;

        void GetRotation( TMat3& dstRotation ) /* const */ override;

        void GetTransform( TMat4& dstTransform ) /* const */ override;

        void SetLocalPosition( const TVec3& position ) override;

        void SetLocalRotation( const TMat3& rotation ) override;

        void SetLocalTransform( const TMat4& transform ) override;

        void GetLocalPosition( TVec3& dstPosition ) override;

        void GetLocalRotation( TMat3& dstRotation ) override;

        void GetLocalTransform( TMat4& dstTransform ) override;

        void SetBulletWorld( btDynamicsWorld* world ) { m_bulletWorldRef = world; }

        btRigidBody* rigid_body() { return m_bulletRigidBody.get(); }

        const btRigidBody* rigid_body() const { return m_bulletRigidBody.get(); }

        btDynamicsWorld* world() { return m_bulletWorldRef; }

        const btDynamicsWorld* world() const { return m_bulletWorldRef; }

    private :

        // Bullet rigid-body object (handle to backend single-body)
        std::unique_ptr<btRigidBody> m_bulletRigidBody;
        // Reference to the bullet world used for the simulation
        btDynamicsWorld* m_bulletWorldRef;
    };

}}