#pragma once

#include <loco_common_bullet.h>
#include <primitives/loco_single_body_adapter.h>
#include <primitives/loco_single_body_collider_adapter_bullet.h>
#include <primitives/loco_single_body_constraint_adapter_bullet.h>

namespace loco {
    class TSingleBody;
}

namespace loco {
namespace bullet {

    class TBulletSingleBodyAdapter : public TISingleBodyAdapter
    {
    public :

        TBulletSingleBodyAdapter( TSingleBody* body_ref );

        TBulletSingleBodyAdapter( const TBulletSingleBodyAdapter& other ) = delete;

        TBulletSingleBodyAdapter& operator= ( const TBulletSingleBodyAdapter& other ) = delete;

        ~TBulletSingleBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetTransform( const TMat4& transform ) override;

        void SetLinearVelocity( const TVec3& linear_vel ) override;

        void SetAngularVelocity( const TVec3& angular_vel ) override;

        void SetForceCOM( const TVec3& force_com ) override;

        void SetTorqueCOM( const TVec3& torque_com ) override;

        void GetTransform( TMat4& dst_transform ) /* const */ override;

        void GetLinearVelocity( TVec3& dst_linear_vel ) override;

        void GetAngularVelocity( TVec3& dst_angular_vel ) override;

        void SetBulletWorld( btDynamicsWorld* world );

        btRigidBody* rigid_body() { return m_BulletRigidBody.get(); }

        const btRigidBody* rigid_body() const { return m_BulletRigidBody.get(); }

        btDynamicsWorld* world() { return m_BulletWorldRef; }

        const btDynamicsWorld* world() const { return m_BulletWorldRef; }

    private :

        // Bullet rigid-body object (handle to backend single-body)
        std::unique_ptr<btRigidBody> m_BulletRigidBody;
        // Reference to the bullet world used for the simulation
        btDynamicsWorld* m_BulletWorldRef;
        // Compensation-matrix for the case of heightfield colliders
        btTransform m_HfieldTfCompensation;
        // Inverse of the hfield compensation matrix
        btTransform m_HfieldTfCompensationInv;
    };

}}