#pragma once

#include <loco_common_bullet.h>
#include <kinematic_trees/loco_kinematic_tree_collider_adapter.h>

namespace loco {
namespace kintree {
    class TKinematicTreeCollider;
}}

namespace loco {
namespace kintree {

    class TBulletKinematicTreeColliderAdapter : public TIKinematicTreeColliderAdapter
    {
    public :

        TBulletKinematicTreeColliderAdapter( TKinematicTreeCollider* collider_ref )
            : TIKinematicTreeColliderAdapter( collider_ref ) {}

        ~TBulletKinematicTreeColliderAdapter();

        void Build() override;

        void Initialize() override;

        void SetLocalTransform( const TMat4& local_tf ) override;

        void ChangeSize( const TVec3& new_size ) override;

        void ChangeCollisionGroup( int collision_group ) override;

        void ChangeCollisionMask( int collision_mask ) override;

        void ChangeFriction( const TScalar& friction ) override;

        btCollisionShape* collision_shape() { return m_BulletCollisionShape.get(); }

        const btCollisionShape* collision_shape() const { return m_BulletCollisionShape.get(); }

        btMultiBodyLinkCollider* link_collider() { return m_BulletLinkCollider.get(); }

        const btMultiBodyLinkCollider* link_collider() const { return m_BulletLinkCollider.get(); }

    private :

        // Current size (used for scaling when changing size of collider)
        TVec3 m_Size;
        // Initial size (used for scaling when changing size of collider)
        TVec3 m_Size0;
        // Current scale (used for scaling when changing size of collider)
        TVec3 m_Scale;

        std::unique_ptr<btCollisionShape> m_BulletCollisionShape;

        std::unique_ptr<btMultiBodyLinkCollider> m_BulletLinkCollider;
    };

}}