#pragma once

#include <loco_common_bullet.h>
#include <kinematic_trees/loco_kinematic_tree_joint_adapter.h>

namespace loco {
namespace kintree {
    class TKinematicTreeJoint;
}}

namespace loco {
namespace kintree {

    class TBulletKinematicTreeJointAdapter : public TIKinematicTreeJointAdapter
    {
    public :

        TBulletKinematicTreeJointAdapter( TKinematicTreeJoint* joint_ref )
            : TIKinematicTreeJointAdapter( joint_ref ) {}

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetQpos( const std::vector<TScalar>& qpos ) override;

        void SetQvel( const std::vector<TScalar>& qvel ) override;

        void SetLocalTransform( const TMat4& local_tf ) override;

        void ChangeStiffness( const TScalar& stiffness ) override;

        void ChangeArmature( const TScalar& armature ) override;

        void ChangeDamping( const TScalar& damping ) override;

        void ChangeAxis( const TVec3& axis ) override;

        void ChangeLimits( const TVec2& limits ) override;

        void GetQpos( std::vector<TScalar>& dst_qpos ) override;

        void GetQvel( std::vector<TScalar>& dst_qvel ) override;

        void SetBulletMultibody( btMultibody* multibody_ref );

        void SetBulletLinkIndex( ssize_t index );

        btMultibody* multibody() { return m_BulletMultibodyRef; }

        const btMultibody* multibody() const { return m_BulletMultibodyRef; }

        ssize_t link_index() const { return m_BulletLinkIndex; }

    private :

        // Reference to the bullet articulated-system resource
        btMultibody* m_BulletMultibodyRef = nullptr;
        // Index of the link associated with this joint
        ssize_t m_BulletLinkIndex = -1;
    };
}}