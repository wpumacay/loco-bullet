
#include <kinematic_trees/loco_kinematic_trees_collider_adapter_bullet.h>

namespace loco {
namespace kintree {

    TBulletKinematicTreeColliderAdapter::TBulletKinematicTreeColliderAdapter( TKinematicTreeCollider* collider_ref )
        : TIKinematicTreeColliderAdapter( collider_ref )
        {
            m_Size  = m_ColliderRef->size();
            m_Size0 = m_ColliderRef->size();
            m_Scale = { 1.0f, 1.0f, 1.0f };
        }

    TBulletKinematicTreeColliderAdapter::~TBulletKinematicTreeColliderAdapter()
    {
        m_BulletCollisionShape = nullptr;
        m_BulletLinkCollider = nullptr;
    }

    void TBulletKinematicTreeColliderAdapter::Build()
    {
        m_BulletCollisionShape = bullet::CreateCollisionShape( m_ColliderRef->data() );
        m_BulletCollisionShape->setMargin( 0.001 );

        m_Size  = m_ColliderRef->size();
        m_Size0 = m_ColliderRef->size();
        m_Scale = { 1.0f, 1.0f, 1.0f };
    }

    void TBulletKinematicTreeColliderAdapter::Initialize()
    {

    }

    void TBulletKinematicTreeColliderAdapter::SetLocalTransform( const TMat4& local_tf )
    {

    }

    void TBulletKinematicTreeColliderAdapter::ChangeSize( const TVec3& new_size )
    {
        if ( !m_BulletCollisionShape )
            return;

        m_Size = newSize;
        switch ( m_ColliderRef->shape() )
        {
            case eShapeType::PLANE :
            {
                m_Scale.x() = m_Size.x() / m_Size0.x();
                m_Scale.y() = m_Size.y() / m_Size0.y();
                m_Scale.z() = 1.0f;
                break;
            }
            case eShapeType::BOX :
            {
                m_Scale.x() = m_Size.x() / m_Size0.x();
                m_Scale.y() = m_Size.y() / m_Size0.y();
                m_Scale.z() = m_Size.z() / m_Size0.z();
                break;
            }
            case eShapeType::SPHERE :
            {
                const auto scale = m_Size.x() / m_Size0.x();
                m_Scale.x() = scale;
                m_Scale.y() = scale;
                m_Scale.z() = scale;
                break;
            }
            case eShapeType::CYLINDER :
            {
                const auto scale_radius = m_Size.x() / m_Size0.x();
                const auto scale_height = m_Size.y() / m_Size0.y();
                m_Scale.x() = scale_radius;
                m_Scale.y() = scale_radius;
                m_Scale.z() = scale_height;
                break;
            }
            case eShapeType::CAPSULE :
            {
                const auto scale_radius = m_Size.x() / m_Size0.x();
                const auto scale_height = m_Size.y() / m_Size0.y();
                m_Scale.x() = scale_radius;
                m_Scale.y() = scale_radius;
                m_Scale.z() = scale_height;
                break;
            }
            case eShapeType::CONVEX_MESH :
            case eShapeType::TRIANGULAR_MESH :
            {
                m_Scale.x() = m_Size.x() / m_Size0.x();
                m_Scale.y() = m_Size.y() / m_Size0.y();
                m_Scale.z() = m_Size.z() / m_Size0.z();
                break;
            }
            case eShapeType::HEIGHTFIELD :
            {
                m_Scale.x() = m_Size.x() / m_Size0.x();
                m_Scale.y() = m_Size.y() / m_Size0.y();
                m_Scale.z() = m_Size.z() / m_Size0.z();
                break;
            }
        }

        m_BulletCollisionShape->setLocalScaling( bullet::vec3_to_bt( m_Scale ) );
    }

    void TBulletKinematicTreeColliderAdapter::ChangeCollisionGroup( int collision_group )
    {
        m_BulletLinkCollider->getBroadphaseHandle()->m_collisionFilterGroup = collision_group;
    }

    void TBulletKinematicTreeColliderAdapter::ChangeCollisionMask( int collision_mask )
    {
        m_BulletLinkCollider->getBroadphaseHandle()->m_collisionFilterMask = collision_mask;
    }

    void TBulletKinematicTreeColliderAdapter::ChangeFriction( const TScalar& friction )
    {
        m_BulletLinkCollider->setFriction( friction );
    }
}}