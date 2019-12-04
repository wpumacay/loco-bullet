#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

#include <adapters/collision_adapter.h>

#define BT_COLLISION_DEFAULT_HFIELD_BASE 1.0f

namespace tysoc
{

    class TCollision;

    class TBtCollisionAdapter : public TICollisionAdapter
    {

    public :

        TBtCollisionAdapter( TCollision* collision );

        ~TBtCollisionAdapter();

        void build() override;

        void reset() override;

        void update() override;

        void setLocalPosition( const TVec3& position ) override;

        void setLocalRotation( const TMat3& rotation ) override;

        void setLocalTransform( const TMat4& transform ) override;

        void changeSize( const TVec3& newSize ) override;

        void changeElevationData( const std::vector< float >& heightData ) override;

        btCollisionShape* collisionShape() const { return m_btCollisionShape.get(); }

    private :

        /* bullet collision shape handled by this adapter */
        std::unique_ptr< btCollisionShape > m_btCollisionShape;

        /* current size (used for scaling when changing size of collider) */
        TVec3 m_size;

        /* initial size (used for scaling when changing size of collider) */
        TVec3 m_size0;

        /* current scale (used for scaling when changing size of collider) */
        TVec3 m_scale;

        /* type of shape that this drawable represents */
        eShapeType  m_type;

    };

    extern "C" TICollisionAdapter* simulation_createCollisionAdapter( TCollision* collision );
}