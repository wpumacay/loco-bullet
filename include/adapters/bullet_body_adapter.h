#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

#include <adapters/body_adapter.h>
#include <adapters/bullet_collision_adapter.h>

namespace tysoc {

    class TBtBodyAdapter : public TIBodyAdapter
    {

    public :

        TBtBodyAdapter( TBody* bodyPtr );

        ~TBtBodyAdapter();

        void build() override;

        void reset() override;

        void update() override;

        void setPosition( const TVec3& position ) override;

        void setRotation( const TMat3& rotation ) override;

        void setTransform( const TMat4& transform ) override;

        void getPosition( TVec3& dstPosition ) override;

        void getRotation( TMat3& dstRotation ) override;

        void getTransform( TMat4& dstTransform ) override;

        btRigidBody* btRigidBodyPtr() const { return m_btRigidBody.get(); }

        btCompoundShape* btCompoundShapePtr() const { return m_btCompoundShape.get(); }

    private :

        std::unique_ptr< btRigidBody > m_btRigidBody;
        std::unique_ptr< btCompoundShape > m_btCompoundShape;

    };

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr );

}