
#pragma once

#include <tysocBulletCommon.h>
#include <utils/primitives_spawner.h>


namespace tysocBullet
{

    struct TBulletDebugPrimitive : tysocUtils::TDebugPrimitive
    {
        btRigidBody* rigidBody;
    };


    class TBulletPrimitivesSpawner : public tysocUtils::TPrimitivesSpawner
    {

        private :

        btDynamicsWorld* m_bulletWorldPtr;

        btRigidBody* _createRigidBody( const std::string& shape,
                                       float sx, float sy, float sz,
                                       float x, float y, float z );

        protected :

        tysocUtils::TDebugPrimitive* _createPrimitiveInternal( const std::string& type,
                                                   float sx, float sy, float sz,
                                                   float x, float y, float z ) override;
        void _recyclePrimitiveInternal( tysocUtils::TDebugPrimitive* primitivePtr ) override;
        void _activatePrimitiveInternal( tysocUtils::TDebugPrimitive* primitivePtr,
                                         float sx, float sy, float sz,
                                         float x, float y, float z ) override;
        void _updatePrimitiveInternal( tysocUtils::TDebugPrimitive* primitivePtr ) override;

        public :

        TBulletPrimitivesSpawner( btDynamicsWorld* worldPtr );
        ~TBulletPrimitivesSpawner();

    };



}