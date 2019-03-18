
#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

#include <sandbox/body_wrapper.h>

namespace tysoc {
namespace bullet {


    class TBtBodyWrapper : public TBodyWrapper
    {

        private :

        btDiscreteDynamicsWorld* m_btWorldPtr;
        btCollisionShape* m_btCollisionShapePtr;
        btRigidBody* m_btRigidBodyPtr;

        void _createBtCollisionShape();
        void _createBtRigidBody();

        protected :

        void _initializeInternal() override;
        void _resetInternal() override;
        void _preStepInternal() override;
        void _postStepInternal() override;
        void _changePositionInternal() override;
        void _changeRotationInternal() override;
        void _changeSizeInternal() override;

        public :

        TBtBodyWrapper( sandbox::TBody* bodyPtr,
                        const std::string& workingDir );

        ~TBtBodyWrapper();

        void setBtWorld( btDiscreteDynamicsWorld* btWorldPtr );

    };




}}