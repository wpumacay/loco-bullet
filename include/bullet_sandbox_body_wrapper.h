
#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

#include <sandbox/body_wrapper.h>

#define CONSTRAINT_DISABLE_COLLISION_CONNECTED_BODIES true

namespace tysoc {
namespace bullet {


    class TBtBodyWrapper : public TBodyWrapper
    {

        private :

        btDiscreteDynamicsWorld* m_btWorldPtr;
        btCollisionShape* m_btCollisionShapePtr;
        btRigidBody* m_btRigidBodyPtr;

        std::map< std::string, btRigidBody* > m_btBodies;
        std::map< std::string, btTypedConstraint* > m_btConstraints;

        /**
        *   Process a body and creates the resources that this object contains.
        *
        *   @param bodyPtr  TBody object to be processed recursively
        */
        void _createBtResourcesFromBody( sandbox::TBody* bodyPtr, btRigidBody* parentBtBodyPtr );

        /**
        *   Creates a Bullet ridig body from the given body data
        *
        *   @param bodyPtr      TBody object that contains the data to be used
        */
        btRigidBody* _createBtRigidBody( sandbox::TBody* bodyPtr );

        /**
        *   Creates a Bullet collision shape from the given body data
        *
        *   @param bodyPtr      TBody object that contains the collision data
        */
        btCollisionShape* _createBtCollisionShape( sandbox::TBody* bodyPtr );

        /**
        *   Creates a Bullet constraint from the given joint data. Just one ...
        *   constraint is used to generate the dof that this body has, by using:
        *
        *   | Njoints |             Type(s)            |       Constraint       
        *   |---------|--------------------------------|------------------------ 
        *   |    0    |               N/A              |    btFixedConstraint   
        *   |    1    |              free              |    NULL (treated with care)
        *   |    1    |              hinge             |    btHingeConstraint   
        *   |    1    |              slide             |    btSliderConstraint  
        *   |    1    |               ball             |    btPoint2PointConstraint
        *   |    2+   |               any              |    btGeneric6DofSpring2Constraint                    
        *   |         |                                |                        
        *   @param joints   TJoint object that contains the data to be used
        */
        btTypedConstraint* _createBtConstraint( const std::vector< sandbox::TJoint* >& joints,
                                                btRigidBody* currentBtBodyPtr,
                                                btRigidBody* parentBtBodyPtr );

        // Helper constraint-creation methods

        btFixedConstraint* _createFixedConstraint( btRigidBody* currentBtBodyPtr,
                                                   btRigidBody* parentBtBodyPtr );

        btHingeConstraint* _createHingeConstraint( sandbox::TJoint* jointPtr,
                                                   btRigidBody* currentBtBodyPtr,
                                                   btRigidBody* parentBtBodyPtr );

        btSliderConstraint* _createSliderConstraint( sandbox::TJoint* jointPtr,
                                                     btRigidBody* currentBtBodyPtr,
                                                     btRigidBody* parentBtBodyPtr );

        btPoint2PointConstraint* _createPoint2PointConstraint( sandbox::TJoint* jointPtr,
                                                               btRigidBody* currentBtBodyPtr,
                                                               btRigidBody* parentBtBodyPtr );

        btGeneric6DofSpring2Constraint* _createGenericConstraintFromJoints( const std::vector< sandbox::TJoint* >& joints,
                                                                            btRigidBody* currentBtBodyPtr,
                                                                            btRigidBody* parentBtBodyPtr );

        /**
        *   Updates the body information recursively from the simulation
        *
        *   @param bodyPtr  The body we want to update its data
        */
        void _updateBodyRecursively( sandbox::TBody* bodyPtr );

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