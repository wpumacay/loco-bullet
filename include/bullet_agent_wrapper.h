
#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

#include <agent_wrapper.h>

namespace tysoc {
namespace bullet {

    // @TODO|@MUST: Change the representation from single bodies to ...
    // Bullet's multibody, which seems to support the features ...
    // required for mjcf models as well (multi)

    class TBtKinTreeAgentWrapper : public TKinTreeAgentWrapper
    {

        private :

        btDiscreteDynamicsWorld* m_btWorldPtr;

        std::map< std::string, btRigidBody* > m_btBodies;
        std::map< std::string, btTypedConstraint* > m_btConstraints;

        void _createBtResourcesFromKinTree();
        void _createBtResourcesFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr,
                                             btRigidBody* parentBtBodyPtr );

        /**
        *   Creates bullet btRidigBody from a given body in the core kintree
        *
        *   @param kinTreeBodyPtr   body node in the core kintree
        */
        btRigidBody* _createBtRigidBody( agent::TKinTreeBody* kinTreeBodyPtr );

        /**
        *   Creates a bullet collision shape from the collision objects ...
        *   that the given kintreebody contains. The frames are compensated ...
        *   by the local inertial frame, if there exists inertial ...
        *   properties passed by the user for this body
        *
        *   @param kinTreeBodyPtr                   body object that contains the collision data
        *   @param compensateLocalInertialFrame     whether or not to use the compensated local frame
        *   @param inertialLocalFrame               local inertial frame to be used for compensation
        */
        btCollisionShape* _createBtCollisionShape( agent::TKinTreeBody* kinTreeBodyPtr,
                                                   bool compensateLocalInertialFrame,
                                                   const btTransform& inertialLocalFrame );

        /**
        *   Creates a collision shape from a given kintree collision object.
        *
        *   @param kinTreeCollisionPtr      collision object with the data to construct the shape
        */
        btCollisionShape* _createBtCollisionShapeSingle( agent::TKinTreeCollision* kinTreeCollisionPtr );

        /**
        *   Creates a bullet constraint from the given dofs that a kintree body contains
        *
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
        *
        *   @param kinTreeBodyPtr       body object with the data of the dofs that it contains
        *   @param currentBtBodyPtr     current bullet body object being processed
        *   @param parentBtBodyPtr      bullet body that is the parent of the current body
        */
        btTypedConstraint* _createBtConstraint( agent::TKinTreeBody* kinTreeBodyPtr,
                                                btRigidBody* currentBtBodyPtr,
                                                btRigidBody* parentBtBodyPtr );

        /**
        *   Computes inertia properties from the inertiañ data of a body in ...
        *   the core kintree. It computes the transform to a principal-axis, ...
        *   the diagonalized inertia matrix (diagonal elements) for those ...
        *   principal axes, and copies the mass as well.
        *
        *   @param kinTreeInertiaPtr    inertia data from a body of the core kintree
        *   @param inertialFrame        resulting local transform after diagonalizing for principal-axes
        *   @param inertiaMass          resulting mass copied from inertia properties
        */
        bool _computeInertialProperties( agent::TKinTreeBody* kinTreeBodyPtr,
                                         btTransform& inertialFrame,
                                         btVector3& inertiaDiag,
                                         btScalar& inertiaMass );

        /****** Helper constraint-creation methods ******/

        btGeneric6DofConstraint* _createFixedConstraint( agent::TKinTreeBody* kinTreeBodyPtr,
                                                         btRigidBody* currentBtBodyPtr,
                                                         btRigidBody* parentBtBodyPtr );

        btHingeConstraint* _createHingeConstraint( agent::TKinTreeJoint* kinTreeJointPtr,
                                                   btRigidBody* currentBtBodyPtr,
                                                   btRigidBody* parentBtBodyPtr );

        btSliderConstraint* _createSliderConstraint( agent::TKinTreeJoint* kinTreeJointPtr,
                                                     btRigidBody* currentBtBodyPtr,
                                                     btRigidBody* parentBtBodyPtr );

        btPoint2PointConstraint* _createPoint2PointConstraint( agent::TKinTreeJoint* kinTreeJointPtr,
                                                               btRigidBody* currentBtBodyPtr,
                                                               btRigidBody* parentBtBodyPtr );

        btGeneric6DofConstraint* _createGenericConstraintFromJoints( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                     btRigidBody* currentBtBodyPtr,
                                                                     btRigidBody* parentBtBodyPtr );

        protected :

        void _initializeInternal() override;
        void _resetInternal() override;
        void _preStepInternal() override;
        void _postStepInternal() override;

        public :

        TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                const std::string& workingDir );
        ~TBtKinTreeAgentWrapper();

        void setBtWorld( btDiscreteDynamicsWorld* btWorldPtr );
    };

    extern "C" TKinTreeAgentWrapper* agent_createFromAbstract( agent::TAgentKinTree* kinTreeAgentPtr,
                                                               const std::string& workingDir );

    extern "C" TKinTreeAgentWrapper* agent_createFromFile( const std::string& name,
                                                           const std::string& filename,
                                                           const std::string& workingDir );

    extern "C" TKinTreeAgentWrapper* agent_createFromId( const std::string& name,
                                                         const std::string& format,
                                                         const std::string& id,
                                                         const std::string& workingDir );

}}