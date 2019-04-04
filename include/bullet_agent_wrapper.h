
#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

#include <agent_wrapper.h>

#define TYSOC_BULLET_USE_MULTIBODY false

namespace tysoc {
namespace bullet {

    // @TODO|@MUST: Change the representation from single bodies to ...
    // Bullet's multibody, which seems to support the features ...
    // required for mjcf models as well (multi)

    /* 
    *   This object encloses all collisions and fixed-constraints used while ...
    *   creating the resources from a kintree body node. Because the dynamics ...
    *   properties are not recomputed correctly (another approach would be to ...
    *   take into account each collision body, which actually corresponds to a ...
    *   dynamics body (if no inertia is given) and compensate the inertial frame ...
    *   of the actual (and only one) btbody with this correction terms )
    *
    *   For this approach we just create a single btbulletbody for each kintree ...
    *   collision object we have, chain them with fixed constraints sequentially, ...
    *   and update the kintree body worldTransform
    */
    struct TBtBodyCompound
    {
        // A reference to the actual kintree body node (that contains all required info)
        agent::TKinTreeBody* kinTreeBodyPtr;

        // bodies used for each collision (i) in { 1 ... nb }
        std::vector< btRigidBody* > btBodiesInChain;
         // fixed constraints used to link body (i) to body (i+1), for i in { 2 ... nb }
        std::vector< btTypedConstraint* > btConstraintsInChain;
        // transforms for each body in the chain, wrt the previous body in the same chain
        std::vector< TMat4 > relTransformsInChain;

        // first body used in the chain
        btRigidBody* btStartBody;
        // the transform from the 'kintree body frame' to the 'start body', fixed wrt body frame
        TMat4 startBodyToBaseTransform;
        // and the inverse of that, used every simulation step for calculations
        TMat4 baseToStartBodyTransform;
        // last body used in the chain
        btRigidBody* btEndBody;
        // the transform from the 'kintree body frame' to the 'end body', fixed wrt body frame
        TMat4 endBodyToBaseTransform;
    };

    class TBtKinTreeAgentWrapper : public TKinTreeAgentWrapper
    {

        private :

        btDiscreteDynamicsWorld* m_btWorldPtr;

        TBtBodyCompound* m_rootCompound;
        std::map< std::string, TBtBodyCompound* > m_bodyCompounds;

        void _createBtResourcesFromKinTree();

        /**
        *   Creates a bullet btRidigBody from a given collision object. The ...
        *   object created is expected to be used by a chain in a compound.
        *
        *   @param kinTreeCollisionPtr   collision object from a given kintree body node
        */
        btRigidBody* _createBtRigidBodyForChain( agent::TKinTreeCollision* kinTreeCollisionPtr );

        /**
        *   Creates a body compound out of a given kintree body object
        *
        *   @param kinTreeBodyPtr       body node from the core kintree
        *   @param parentBodyCompound   compound corresponding to the parent body
        */
        TBtBodyCompound* _createBtCompoundFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr,
                                                        TBtBodyCompound* parentBodyCompound );

        /**
        *   Computes the volume of the given shape
        *
        *   @param collisionShapePtr    collision shape from which we want to compute its volume
        *   @param frameTransform       frame of reference to use for AABB calculation, if needed
        */
        btScalar _computeVolumeFromShape( btCollisionShape* collisionShapePtr,
                                          const btTransform& frameTransform );
        /**
        *   Creates a collision shape from a given kintree collision object.
        *
        *   @param kinTreeCollisionPtr      collision object with the data to construct the shape
        */
        btCollisionShape* _createBtCollisionShapeSingle( agent::TKinTreeCollision* kinTreeCollisionPtr );

        /**
        *   Creates a bullet constraint from the given dofs that a kintree body contains.
        *   The requirements are extracted from the compounds passed as parameters (non-multibody mode)
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
        btTypedConstraint* _createNodeBtConstraintFromCompound( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                TBtBodyCompound* currentBodyCompound,
                                                                TBtBodyCompound* parentBodyCompound );

        /****** Helper constraint-creation methods ******/

        btTypedConstraint* _createFixedConstraint( btRigidBody* currentBtBodyPtr,
                                                   btRigidBody* parentBtBodyPtr,
                                                   const TMat4& currentToParentTransform );

        btHingeConstraint* _createHingeConstraint( btRigidBody* currentBtBodyPtr,
                                                   btRigidBody* parentBtBodyPtr,
                                                   const TVec3& pivotInCurrent,
                                                   const TVec3& axisInCurrent,
                                                   const TMat4& currentToParentTransform,
                                                   const TVec2& limits );

        btSliderConstraint* _createSliderConstraint( btRigidBody* currentBtBodyPtr,
                                                     btRigidBody* parentBtBodyPtr,
                                                     const TVec3& pivotInCurrent,
                                                     const TVec3& axisInCurrent,
                                                     const TMat4& currentToParentTransform,
                                                     const TVec2& limits );

        btPoint2PointConstraint* _createPoint2PointConstraint( btRigidBody* currentBtBodyPtr,
                                                               btRigidBody* parentBtBodyPtr,
                                                               const TVec3& pivotInCurrent,
                                                               const TMat4& currentToParentTransform );

        btGeneric6DofConstraint* _createGenericConstraintFromJoints( btRigidBody* currentBtBodyPtr,
                                                                     btRigidBody* parentBtBodyPtr,
                                                                     const std::vector< agent::TKinTreeJoint* >& joints,
                                                                     const TMat4& currentToParentTransform );

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