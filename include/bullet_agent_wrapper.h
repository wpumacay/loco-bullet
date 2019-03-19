
#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

#include <agent_wrapper.h>

namespace tysoc {
namespace bullet {






    class TBtKinTreeAgentWrapper : public TKinTreeAgentWrapper
    {

        private :

        std::map< std::string, btRigidBody* > m_btBodies;
        std::map< std::string, btTypedConstraint* > m_btJoints;

        void _createBtResourcesFromKinTree();
        void _createBtResourcesFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr );

        /**
        *   Creates bullet btRidigBody from a given body in the core kintree
        *
        *   @param kinTreeBodyPtr   body node in the core kintree
        */
        btRigidBody* _processBody( agent::TKinTreeBody* kinTreeBodyPtr );

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
        void _processInertial( agent::TKinTreeInertia* kinTreeInertiaPtr,
                               btTransform& inertialFrame,
                               btVector3& inertiaDiag,
                               btScalar& inertiaMass );

        /**
        *   Creates a collision shape from a given collision object of the ...
        *   core kintree.
        *
        *   @param kinTreeCollisionPtr  collision object with the data to construct the shape
        */
        btCollisionShape* _processCollision( agent::TKinTreeCollision* kinTreeCollisionPtr );

        /**
        *   Creates a bullet joint constraint from a given joint object of the ...
        *   core kintree.
        *
        *   @param kinTreeJointPtr  joint object with the data to construct the shape
        *   @param childBodyPtr     the body this joint belongs to (adds a dof to the body)
        *   @param parentBpdyPtr    the parent of the given body, to which this joint connects it to
        */
        btTypedConstraint* _processJoint( agent::TKinTreeJoint* kinTreeJointPtr,
                                          agent::TKinTreeBody* kinTreeBodyPtr,
                                          agent::TKinTreeBody* parentBodyPtr );

        public :

        TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                const std::string& workingDir );
        ~TBtKinTreeAgentWrapper();


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