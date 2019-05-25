
#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

#include <agent_wrapper.h>

#define TYSOC_BULLET_USE_MULTIBODY false

namespace tysoc {
namespace bullet {

    /**
    *   A wrapper for the links in the multibody that represents a ...
    *   kinematic tree.
    */
    class TBtMultiBodyLink
    {

        private :

        // Index of this link in the btMultiBody
        int m_btLinkIndx;

        // Bullet link to be wrapped
        btMultibodyLink* m_btLinkPtr;

        // Reference to the btMultiBody it belongs to
        btMultiBody* m_btMultiBodyPtr;

        // Reference to the kintree-collision object it relates to (non-dummy)
        agent::TKinTreeCollision* m_kinTreeCollisionPtr;

        public :

        TBtMultiBodyLink( int indx, 
                          btMultibodyLink* btLinkPtr,
                          btMultiBody* btMultiBodyPtr,
                          agent::TKinTreeCollision* kinTreeCollisionPtr = NULL );

        ~TBtMultiBodyLink();

        /**
        *   Returns the transform in worldspace of this link, using the ...
        *   btMultiBody reference for this computation.
        */
        TMat4 getWorldTransform();

        /**
        *   Returns the index of the wrapped btMultiBodyLink in the ... 
        *   btMultiBody it belongs to.
        */
        int getIndx();

        /**
        *   Returns the btMultiBodyLink reference being wrapped.
        */
        btMultibodyLink* ptrBtLink();

    };

    /* 
    *   This object encloses all collisions and fixed-constraints used while ...
    *   creating the resources from a kintree body node. Because the dynamics ...
    *   properties are not recomputed correctly (another approach would be to ...
    *   take into account each collision body, which actually corresponds to a ...
    *   dynamics body (if no inertia is given) and compensate the inertial frame ...
    *   of the actual (and only one) btbody with this correction terms )
    *
    *   For this approach we just create a single btMultiBodyLink for each kintree ...
    *   collision object we have and chain them with fixed constraints sequentially.
    *
    *   The DOFs in the body that represents the whole compound are used in the ...
    *   root of this chain
    */
    class TBodyCompound
    {
        private :

        // A reference to the actual kintree body node (that contains all required info)
        agent::TKinTreeBody* m_kinTreeBodyPtr;

        // A reference to the btMultiBody that represents the kintree
        btMultiBody* m_btMultiBodyPtr;

        // bodies used for each collision (i) in { 1 ... nb }
        std::vector< TBtMultiBodyLink* > m_linksInChain;

        // transforms for each body in the chain, wrt the previous body in the same chain
        std::vector< TMat4 > m_relTransformsInChain;

        // first link used in the chain
        TBtMultiBodyLink* m_startLink;

        // the transform from the 'kintree body frame' to the 'start link', fixed w.r.t body frame
        TMat4 m_startLinkToBaseTransform;

        // and the inverse of that, used every simulation step for some calculations
        TMat4 m_baseToStartLinkTransform;

        // last link used in the chain
        TBtMultiBodyLink* m_endLink;

        // the transform from the 'kintree body frame' to the 'end link', fixed w.r.t body frame
        TMat4 m_endLinkToBaseTransform;

        public :

        TBodyCompound( agent::TKinTreeBody* kinTreeBodyPtr,
                       btMultiBody* btMultiBodyPtr,
                       int parentLinkIndx );

        ~TBodyCompound();


    };

    class TBtKinTreeAgentWrapper : public TKinTreeAgentWrapper
    {

        private :

        // A reference to the world to add our resources to
        btMultiBodyDynamicsWorld* m_btWorldPtr;

        // A bullet multibody object that represents this kintree
        btMultiBody* m_btMultiBodyPtr;

        // The root compound for this kintree (recall body-node <> body-compound)
        TBodyCompound* m_rootCompound;

        // A storage for later access to each compound (by name)
        std::map< std::string, TBodyCompound* > m_bodyCompoundsMap;

        // A storage for later access to each compound (by index)
        std::vector< TBodyCompound* > m_bodyCompoundsArray;

        /**
        *   Start the creation process for all resources required by the ...
        *   kintree to be wrapped.
        */
        void _createBtResourcesFromKinTree();

        /**
        *   Creates a body compound out of a given kintree body object
        *
        *   @param kinTreeBodyPtr       body node from the core kintree
        *   @param parentBodyCompound   compound corresponding to the parent body
        */
        TBodyCompound* _createBodyCompoundFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr,
                                                        TBodyCompound* parentBodyCompound );


        protected :

        void _initializeInternal() override;
        void _resetInternal() override;
        void _preStepInternal() override;
        void _postStepInternal() override;

        public :

        TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                const std::string& workingDir );
        ~TBtKinTreeAgentWrapper();

        void setBtWorld( btMultiBodyDynamicsWorld* btWorldPtr );
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