
#include <bullet_agent_wrapper.h>

namespace tysoc {
namespace bullet {

    TBtKinTreeAgentWrapper::TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                                    const std::string& workingDir )
        : TKinTreeAgentWrapper( kinTreeAgentPtr, workingDir )
    {
        _createBtResourcesFromKinTree();
    }

    TBtKinTreeAgentWrapper::~TBtKinTreeAgentWrapper()
    {
        // @WIP
    }

    void TBtKinTreeAgentWrapper::_createBtResourcesFromKinTree()
    {
        if ( !m_kinTreeAgentPtr )
            return;

        auto _rootBodyPtr = m_kinTreeAgentPtr->getRootBody();
        _createBtResourcesFromBodyNode( _rootBodyPtr );
    }

    void TBtKinTreeAgentWrapper::_createBtResourcesFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr )
    {
        /* @DOC|@JUPYTER: */
        // process inertial
        btTransform _tfInertialLocalFrame;
        btVector3 _inertiaDiag;
        btScalar _inertiaMass;
        // _processInertial( kinTreeBodyPtr, _tfInertialLocalFrame, _inertiaDiag, _inertiaMass );

        /* @DOC|@JUPYTER: */
        // process collisions
        auto _compoundCollisionShape = new btCompoundShape();
        auto _collisions = kinTreeBodyPtr->childCollisions;
        for ( size_t i = 0; i < _collisions.size(); i++ )
        {
            auto _collisionShape = _processCollision( _collisions[i] );
            if ( !_collisionShape )
                continue;

            /* @DOC|@JUPYTER: */
            // construct the transform from the collision frame to the inertial frame
            auto _tfCollisionToBody = utils::toBtTransform( _collisions[i]->relTransform );
            auto _tfCollisionToInertial = _tfInertialLocalFrame.inverse() * _tfCollisionToBody;
            // and add the shape to the compound according to this transform
            _compoundCollisionShape->addChildShape( _tfCollisionToInertial,
                                                    _collisionShape );
        }

        /* @DOC|@JUPYTER: */
        // process joints
        auto _joints = kinTreeBodyPtr->childJoints;
        if ( _joints.size() < 1 )
        {
            // No degrees of freedom, so create a fixed constraint

        }
        else if ( _joints.size() == 1 )
        {
            // There is a joint to define the degrees of freedom, so it should ...
            // be one of these cases: hinge|revolute|continuous, slide|prismatic, ...
            // ball|spherical or free|floating
        }
        else
        {
            // There are various dofs specified by various joints, so use a ...
            // generic 6dof constraint, and force the axis|limits accordingly
        }

        for ( size_t i = 0; i < kinTreeBodyPtr->childBodies.size(); i++ )
            _createBtResourcesFromBodyNode( kinTreeBodyPtr->childBodies[i] );
    }


    void TBtKinTreeAgentWrapper::_processInertial( agent::TKinTreeInertia* kinTreeInertiaPtr,
                                                   btTransform& inertialFrame,
                                                   btVector3& inertiaDiag,
                                                   btScalar& inertiaMass )
    {
        // @WIP
    }

    btCollisionShape* TBtKinTreeAgentWrapper::_processCollision( agent::TKinTreeCollision* kinTreeCollisionPtr )
    {
        // @WIP
        return NULL;
    }

    btTypedConstraint* TBtKinTreeAgentWrapper::_processJoint( agent::TKinTreeJoint* kinTreeJointPtr,
                                                              agent::TKinTreeBody* kinTreeBodyPtr,
                                                              agent::TKinTreeBody* parentBodyPtr )
    {
        // @WIP
        return NULL;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromAbstract( agent::TAgentKinTree* kinTreeAgentPtr,
                                                               const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromFile( const std::string& name,
                                                           const std::string& filename,
                                                           const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromId( const std::string& name,
                                                         const std::string& format,
                                                         const std::string& id,
                                                         const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }


}}