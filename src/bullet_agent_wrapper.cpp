
#pragma once

#include <bullet_agent_wrapper.h>

namespace tysoc {
namespace bullet {

    TBtKinTreeAgentWrapper::TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr )
        : TKinTreeAgentWrapper( kinTreeAgentPtr )
    {
        _createBtResourcesFromKinTree();
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
        _processInertial( kinTreeBodyPtr, _tfInertialLocalFrame, _inertiaDiag, _inertiaMass );

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






}}