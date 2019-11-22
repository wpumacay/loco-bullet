
#include <bullet_simulation.h>


namespace tysoc {
namespace bullet {

    TBtSimulation::TBtSimulation( TScenario* scenarioPtr )
        : TISimulation( scenarioPtr )
    {

        m_btBroadphaseInterfacePtr      = new btDbvtBroadphase();
        m_btCollisionConfigurationPtr   = new btDefaultCollisionConfiguration();
        m_btCollisionDispatcherPtr      = new btCollisionDispatcher( m_btCollisionConfigurationPtr );
        m_btConstraintSolverPtr         = new btMultiBodyConstraintSolver();
        m_btWorldPtr                    = new btMultiBodyDynamicsWorld(
                                                    m_btCollisionDispatcherPtr,
                                                    m_btBroadphaseInterfacePtr,
                                                    (btMultiBodyConstraintSolver*) m_btConstraintSolverPtr,
                                                    m_btCollisionConfigurationPtr );

        m_btDebugDrawer = new utils::TBtDebugDrawer();
        m_btWorldPtr->setDebugDrawer( m_btDebugDrawer );
        m_isDebugDrawingActive = true;

        // m_btFilterCallback = new utils::TBtOverlapFilterCallback();
        // m_btWorldPtr->getPairCache()->setOverlapFilterCallback( m_btFilterCallback );

        m_btWorldPtr->setGravity( btVector3( 0, 0, -9.81 ) );

        m_runtimeType = "bullet";

        auto _bodies = m_scenarioPtr->getBodies();
        for ( auto _body : _bodies )
        {
            auto _bodyAdapter = new TBtBodyAdapter( _body );
            _body->setAdapter( _bodyAdapter );

            m_bodyAdapters.push_back( _bodyAdapter );

            auto _collisions = _body->collisions();
            for ( auto _collision : _collisions )
            {
                auto _collisionAdapter = new TBtCollisionAdapter( _collision );
                _collision->setAdapter( _collisionAdapter );

                m_collisionAdapters.push_back( _collisionAdapter );
            }
        }

        // auto _agents = m_scenarioPtr->getAgents();
        // for ( size_t q = 0; q < _agents.size(); q++ )
        // {
        //     auto _agentWrapper = new TBtKinTreeAgentWrapper( (TAgent*) _agents[q],
        //                                                       m_workingDir );

        //     m_agentWrappers.push_back( _agentWrapper );
        // }

        // auto _terraingens = m_scenarioPtr->getTerrainGenerators();
        // for ( size_t q = 0; q < _terraingens.size(); q++ )
        // {
        //     auto _terrainGenWrapper = new TBtTerrainGenWrapper( _terraingens[q],
        //                                                          m_workingDir );

        //     m_terrainGenWrappers.push_back( _terrainGenWrapper );
        // }
    }

    TBtSimulation::~TBtSimulation()
    {
        // std::cout << "LOG> Calling bullet-simulation destructor" << std::endl;

        // @WIP: see method "exitPhysics" in bullet example "CommonRigidBodyBase.h"

        if ( m_btDebugDrawer )
        {
            delete m_btDebugDrawer;
            m_btDebugDrawer = NULL;
        }

        if ( m_btFilterCallback )
        {
            // @TODO: Should delete here?
            m_btFilterCallback = NULL;
        }

        if ( m_btWorldPtr )
        {
            // Remove all constraints
            for ( int q = m_btWorldPtr->getNumConstraints() - 1; q >= 0; q-- )
            {
                m_btWorldPtr->removeConstraint( m_btWorldPtr->getConstraint( q ) );
            }

            // Remove all
            for ( int q = m_btWorldPtr->getNumCollisionObjects() - 1; q >= 0; q-- )
            {
                btCollisionObject* _obj = m_btWorldPtr->getCollisionObjectArray()[q];
                btRigidBody* _body = btRigidBody::upcast( _obj );

                // if it's a rigid body (see upcast), then proceed
                if ( _body && _body->getMotionState() )
                {
                    delete _body->getMotionState();
                }
                m_btWorldPtr->removeCollisionObject( _obj );
                delete _obj;
            }
            
            delete m_btWorldPtr;
            m_btWorldPtr = NULL;
        }

        if ( m_btConstraintSolverPtr )
        {
            delete m_btConstraintSolverPtr;
            m_btConstraintSolverPtr = NULL;
        }

        if ( m_btBroadphaseInterfacePtr )
        {
            delete m_btBroadphaseInterfacePtr;
            m_btBroadphaseInterfacePtr = NULL;
        }

        if ( m_btCollisionDispatcherPtr )
        {
            delete m_btCollisionDispatcherPtr;
            m_btCollisionDispatcherPtr = NULL;
        }

        if ( m_btCollisionConfigurationPtr )
        {
            delete m_btCollisionConfigurationPtr;
            m_btCollisionConfigurationPtr = NULL;
        }
    }

    bool TBtSimulation::_initializeInternal()
    {
        /* Initialize and assemble low-level resources ********************************************/

        for ( auto _bodyAdapter : m_bodyAdapters )
            m_btWorldPtr->addRigidBody( dynamic_cast< TBtBodyAdapter* >( _bodyAdapter )->btRigidBodyPtr() );

        // for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
        // {
        //     auto _btAgentWrapper = reinterpret_cast< TBtKinTreeAgentWrapper* >( m_agentWrappers[q] );

        //     _btAgentWrapper->setBtWorld( m_btWorldPtr );
        //     _btAgentWrapper->setParentSimulation( this );
        //     _btAgentWrapper->initialize();
        //     _btAgentWrapper->finishedCreatingResources();
        // }

        // for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
        // {
        //     auto _btTerrainGenWrapper = reinterpret_cast< TBtTerrainGenWrapper* >( m_terrainGenWrappers[q] );

        //     _btTerrainGenWrapper->setBtWorld( m_btWorldPtr );
        //     _btTerrainGenWrapper->initialize();
        // }

        /******************************************************************************************/

        return true;
    }

    void TBtSimulation::_preStepInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }

    void TBtSimulation::_simStepInternal()
    {
        if ( !m_btWorldPtr )
            return;

        // for (int i = 0; i < m_btWorldPtr->getNumMultibodies(); i++)
        // {
        //     btMultiBody* mb = m_btWorldPtr->getMultiBody(i);
        //     for (int l = 0; l < mb->getNumLinks(); l++)
        //     {
        //         for (int d = 0; d < mb->getLink(l).m_dofCount; d++)
        //         {
        //             double damping_coefficient = 0.1;
        //             double damping = -damping_coefficient * mb->getJointVelMultiDof(l)[d];
        //             mb->addJointTorqueMultiDof(l, d, damping);
        //         }
        //     }
        // }

        m_btWorldPtr->stepSimulation( 1.0f / 60.0f );

        // @DEBUG: calls own debug drawing functionality
        if ( m_btWorldPtr && m_isDebugDrawingActive )
        {
            m_btDebugDrawer->setVisualizer( m_visualizerPtr );
            m_btWorldPtr->debugDrawWorld();
        }
    }

    void TBtSimulation::_postStepInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }

    void TBtSimulation::_resetInternal()
    {
        // do nothing here, as call to wrappers is enough (made in base)
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr )
    {
        std::cout << "INFO> creating bullet simulation" << std::endl;
        return new TBtSimulation( scenarioPtr );
    }

}}