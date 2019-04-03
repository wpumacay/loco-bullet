
#include <bullet_simulation.h>


namespace tysoc {
namespace bullet {

    TBtSimulation::TBtSimulation( TScenario* scenarioPtr,
                                  const std::string& workingDir )
        : TISimulation( scenarioPtr, workingDir )
    {

        m_btBroadphaseInterfacePtr      = new btDbvtBroadphase();
        m_btCollisionConfigurationPtr   = new btDefaultCollisionConfiguration();
        m_btCollisionDispatcherPtr      = new btCollisionDispatcher( m_btCollisionConfigurationPtr );
        m_btConstraintSolverPtr         = new btSequentialImpulseConstraintSolver();
        m_btWorldPtr                    = new btDiscreteDynamicsWorld(
                                                    m_btCollisionDispatcherPtr,
                                                    m_btBroadphaseInterfacePtr,
                                                    m_btConstraintSolverPtr,
                                                    m_btCollisionConfigurationPtr );

        m_btWorldPtr->setGravity( btVector3( 0, 0, -10 ) );

        m_runtimeType = "bullet";

        auto _sbodies = m_scenarioPtr->getBodies();

        for ( size_t q = 0; q < _sbodies.size(); q++ )
        {
            auto _bodyWrapper = new TBtBodyWrapper( _sbodies[q],
                                                    m_workingDir );

            m_bodyWrappers.push_back( _bodyWrapper );
        }

        auto _agents = m_scenarioPtr->getAgentsByType( agent::AGENT_TYPE_KINTREE );
        for ( size_t q = 0; q < _agents.size(); q++ )
        {
            auto _agentWrapper = new TBtKinTreeAgentWrapper( (agent::TAgentKinTree*) _agents[q],
                                                              m_workingDir );

            m_agentWrappers.push_back( _agentWrapper );
        }
    }

    TBtSimulation::~TBtSimulation()
    {
        // @WIP: see method "exitPhysics" in bullet example "CommonRigidBodyBase.h"

        if ( m_btWorldPtr )
        {
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
        /* Initialize wrappers (to create their internal resources) ***********/

        for ( size_t q = 0; q < m_bodyWrappers.size(); q++ )
        {
            auto _btBodyWrapper = reinterpret_cast< TBtBodyWrapper* >( m_bodyWrappers[q] );

            _btBodyWrapper->setBtWorld( m_btWorldPtr );
            _btBodyWrapper->initialize();
        }

        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
        {
            auto _btAgentWrapper = reinterpret_cast< TBtKinTreeAgentWrapper* >( m_agentWrappers[q] );

            _btAgentWrapper->setBtWorld( m_btWorldPtr );
            _btAgentWrapper->setParentSimulation( this );
            _btAgentWrapper->initialize();
        }

        /**********************************************************************/

        return true;
    }

    void TBtSimulation::_preStepInternal()
    {
        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
            m_terrainGenWrappers[q]->preStep();

        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
            m_agentWrappers[q]->preStep();

        for ( size_t q = 0; q < m_bodyWrappers.size(); q++ )
            m_bodyWrappers[q]->preStep();
    }

    void TBtSimulation::_simStepInternal()
    {
        if ( !m_btWorldPtr )
            return;

        m_btWorldPtr->stepSimulation( 1.0f / 60.0f );
    }

    void TBtSimulation::_postStepInternal()
    {
        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
            m_terrainGenWrappers[q]->postStep();

        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
            m_agentWrappers[q]->postStep();

        for ( size_t q = 0; q < m_bodyWrappers.size(); q++ )
            m_bodyWrappers[q]->postStep();
    }

    void TBtSimulation::_resetInternal()
    {
        for ( size_t q = 0; q < m_terrainGenWrappers.size(); q++ )
            m_terrainGenWrappers[q]->reset();

        for ( size_t q = 0; q < m_agentWrappers.size(); q++ )
            m_agentWrappers[q]->reset();

        for ( size_t q = 0; q < m_bodyWrappers.size(); q++ )
            m_bodyWrappers[q]->reset();
    }

    std::map< std::string, std::vector<TScalar> > TBtSimulation::_getVectorizedInfoInternal()
    {
        std::map< std::string, std::vector<TScalar> > _data;




        return _data;
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr,
                                                const std::string& workingDir )
    {
        std::cout << "INFO> creating bullet simulation" << std::endl;
        return new TBtSimulation( scenarioPtr, workingDir );
    }

}}