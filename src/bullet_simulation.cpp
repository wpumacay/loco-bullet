
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

    }

    TBtSimulation::~TBtSimulation()
    {
        // @WIP: see method "exitPhysics" in bullet example "CommonRigidBodyBase.h"

        for ( size_t q = 0; q < m_bodyWrappers.size(); q++ )
        {
            delete m_bodyWrappers[q];
            m_bodyWrappers[q] = NULL;
        }
        m_bodyWrappers.clear();

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

    }

    void TBtSimulation::_preStepInternal()
    {

    }

    void TBtSimulation::_simStepInternal()
    {

    }

    void TBtSimulation::_postStepInternal()
    {

    }

    void TBtSimulation::_resetInternal()
    {

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