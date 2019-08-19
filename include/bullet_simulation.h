
#pragma once

#include <simulation_base.h>

#include <bullet_common.h>
#include <bullet_agent_wrapper.h>
#include <bullet_terrain_wrapper.h>

namespace tysoc {
namespace bullet {


    class TBtSimulation : public TISimulation
    {

        private:

        btMultiBodyDynamicsWorld*               m_btWorldPtr;
        btMultiBodyConstraintSolver*            m_btConstraintSolverPtr;
        btCollisionDispatcher*                  m_btCollisionDispatcherPtr;
        btDefaultCollisionConfiguration*        m_btCollisionConfigurationPtr;
        btBroadphaseInterface*                  m_btBroadphaseInterfacePtr;

        utils::TBtOverlapFilterCallback* m_btFilterCallback;
        utils::TBtDebugDrawer* m_btDebugDrawer;

        protected :

        bool _initializeInternal() override;
        void _preStepInternal() override;
        void _simStepInternal() override;
        void _postStepInternal() override;
        void _resetInternal() override;

        public :

        TBtSimulation( TScenario* scenarioPtr,
                       const std::string& workingDir );

        ~TBtSimulation();

    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr,
                                                const std::string& workingDir );

}}