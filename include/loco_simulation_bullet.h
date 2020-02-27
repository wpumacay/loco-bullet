#pragma once

#include <loco_common_bullet.h>
#include <loco_simulation.h>
#include <typeinfo>

namespace loco {
namespace bullet {

    class TBulletSimulation : public TISimulation
    {
    public :

        TBulletSimulation( TScenario* scenarioRef );

        TBulletSimulation( const TBulletSimulation& other ) = delete;

        TBulletSimulation& operator=( const TBulletSimulation& other ) = delete;

        ~TBulletSimulation();

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal() override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

    private :

        std::unique_ptr<btMultiBodyDynamicsWorld>       m_bulletDynamicsWorld;
        std::unique_ptr<btMultiBodyConstraintSolver>    m_bulletConstraintSolver;
        std::unique_ptr<btCollisionDispatcher>          m_bulletCollisionDispatcher;
        std::unique_ptr<btCollisionConfiguration>       m_bulletCollisionConfiguration;
        std::unique_ptr<btBroadphaseInterface>          m_bulletBroadphase;
    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef );

}}