#pragma once

#include <loco_common_bullet.h>
#include <loco_simulation.h>
#include <typeinfo>

#include <primitives/loco_single_body_collider_adapter_bullet.h>
#include <primitives/loco_single_body_adapter_bullet.h>

namespace loco {
    class TIVisualizer;
}

namespace loco {
namespace bullet {

    class TBulletDebugDrawer : public btIDebugDraw
    {
    public :

        TBulletDebugDrawer( TIVisualizer* visualizerRef );

        ~TBulletDebugDrawer();

        void drawLine( const btVector3& from, const btVector3& to, const btVector3& color ) override;

        void drawContactPoint( const btVector3& point_on_b, const btVector3& normal_on_b,
                               btScalar distance, int life_time, const btVector3& color ) override;

        void reportErrorWarning( const char* warning_string ) override;

        void draw3dText( const btVector3& location, const char* text_string ) override {}

        void setDebugMode( int debug_mode_bits ) override { m_debug_mode = debug_mode_bits; };

        int getDebugMode() const override { return m_debug_mode; }

    private :

        TIVisualizer* m_visualizerRef;

        int m_debug_mode;
    };

    struct TBulletOverlapFilterCallback : public btOverlapFilterCallback
    {
        // custom collision checking
        bool needBroadphaseCollision( btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1 ) const override;
    };

    class TBulletSimulation : public TISimulation
    {
    public :

        TBulletSimulation( TScenario* scenarioRef );

        TBulletSimulation( const TBulletSimulation& other ) = delete;

        TBulletSimulation& operator=( const TBulletSimulation& other ) = delete;

        ~TBulletSimulation();

        btMultiBodyDynamicsWorld* bullet_world() { return m_bulletDynamicsWorld.get(); }

        const btMultiBodyDynamicsWorld* bullet_world() const { return m_bulletDynamicsWorld.get(); }

        btMultiBodyConstraintSolver* bullet_constraint_solver() { return m_bulletConstraintSolver.get(); }

        const btMultiBodyConstraintSolver* bullet_constraint_solver() const { return m_bulletConstraintSolver.get(); }

        btCollisionDispatcher* bullet_collision_dispatcher() { return m_bulletCollisionDispatcher.get(); }

        const btCollisionDispatcher* bullet_collision_dispatcher() const { return m_bulletCollisionDispatcher.get(); }

        btBroadphaseInterface* bullet_broadphase() { return m_bulletBroadphase.get(); }

        const btBroadphaseInterface* bullet_broadphase() const { return m_bulletBroadphase.get(); }

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal() override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

        void _SetVisualizerInternal( TIVisualizer* visualizerRef ) override;

    private :

        void _CreateSingleBodyAdapters();

        // void _CreateCompoundAdapters();

        // void _CreateKintreeAdapters();

        // void _CreateTerrainGeneratorAdapters();

    private :

        std::unique_ptr<btMultiBodyDynamicsWorld>       m_bulletDynamicsWorld;
        std::unique_ptr<btMultiBodyConstraintSolver>    m_bulletConstraintSolver;
        std::unique_ptr<btCollisionDispatcher>          m_bulletCollisionDispatcher;
        std::unique_ptr<btCollisionConfiguration>       m_bulletCollisionConfiguration;
        std::unique_ptr<btBroadphaseInterface>          m_bulletBroadphase;
        std::unique_ptr<btIDebugDraw>                   m_bulletDebugDrawer;
        std::unique_ptr<btOverlapFilterCallback>        m_bulletOverlapFilterCallback;
    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef );

}}