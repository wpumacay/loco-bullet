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

        void setDebugMode( int debug_mode_bits ) override { m_DebugMode = debug_mode_bits; };

        int getDebugMode() const override { return m_DebugMode; }

    private :

        /// Reference to the visualizer used for internal drawing calls
        TIVisualizer* m_VisualizerRef;
        /// Indicator of what can be drawn using the debug-drawer
        int m_DebugMode;
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

        btMultiBodyDynamicsWorld* bullet_world() { return m_BulletDynamicsWorld.get(); }

        const btMultiBodyDynamicsWorld* bullet_world() const { return m_BulletDynamicsWorld.get(); }

        btMultiBodyConstraintSolver* bullet_constraint_solver() { return m_BulletConstraintSolver.get(); }

        const btMultiBodyConstraintSolver* bullet_constraint_solver() const { return m_BulletConstraintSolver.get(); }

        btCollisionDispatcher* bullet_collision_dispatcher() { return m_BulletCollisionDispatcher.get(); }

        const btCollisionDispatcher* bullet_collision_dispatcher() const { return m_BulletCollisionDispatcher.get(); }

        btBroadphaseInterface* bullet_broadphase() { return m_BulletBroadphase.get(); }

        const btBroadphaseInterface* bullet_broadphase() const { return m_BulletBroadphase.get(); }

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal( const TScalar& dt ) override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

        void _SetTimeStepInternal( const TScalar& time_step ) override;

        void _SetGravityInternal( const TVec3& gravity ) override;

        void _SetVisualizerInternal( TIVisualizer* visualizerRef ) override;

    private :

        void _CreateSingleBodyAdapters();

        // void _CreateCompoundAdapters();

        // void _CreateKintreeAdapters();

        // void _CreateTerrainGeneratorAdapters();

        void _CollectContacts();

    private :

        /// Bullet's world used for the simulation
        std::unique_ptr<btMultiBodyDynamicsWorld> m_BulletDynamicsWorld = nullptr;
        /// Bullet's constraint solver used for the simulation
        std::unique_ptr<btMultiBodyConstraintSolver> m_BulletConstraintSolver = nullptr;
        /// Bullet's collision dispatcher used for collision detection
        std::unique_ptr<btCollisionDispatcher> m_BulletCollisionDispatcher = nullptr;
        /// Bullet's collision configuration used for collision detection
        std::unique_ptr<btCollisionConfiguration> m_BulletCollisionConfiguration = nullptr;
        /// Bullet's broadphase interface used for collision detection on the broadphase stage
        std::unique_ptr<btBroadphaseInterface> m_BulletBroadphase = nullptr;
        /// Bullet's overlap filter used for collision detection
        std::unique_ptr<btOverlapFilterCallback> m_BulletOverlapFilterCallback = nullptr;
        /// Bullet's debug drawer, used to visualizer various debugging graphics
        std::unique_ptr<btIDebugDraw> m_BulletDebugDrawer = nullptr;
        /// Maximum number of substeps to be taken during a simulation step
        ssize_t m_BulletMaxNumSubsteps = 10;
    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef );

}}