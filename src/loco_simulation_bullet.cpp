
#include <loco_simulation_bullet.h>

namespace loco {
namespace bullet {

    /***********************************************************************************************
    *                                   Bullet DebugDrawer Impl.                                   *
    ***********************************************************************************************/

    TBulletDebugDrawer::TBulletDebugDrawer( TIVisualizer* visualizerRef )
        : m_VisualizerRef( visualizerRef )
    {
        LOCO_CORE_ASSERT( m_VisualizerRef, "TBulletDebugDrawer >>> visualizer-ref must be valid (given nullptr)" );
        m_DebugMode = btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawContactPoints;
    }

    TBulletDebugDrawer::~TBulletDebugDrawer()
    {
        m_VisualizerRef = nullptr;
    }

    void TBulletDebugDrawer::drawLine( const btVector3& from, const btVector3& to, const btVector3& color )
    {
        m_VisualizerRef->DrawLine( vec3_from_bt( from ), vec3_from_bt( to ), vec3_from_bt( color ) );
    }

    void TBulletDebugDrawer::drawContactPoint( const btVector3& point_on_b, const btVector3& normal_on_b,
                                               btScalar distance, int life_time, const btVector3& color )
    {
        m_VisualizerRef->DrawLine( vec3_from_bt( point_on_b ), vec3_from_bt( point_on_b + normal_on_b * distance ), vec3_from_bt( color ) );
        m_VisualizerRef->DrawLine( vec3_from_bt( point_on_b ), vec3_from_bt( point_on_b + normal_on_b * 0.01 ), { 0.0, 0.0, 0.0 } );
    }

    void TBulletDebugDrawer::reportErrorWarning( const char* warning_string )
    {
        LOCO_CORE_WARN( "TBulletDebugDrawer::reportErrorWarning >>> {0}", warning_string );
    }

    /***********************************************************************************************
    *                        Bullet custom overlap-filter callback Impl.                           *
    ***********************************************************************************************/

    bool TBulletOverlapFilterCallback::needBroadphaseCollision( btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1 ) const 
    {
        bool proxy_affinity_0_1 = ( proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask ) != 0;
        bool proxy_affinity_1_0 = ( proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask ) != 0;

        return proxy_affinity_0_1 || proxy_affinity_1_0;
    }

    /***********************************************************************************************
    *                                   Bullet Simulation Impl.                                    *
    ***********************************************************************************************/

    TBulletSimulation::TBulletSimulation( TScenario* scenarioRef )
        : TISimulation( scenarioRef )
    {
        m_BackendId = "BULLET";

        m_BulletBroadphase              = std::make_unique<btDbvtBroadphase>();
        m_BulletCollisionConfiguration  = std::make_unique<btDefaultCollisionConfiguration>();
        m_BulletCollisionDispatcher     = std::make_unique<btCollisionDispatcher>( m_BulletCollisionConfiguration.get() );
        m_BulletConstraintSolver        = std::make_unique<btMultiBodyConstraintSolver>();
        m_BulletDynamicsWorld           = std::make_unique<btMultiBodyDynamicsWorld>(
                                                                m_BulletCollisionDispatcher.get(),
                                                                m_BulletBroadphase.get(),
                                                                m_BulletConstraintSolver.get(),
                                                                m_BulletCollisionConfiguration.get() );
        m_BulletDynamicsWorld->setGravity( vec3_to_bt( m_Gravity ) );

        m_BulletOverlapFilterCallback = std::make_unique<TBulletOverlapFilterCallback>();
        m_BulletDynamicsWorld->getPairCache()->setOverlapFilterCallback( m_BulletOverlapFilterCallback.get() );

        _CreateSingleBodyAdapters();
        //// _CreateCompoundAdapters();
        //// _CreateKintreeAdapters();
        //// _CreateTerrainGeneratorAdapters();

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( tinyutils::Logger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TBulletSimulation @ {0}", tinyutils::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TBulletSimulation @ " << tinyutils::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TBulletSimulation::_CreateSingleBodyAdapters()
    {
        auto single_bodies = m_ScenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto single_body_adapter = std::make_unique<TBulletSingleBodyAdapter>( single_body );
            single_body->SetBodyAdapter( single_body_adapter.get() );
            m_SingleBodyAdapters.push_back( std::move( single_body_adapter ) );
        }
    }

    TBulletSimulation::~TBulletSimulation()
    {
        m_BulletDynamicsWorld = nullptr;
        m_BulletConstraintSolver = nullptr;
        m_BulletBroadphase = nullptr;
        m_BulletCollisionDispatcher = nullptr;
        m_BulletCollisionConfiguration = nullptr;
        m_BulletDebugDrawer = nullptr;
        m_BulletOverlapFilterCallback = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( tinyutils::Logger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TBulletSimulation @ {0}", tinyutils::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TBulletSimulation @ " << tinyutils::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    bool TBulletSimulation::_InitializeInternal()
    {
        for ( auto& single_body_adapter : m_SingleBodyAdapters )
            if ( auto bullet_adapter = dynamic_cast<TBulletSingleBodyAdapter*>( single_body_adapter.get() ) )
                bullet_adapter->SetBulletWorld( m_BulletDynamicsWorld.get() );

        LOCO_CORE_TRACE( "Bullet-backend >>> broadphase         : {0}", typeid( *m_BulletBroadphase ).name() );
        LOCO_CORE_TRACE( "Bullet-backend >>> collision config.  : {0}", typeid( *m_BulletCollisionConfiguration ).name() );
        LOCO_CORE_TRACE( "Bullet-backend >>> constraint solver  : {0}", typeid( *m_BulletConstraintSolver ).name() );
        LOCO_CORE_TRACE( "Bullet-backend >>> bullett-world      : {0}", typeid( *m_BulletDynamicsWorld ).name() );

        return true;
    }

    void TBulletSimulation::_PreStepInternal()
    {
        // Do nothing here, as call to wrappers is enough (made in base)
    }

    void TBulletSimulation::_SimStepInternal( const TScalar& dt )
    {
        LOCO_CORE_ASSERT( m_BulletDynamicsWorld, "TBulletSimulation::_SimStepInternal >>> \
                          btDynamicsWorld object is required, but got nullptr instead" );
        m_BulletDynamicsWorld->stepSimulation( dt, m_BulletMaxNumSubsteps, m_FixedTimeStep );
    }

    void TBulletSimulation::_PostStepInternal()
    {
        // @todo: run loco-contact-manager here to grab all detected contacts

        m_BulletDynamicsWorld->debugDrawWorld();
    }

    void TBulletSimulation::_ResetInternal()
    {
        // @todo: reset loco-contact-manager
    }

    void TBulletSimulation::_SetTimeStepInternal( const TScalar& time_step )
    {
        LOCO_CORE_ASSERT( m_BulletDynamicsWorld, "TBulletSimulation::_SetTimeStepInternal >>> \
                          btDynamicsWorld object is required, but got nullptr instead" );
        // Do nothing here, as the fixed timestep is set in each call to stepSimulation
    }

    void TBulletSimulation::_SetGravityInternal( const TVec3& gravity )
    {
        LOCO_CORE_ASSERT( m_BulletDynamicsWorld, "TBulletSimulation::_SetGravityInternal >>> \
                          btDynamicsWorld object is required, but got nullptr instead" );
        m_BulletDynamicsWorld->setGravity( vec3_to_bt( gravity ) );
    }

    void TBulletSimulation::_SetVisualizerInternal( TIVisualizer* visualizerRef )
    {
        m_BulletDebugDrawer = std::make_unique<TBulletDebugDrawer>( visualizerRef );
        m_BulletDynamicsWorld->setDebugDrawer( m_BulletDebugDrawer.get() );
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef )
    {
        return new loco::bullet::TBulletSimulation( scenarioRef );
    }

}}