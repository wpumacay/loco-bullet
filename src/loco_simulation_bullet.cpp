
#include <loco_simulation_bullet.h>

namespace loco {
namespace bullet {

    /***********************************************************************************************
    *                                   Bullet DebugDrawer Impl.                                   *
    ***********************************************************************************************/

    TBulletDebugDrawer::TBulletDebugDrawer( TIVisualizer* visualizerRef )
        : m_visualizerRef( visualizerRef )
    {
        LOCO_CORE_ASSERT( m_visualizerRef, "TBulletDebugDrawer >>> visualizer-ref must be valid (given nullptr)" );

        m_debug_mode = btIDebugDraw::DBG_DrawWireframe |
                       btIDebugDraw::DBG_DrawConstraints |
                       btIDebugDraw::DBG_DrawContactPoints;
    }

    TBulletDebugDrawer::~TBulletDebugDrawer()
    {
        m_visualizerRef = nullptr;
    }

    void TBulletDebugDrawer::drawLine( const btVector3& from, const btVector3& to, const btVector3& color )
    {
        m_visualizerRef->DrawLine( vec3_from_bt( from ), vec3_from_bt( to ), vec3_from_bt( color ) );
    }

    void TBulletDebugDrawer::drawContactPoint( const btVector3& point_on_b, const btVector3& normal_on_b,
                                               btScalar distance, int life_time, const btVector3& color )
    {
        m_visualizerRef->DrawLine( vec3_from_bt( point_on_b ), vec3_from_bt( point_on_b + normal_on_b * distance ), vec3_from_bt( color ) );
        m_visualizerRef->DrawLine( vec3_from_bt( point_on_b ), vec3_from_bt( point_on_b + normal_on_b * 0.01 ), { 0.0, 0.0, 0.0 } );
    }

    void TBulletDebugDrawer::reportErrorWarning( const char* warning_string )
    {
        LOCO_CORE_WARN( "TBulletDebugDrawer::reportErrorWarning >>> {0}", warning_string );
    }

    /***********************************************************************************************
    *                                   Bullet Simulation Impl.                                    *
    ***********************************************************************************************/

    TBulletSimulation::TBulletSimulation( TScenario* scenarioRef )
        : TISimulation( scenarioRef )
    {
        m_backendId = "BULLET";

        m_bulletBroadphase              = std::make_unique<btDbvtBroadphase>();
        m_bulletCollisionConfiguration  = std::make_unique<btDefaultCollisionConfiguration>();
        m_bulletCollisionDispatcher     = std::make_unique<btCollisionDispatcher>( m_bulletCollisionConfiguration.get() );
        m_bulletConstraintSolver        = std::make_unique<btMultiBodyConstraintSolver>();
        m_bulletDynamicsWorld           = std::make_unique<btMultiBodyDynamicsWorld>(
                                                                m_bulletCollisionDispatcher.get(),
                                                                m_bulletBroadphase.get(),
                                                                m_bulletConstraintSolver.get(),
                                                                m_bulletCollisionConfiguration.get() );
        m_bulletDynamicsWorld->setGravity( btVector3( 0, 0, -9.81 ) );
        m_bulletDebugDrawer = nullptr;

        _CreateSingleBodyAdapters();
        //// _CreateCompoundAdapters();
        //// _CreateKintreeAdapters();
        //// _CreateTerrainGeneratorAdapters();

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TBulletSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TBulletSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TBulletSimulation::_CreateSingleBodyAdapters()
    {
        auto single_bodies = m_scenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto single_body_adapter = std::make_unique<TBulletSingleBodyAdapter>( single_body );
            single_body->SetBodyAdapter( single_body_adapter.get() );
            m_singleBodyAdapters.push_back( std::move( single_body_adapter ) );
        }
    }

    TBulletSimulation::~TBulletSimulation()
    {
        m_bulletDynamicsWorld = nullptr;
        m_bulletConstraintSolver = nullptr;
        m_bulletBroadphase = nullptr;
        m_bulletCollisionDispatcher = nullptr;
        m_bulletCollisionConfiguration = nullptr;
        m_bulletDebugDrawer = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TBulletSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TBulletSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    bool TBulletSimulation::_InitializeInternal()
    {
        for ( auto& single_body_adapter : m_singleBodyAdapters )
        {
            if ( auto bullet_adapter = dynamic_cast<TBulletSingleBodyAdapter*>( single_body_adapter.get() ) )
                bullet_adapter->SetBulletWorld( m_bulletDynamicsWorld.get() );
        }

        LOCO_CORE_TRACE( "Bullet-backend >>> broadphase         : {0}", typeid( *m_bulletBroadphase ).name() );
        LOCO_CORE_TRACE( "Bullet-backend >>> collision config.  : {0}", typeid( *m_bulletCollisionConfiguration ).name() );
        LOCO_CORE_TRACE( "Bullet-backend >>> constraint solver  : {0}", typeid( *m_bulletConstraintSolver ).name() );
        LOCO_CORE_TRACE( "Bullet-backend >>> bullett-world      : {0}", typeid( *m_bulletDynamicsWorld ).name() );

        return true;
    }

    void TBulletSimulation::_PreStepInternal()
    {
        // Do nothing here, as call to wrappers is enough (made in base)
    }

    void TBulletSimulation::_SimStepInternal()
    {
        if ( !m_bulletDynamicsWorld )
        {
            LOCO_CORE_WARN( "TBulletSimulation::_SimStepInternal >>> btDynamicsWorld object is required \
                             for taking a simulation step, but got nullptr instead" );
            return;
        }

        m_bulletDynamicsWorld->stepSimulation( 1.0 / 60.0 );
    }

    void TBulletSimulation::_PostStepInternal()
    {
        // @todo: run loco-contact-manager here to grab all detected contacts

        m_bulletDynamicsWorld->debugDrawWorld();
    }

    void TBulletSimulation::_ResetInternal()
    {
        // @todo: reset loco-contact-manager
    }

    void TBulletSimulation::_SetVisualizerInternal( TIVisualizer* visualizerRef )
    {
        m_bulletDebugDrawer = std::make_unique<TBulletDebugDrawer>( visualizerRef );
        m_bulletDynamicsWorld->setDebugDrawer( m_bulletDebugDrawer.get() );
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef )
    {
        return new loco::bullet::TBulletSimulation( scenarioRef );
    }

}}