
#include <loco_simulation_bullet.h>

namespace loco {
namespace bullet {

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

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TBulletSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TBulletSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TBulletSimulation::~TBulletSimulation()
    {
        m_bulletConstraintSolver = nullptr;
        m_bulletBroadphase = nullptr;
        m_bulletCollisionDispatcher = nullptr;
        m_bulletCollisionConfiguration = nullptr;
        m_bulletDynamicsWorld = nullptr;
    }

    bool TBulletSimulation::_InitializeInternal()
    {
        // Collect xml-resources from the adapters, assemble them into a single xml-object, and save to disk
        // @todo: implement-me ...


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
        // @todo: run loco-contact-manager here to grabs all detected contacts
    }

    void TBulletSimulation::_ResetInternal()
    {
        // @todo: reset loco-contact-manager
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef )
    {
        return new loco::bullet::TBulletSimulation( scenarioRef );
    }

}}