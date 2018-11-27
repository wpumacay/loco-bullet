
#include <tysocBullet.h>



namespace tysocBullet
{


    TTysocBulletApi::TTysocBulletApi()
    {
        m_btBroadphaseInterfacePtr      = NULL;
        m_btCollisionConfigurationPtr   = NULL;
        m_btCollisionDispatcherPtr      = NULL;
        m_btConstraintSolverPtr         = NULL;
        m_btWorldPtr                    = NULL;
    }


    TTysocBulletApi::~TTysocBulletApi()
    {
        // @TODO: Check if base gets called

        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            delete m_terrainGenWrappers[i];
        }
        m_terrainGenWrappers.clear();

        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
            delete it->second;
        }
        m_agentWrappers.clear();
    }

    void TTysocBulletApi::addAgentWrapper( TBulletAgentWrapper* agentWrapperPtr )
    {
        m_agentWrappers[ agentWrapperPtr->name() ] = agentWrapperPtr;
    }

    void TTysocBulletApi::addTerrainGenWrapper( TBulletTerrainGenWrapper* terrainGenWrapperPtr )
    {
        m_terrainGenWrappers.push_back( terrainGenWrapperPtr );
    }

    bool TTysocBulletApi::initializeBulletApi()
    {
        // Initialize bullet ***********************************************

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
        

        // *****************************************************************

        // Initialize all other functionality ******************************

        if ( !m_scenarioPtr )
        {
            // create a default scenario if none given
            m_scenarioPtr = new tysoc::TScenario();
        }

        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            m_terrainGenWrappers[i]->setBulletWorld( m_btWorldPtr );
            m_scenarioPtr->addTerrainGenerator( m_terrainGenWrappers[i]->terrainGenerator() );
        }

        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
        
            m_scenarioPtr->addAgent( it->second->agent() );
        }

        // initialize all underlying base resources
        initialize();

        // Initialize wrappers
        for( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            m_terrainGenWrappers[i]->initialize();
        }

        return true;
    }

    void TTysocBulletApi::setAgentPosition( const std::string& name,
                                            float x, float y, float z )
    {
        // if ( m_agentWrappers.find( name ) != m_agentWrappers.end() )
        // {
        //     m_agentWrappers[ name ]->setPosition( x, y, z );
        // }
    }
    void TTysocBulletApi::getAgentPosition( const std::string& name,
                                            float &x, float &y, float &z )
    {
        // if ( m_agentWrappers.find( name ) != m_agentWrappers.end() )
        // {
        //     m_agentWrappers[ name ]->getPosition( x, y, z );
        // }
    }

    void TTysocBulletApi::_preStep()
    {
        // collect terrain generaion info by letting ...
        // the terrain wrappers do the job
        for ( size_t i = 0; i < m_terrainGenWrappers.size(); i++ )
        {
            m_terrainGenWrappers[ i ]->preStep();
        }

        // Collect actuator controls by letting ...
        // the agent wrappers do the job
        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
            it->second->preStep();
        }
    }

    void TTysocBulletApi::_updateStep()
    {
        m_btWorldPtr->stepSimulation( 1.0f / 60.0f );
    }

    void TTysocBulletApi::_postStep()
    {
        // collect bodies and joints information
        for ( auto it = m_agentWrappers.begin();
              it != m_agentWrappers.end();
              it++ )
        {
            it->second->postStep();
        }
    }
}