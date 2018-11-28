
#pragma once

#include <tysocBulletAgent.h>
#include <tysocBulletTerrain.h>
#include <tysocBulletFactory.h>
#include <tysocBulletPrimitivesSpawner.h>

// abstract api to extend from
#include <api_adapter.h>


namespace tysocBullet
{


    class TTysocBulletApi : public tysoc::TTysocCommonApi
    {

        private :

        std::vector< TBulletTerrainGenWrapper* >       m_terrainGenWrappers;
        std::map< std::string, TBulletAgentWrapper* >  m_agentWrappers;

        btDiscreteDynamicsWorld*                m_btWorldPtr;
        btSequentialImpulseConstraintSolver*    m_btConstraintSolverPtr;
        btCollisionDispatcher*                  m_btCollisionDispatcherPtr;
        btDefaultCollisionConfiguration*        m_btCollisionConfigurationPtr;
        btBroadphaseInterface*                  m_btBroadphaseInterfacePtr;

        protected :

        void _preStep() override;
        void _updateStep() override;
        void _postStep() override;

        public :

        TTysocBulletApi();
        ~TTysocBulletApi();

        void addAgentWrapper( TBulletAgentWrapper* agentWrapperPtr );
        void addTerrainGenWrapper( TBulletTerrainGenWrapper* terrainGenWrapperPtr );

        bool initializeBulletApi();

        void setAgentPosition( const std::string& name,
                               float x, float y, float z );
        void getAgentPosition( const std::string& name,
                               float &x, float &y, float &z );
    };



}