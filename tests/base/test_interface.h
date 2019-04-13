
#pragma once

// Bullet API functionality
// Base bullet functionality
#include <btBulletDynamicsCommon.h>
// Multibody bullet functionality
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodySliderConstraint.h>
// Rendering functionality from 'cat1' engine
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>
// Some functionality from std
#include <vector>

using namespace engine;

namespace bullet
{

    class SimObj
    {
        private :

        btCollisionObject* m_colObj;
        LMesh* m_graphicsObj;

        public :

        SimObj( btCollisionObject* colObj );
        ~SimObj();

        btCollisionObject* colObj();
        LMesh* graphicsObj();

        void update();
    };

    class CustomDebugDrawer : public btIDebugDraw
    {
        private :

        // bit fields used for the mode
        int m_debugMode;

        public : 

        CustomDebugDrawer();
        virtual ~CustomDebugDrawer();

        void drawLine( const btVector3& from, const btVector3& to, const btVector3& color ) override;
        void drawContactPoint( const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color ) override;
        void reportErrorWarning( const char* warningString ) override;
        void draw3dText( const btVector3& location, const char* textString ) override;
        void setDebugMode( int debugMode ) override;
        int getDebugMode() const override;
    };

    class ITestApplication
    {

        protected :

        btConstraintSolver*                     m_btConstraintSolverPtr;
        btCollisionDispatcher*                  m_btCollisionDispatcherPtr;
        btDefaultCollisionConfiguration*        m_btCollisionConfigurationPtr;
        btBroadphaseInterface*                  m_btBroadphaseInterfacePtr;

        btDynamicsWorld* m_btWorldPtr;
        btIDebugDraw* m_btDebugDrawer;

        LApp* m_graphicsApp;
        LScene* m_graphicsScene;

        std::vector< SimObj* > m_simObjs;

        void _init();
        void _initGraphics();
        // Derived base-application types should override this and create ...
        // resources for specific cases (as in multibody case)
        virtual void _initPhysicsInternal() = 0;
        // User should override this and create what he wants
        virtual void _initScenario() = 0;
        // Same considerations as for _initPhysics
        virtual void _startInternal() = 0;
        // Same considerations as for _startInternal
        virtual void _stepInternal() = 0;

        public :

        ITestApplication();
        ~ITestApplication();

        void start();
        void step();

        /* Helper functions */

        btCollisionShape* createCollisionShape( const std::string& shape,
                                                const btVector3& size );

        SimObj* createBody( const std::string& shape,
                            const btVector3& size,
                            const btVector3& pos,
                            const btVector3& rot,
                            const btVector3& color,
                            float mass );
    };


    class SimpleTestApplication : public ITestApplication
    {
        protected :

        void _initPhysicsInternal() override;
        void _startInternal() override;
        void _stepInternal() override;

        public :

        SimpleTestApplication();
        ~SimpleTestApplication();
    };

    class SimMultibodyLink : SimObj
    {
        private :

        SimMultibodyLink* m_parentObj;

        public :

        SimMultibodyLink( btCollisionObject* colObj,
                          SimMultibodyLink* parentObj );
        ~SimMultibodyLink();

        SimMultibodyLink* parentObj();
    };

    class SimMultibody
    {
        private :

        btMultiBody* m_btMultibody;
        std::vector< SimMultibodyLink* > m_simLinks;

        public :

        SimMultibody();
        ~SimMultibody();

        SimMultibodyLink* addLink( const std::string& shapeType,
                                   const btVector3& shapeSize,
                                   const btTransform& localTransform,
                                   SimMultibodyLink* parentObj,
                                   const std::string& jointType,
                                   const btVector3& jointAxis,
                                   const btTransform& jointLocalTransform );

        void update();
    };


    class MultibodyTestApplication : public ITestApplication
    {
        private :

        std::vector< SimMultibody* > m_simMultibodies;

        protected :

        void _initPhysicsInternal() override;
        void _startInternal() override;
        void _stepInternal() override;

        public :

        MultibodyTestApplication();
        ~MultibodyTestApplication();

    };

}