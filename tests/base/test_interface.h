
#pragma once

// Bullet API functionality
// Base bullet functionality
#include <btBulletDynamicsCommon.h>
// Multibody bullet functionality
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodySliderConstraint.h>
// Rendering functionality from 'cat1' engine
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>
// UI functionality (from Dear ImGui)
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// Some functionality from std
#include <vector>

using namespace engine;

#define DEFAULT_DENSITY 1000.0f // density of water, same default as in mujoco

namespace bullet
{

    /* Some helpers */

    btCollisionShape* createCollisionShape( const std::string& shape,
                                            const btVector3& size );

    btScalar computeMassFromShape( btCollisionShape* colShape );

    /* SimObj: simple wrapper for collision objects being simulated by the engine */

    class SimObj
    {
        protected :

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

        bool m_isRunning;

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
        // UI functionality
        virtual void _renderUI();

        public :

        ITestApplication();
        ~ITestApplication();

        void start();
        void step();
        void togglePause();

        /* Helper functions */

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

    class SimMultibodyLink : public SimObj
    {
        private :

        int m_linkIndx;
        SimMultibodyLink* m_parentObj;

        engine::LModel* m_axesObj;

        public :

        SimMultibodyLink( btCollisionObject* colObj,
                          int linkIndx,
                          SimMultibodyLink* parentObj );
        ~SimMultibodyLink();

        SimMultibodyLink* parentObj();
        engine::LModel* axesObj();
        int getIndx();
    };

    class SimMultibody
    {
        private :

        std::string m_name;

        btMultiBody* m_btMultibody;
        std::vector< SimMultibodyLink* > m_simLinks;

        std::vector< btMultiBodyConstraint* >           m_constraints;
        std::vector< btMultiBodyJointMotor* >           m_jointMotors;
        std::vector< btMultiBodySphericalJointMotor* >  m_jointSphericalMotors;

        public :

        SimMultibody( const std::string& name,
                      size_t numLinks, 
                      const btVector3& position,
                      const std::string& baseShape,
                      const btVector3& baseSize,
                      float baseMass, 
                      bool baseIsFixed );
        ~SimMultibody();

        SimMultibodyLink* setupLinkSingleJoint( int linkIndx,
                                                const std::string& shapeType,
                                                const btVector3& shapeSize,
                                                const btTransform& localTransform,
                                                SimMultibodyLink* parentObj,
                                                const std::string& jointType,
                                                const btVector3& jointAxis,
                                                const btVector3& jointPivot,
                                                bool useMotor = true,
                                                float lowerLimit = -1.0f,
                                                float upperLimit = 1.0f );

        std::vector< SimMultibodyLink* > setupLinkMultiDof( int linkIndx,
                                                            const std::string& shapeType,
                                                            const btVector3& shapeSize,
                                                            const btTransform& localTransform,
                                                            SimMultibodyLink* parentObj,
                                                            const std::vector< std::string >& jointsTypes,
                                                            const std::vector< btVector3 >& jointsAxis,
                                                            const std::vector< btVector3 >& jointsPivots,
                                                            const std::vector< bool >& jointsUseMotor,
                                                            const std::vector< float >& jointsLowerLimits = { -1.0f },
                                                            const std::vector< float >& jointsUpperLimits = { 1.0f } );

        void update();

        std::string name();

        std::vector< SimMultibodyLink* > linksPtrs();
        std::vector< btMultiBodyConstraint* > constraintsPtrs();
        std::vector< btMultiBodyJointMotor* > motorsPtrs();
        std::vector< btMultiBodySphericalJointMotor* > sphericalMotorsPtrs();

        SimMultibodyLink* ptrRootLink();
        btMultiBody* ptrBtMultibody();

        bool hasJointMotors();
        bool hasSphericalJointMotors();
    };


    class MultibodyTestApplication : public ITestApplication
    {
        private :

        std::vector< SimMultibody* > m_simMultibodies;

        SimMultibody* m_currentSimMultibody;
        std::string m_currentSimMultibodyName;

        // some ui options
        bool m_showWireframe;
        bool m_useBtDebugDrawer;

        protected :

        void _initPhysicsInternal() override;
        void _startInternal() override;
        void _stepInternal() override;
        void _renderUI() override;

        public :

        MultibodyTestApplication();
        ~MultibodyTestApplication();

        void addSimMultibody( SimMultibody* simMultibodyPtr );

    };

}