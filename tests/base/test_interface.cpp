
#include "test_interface.h"

using namespace engine;

namespace bullet
{

    btCollisionShape* createCollisionShape( const std::string& shape,
                                            const btVector3& size )
    {
        btCollisionShape* _colShape = NULL;

        if ( shape == "box" )
            _colShape = new btBoxShape( btVector3( 0.5 * size.x(), 0.5 * size.y(), 0.5 * size.z() ) );
        else if ( shape == "sphere" )
            _colShape = new btSphereShape( size.x() );
        else if ( shape == "capsule" )
            _colShape = new btCapsuleShapeZ( size.x(), size.y() );
        else if ( shape == "cylinder" )
            _colShape = new btCylinderShapeZ( btVector3( size.x(), size.x(), size.y() ) );
        else
            std::cout << "ERROR> shape: " << shape << " not supported" << std::endl;

        if ( !_colShape )
            return NULL;

        _colShape->setMargin( 0.0f );

        return _colShape;
    }

    btScalar computeMassFromShape( btCollisionShape* colShape )
    {
        if ( !colShape )
        {
            std::cout << "ERROR> no collision shape provided for mass calculation" << std::endl;
            return btScalar( 0.0f );
        }

        int _type = colShape->getShapeType();

        btScalar _volume;
        btScalar _pi = 3.141592653589793;

        if ( _type == BOX_SHAPE_PROXYTYPE )
        {
            auto _boxShape = reinterpret_cast< btBoxShape* >( colShape );
            auto _boxDimensions = _boxShape->getHalfExtentsWithoutMargin();

            _volume = ( _boxDimensions.x() * _boxDimensions.y() * _boxDimensions.z() );
        }
        else if ( _type == SPHERE_SHAPE_PROXYTYPE )
        {
            auto _sphereShape = reinterpret_cast< btSphereShape* >( colShape );
            auto _sphereRadius = _sphereShape->getRadius();

            _volume = _pi * ( _sphereRadius * _sphereRadius * _sphereRadius );
        }
        else if ( _type == CYLINDER_SHAPE_PROXYTYPE )
        {
            auto _cylinderShape = reinterpret_cast< btCylinderShape* >( colShape );
            auto _cylinderUpAxis = _cylinderShape->getUpAxis();
            auto _cylinderRadius = _cylinderShape->getRadius();
            auto _cylinderHalfExtents = _cylinderShape->getHalfExtentsWithoutMargin();
            auto _cylinderHeight = 2.0 * _cylinderHalfExtents[_cylinderUpAxis];

            _volume = ( _pi * _cylinderRadius * _cylinderRadius ) * _cylinderHeight;
        }
        else if ( _type == CAPSULE_SHAPE_PROXYTYPE )
        {
            auto _capsuleShape = reinterpret_cast< btCapsuleShape* >( colShape );
            auto _capsuleUpAxis = _capsuleShape->getUpAxis();
            auto _capsuleRadius = _capsuleShape->getRadius();
            auto _capsuleHeight = 2.0 * _capsuleShape->getHalfHeight();

            _volume = ( _pi * _capsuleRadius * _capsuleRadius ) * _capsuleHeight +
                      ( _pi * _capsuleRadius * _capsuleRadius * _capsuleRadius );
        }
        else
        {
            // compute from aabb
            btVector3 _aabbMin, _aabbMax;

            btTransform _identityTransform;
            _identityTransform.setIdentity();

            colShape->getAabb( _identityTransform, _aabbMin, _aabbMax );

            _volume = btFabs( ( _aabbMax.x() - _aabbMin.x() ) * 
                              ( _aabbMax.y() - _aabbMin.y() ) *
                              ( _aabbMax.z() - _aabbMin.z() ) );
        }

        return _volume * DEFAULT_DENSITY;
    }

    /* SimObj ******************************************************************/

    SimObj::SimObj( btCollisionObject* colObj )
    {
        m_colObj = colObj;

        if ( !m_colObj )
            return;

        auto _colShape = m_colObj->getCollisionShape();
        auto _shapeType = _colShape->getShapeType();

        if ( _shapeType == BOX_SHAPE_PROXYTYPE )
        {
            auto _boxShape = reinterpret_cast< btBoxShape* >( _colShape );
            auto _boxDimensions = _boxShape->getHalfExtentsWithoutMargin();

            m_graphicsObj = LMeshBuilder::createBox( 2.0 * _boxDimensions.x(),
                                                     2.0 * _boxDimensions.y(),
                                                     2.0 * _boxDimensions.z() );
        }
        else if ( _shapeType == SPHERE_SHAPE_PROXYTYPE )
        {
            auto _sphereShape = reinterpret_cast< btSphereShape* >( _colShape );
            auto _sphereRadius = _sphereShape->getRadius();

            m_graphicsObj = LMeshBuilder::createSphere( _sphereRadius );
        }
        else if ( _shapeType == CAPSULE_SHAPE_PROXYTYPE )
        {
            auto _capsuleShape = reinterpret_cast< btCapsuleShape* >( _colShape );
            auto _capsuleUpAxis = _capsuleShape->getUpAxis();
            auto _capsuleRadius = _capsuleShape->getRadius();
            auto _capsuleHeight = 2.0 * _capsuleShape->getHalfHeight();

            m_graphicsObj = LMeshBuilder::createCapsule( _capsuleRadius,
                                                         _capsuleHeight,
                                                         _capsuleUpAxis );
        }
        else if ( _shapeType == CYLINDER_SHAPE_PROXYTYPE )
        {
            auto _cylinderShape = reinterpret_cast< btCylinderShape* >( _colShape );
            auto _cylinderUpAxis = _cylinderShape->getUpAxis();
            auto _cylinderRadius = _cylinderShape->getRadius();
            auto _cylinderHalfExtents = _cylinderShape->getHalfExtentsWithoutMargin();
            auto _cylinderHeight = 2.0 * _cylinderHalfExtents[_cylinderUpAxis];

            m_graphicsObj = LMeshBuilder::createCylinder( _cylinderRadius,
                                                          _cylinderHeight,
                                                          _cylinderUpAxis );
        }

        // set the initial positions of the graphics object
        update();
    }

    SimObj::~SimObj()
    {
        // nothing for now
    }

    btCollisionObject* SimObj::colObj()
    {
        return m_colObj;
    }

    LMesh* SimObj::graphicsObj()
    {
        return m_graphicsObj;
    }

    void SimObj::update()
    {
        if ( !m_colObj )
            return;

        if ( !m_graphicsObj )
            return;

        auto _btWorldTransform = m_colObj->getWorldTransform();
        auto _btWorldPosition = _btWorldTransform.getOrigin();
        auto _btWorldRotmat = _btWorldTransform.getBasis();

        m_graphicsObj->pos = { _btWorldPosition.x(),
                               _btWorldPosition.y(),
                               _btWorldPosition.z() };

        for ( size_t i = 0; i < 3; i++ )
            for( size_t j = 0; j < 3; j++ )
                m_graphicsObj->rotation.buff[i + 4 * j] = _btWorldRotmat[i][j];
    }

    /* CustomDebugDrawer *******************************************************/


    CustomDebugDrawer::CustomDebugDrawer()
    {
        m_debugMode = btIDebugDraw::DBG_DrawWireframe | 
                      btIDebugDraw::DBG_DrawAabb |
                      btIDebugDraw::DBG_DrawFrames |
                      btIDebugDraw::DBG_DrawConstraints;
    }

    CustomDebugDrawer::~CustomDebugDrawer()
    {
        // nothing to do here
    }

    void CustomDebugDrawer::drawLine( const btVector3& from, const btVector3& to, const btVector3& color )
    {
        DebugSystem::drawLine( { from.x(), from.y(), from.z() },
                               { to.x(), to.y(), to.z() },
                               { color.x(), color.y(), color.z() } );
    }

    void CustomDebugDrawer::drawContactPoint( const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color )
    {
        btVector3 _p1 = PointOnB;
        btVector3 _p2 = PointOnB + normalOnB * distance;
        btVector3 _p3 = PointOnB + normalOnB * 0.01;

        DebugSystem::drawLine( { _p1.x(), _p1.y(), _p1.z() },
                               { _p2.x(), _p2.y(), _p2.z() },
                               { color.x(), color.y(), color.z() } );

        DebugSystem::drawLine( { _p1.x(), _p1.y(), _p1.z() },
                               { _p3.x(), _p3.y(), _p3.z() },
                               { 0.2, 0.4, 0.5 } );
    }

    void CustomDebugDrawer::reportErrorWarning( const char* warningString )
    {
        std::cout << "WARNING> BtDebugDrawer says: " << warningString << std::endl;
    }

    void CustomDebugDrawer::draw3dText( const btVector3& location, const char* textString )
    {
        // do nothing
    }

    void CustomDebugDrawer::setDebugMode( int debugMode )
    {
        m_debugMode = debugMode;
    }

    int CustomDebugDrawer::getDebugMode() const
    {
        return m_debugMode;
    }

    /* ITestApplication ********************************************************/

    ITestApplication::ITestApplication()
    {
        m_btConstraintSolverPtr = NULL;
        m_btCollisionDispatcherPtr = NULL;
        m_btCollisionConfigurationPtr = NULL;
        m_btBroadphaseInterfacePtr = NULL;

        m_btWorldPtr = NULL;
        m_btDebugDrawer = NULL;

        m_graphicsApp = NULL;
        m_graphicsScene = NULL;
    }

    ITestApplication::~ITestApplication()
    {
        m_btWorldPtr = NULL;

        if ( m_btDebugDrawer )
        {
            delete m_btDebugDrawer;
            m_btDebugDrawer = NULL;
        }

        m_graphicsScene = NULL;

        if ( m_graphicsApp )
        {
            delete m_graphicsApp;
            m_graphicsApp = NULL;
        }
    }

    void ITestApplication::_init()
    {
        _initGraphics();
        _initPhysicsInternal();
        _initScenario();

        if ( m_btWorldPtr )
            m_btWorldPtr->setDebugDrawer( m_btDebugDrawer );
    }

    void ITestApplication::_initGraphics()
    {
        m_graphicsApp = LApp::GetInstance();
        m_graphicsScene = LApp::GetInstance()->scene();

        auto _camera = new LFpsCamera( "fps",
                                       LVec3( 1.0f, 2.0f, 1.0f ),
                                       LVec3( -2.0f, -4.0f, -2.0f ),
                                       LICamera::UP_Z );

        auto _light = new LLightDirectional( LVec3( 0.8, 0.8, 0.8 ), 
                                                     LVec3( 0.8, 0.8, 0.8 ),
                                                     LVec3( 0.3, 0.3, 0.3 ), 
                                                     0, 
                                                     LVec3( 0, 0, -1 ) );
        _light->setVirtualPosition( LVec3( 5, 0, 5 ) );

        m_graphicsScene->addCamera( _camera );
        m_graphicsScene->addLight( _light );

        // create debug drawer helper
        m_btDebugDrawer = new CustomDebugDrawer();
    }

    void ITestApplication::start()
    {
        _init();
        // Create wrappers for SimObj or other necessary resources
        _startInternal();
    }

    void ITestApplication::step()
    {
        if ( m_btWorldPtr )
            m_btWorldPtr->stepSimulation( 1. / 60. );

        for ( size_t i = 0; i < m_simObjs.size(); i++ )
            m_simObjs[i]->update();

        _stepInternal();

        if ( engine::InputSystem::isKeyDown( GLFW_KEY_SPACE ) )
        {
            // @CHECK: Should apply globally, as some cameras will not listen
            m_graphicsScene->getCurrentCamera()->setActiveMode( false );
            m_graphicsApp->window()->enableCursor();
        }
        else if ( engine::InputSystem::isKeyDown( GLFW_KEY_ENTER ) )
        {
            m_graphicsScene->getCurrentCamera()->setActiveMode( true );
            m_graphicsApp->window()->disableCursor();
        }

        m_graphicsApp->begin();
        m_graphicsApp->update();

        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 5.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } );
        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 0.0f, 5.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } );
        engine::DebugSystem::drawLine( { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 5.0f }, { 0.0f, 0.0f, 1.0f } );
        
        if ( m_btWorldPtr )
            m_btWorldPtr->debugDrawWorld();

        m_graphicsApp->end();

    }

    SimObj* ITestApplication::createBody( const std::string& shape,
                                          const btVector3& size,
                                          const btVector3& pos,
                                          const btVector3& rot,
                                          const btVector3& color,
                                          float mass )
    {
        SimObj* _simobj = NULL;

        // collision shape
        btCollisionShape* _colShape = createCollisionShape( shape, size );

        // motion state
        btTransform _rbTransform;
        _rbTransform.setIdentity();
        _rbTransform.setOrigin( pos );
        _rbTransform.getBasis().setEulerZYX( rot.x(), rot.y(), rot.z() );
        auto _rbMotionState = new btDefaultMotionState( _rbTransform );

        // inertial properties
        btScalar _inertiaMass = mass;
        btVector3 _inertiaDiag;
        if ( _inertiaMass != 0.0f )
            _colShape->calculateLocalInertia( _inertiaMass, _inertiaDiag );

        // construction info
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo(
                                                            _inertiaMass,
                                                            _rbMotionState,
                                                            _colShape,
                                                            _inertiaDiag );

        // create the rigid body
        auto _rigidBody = new btRigidBody( _rbConstructionInfo );
        _rigidBody->forceActivationState( DISABLE_DEACTIVATION );

        if ( m_btWorldPtr )
            m_btWorldPtr->addRigidBody( _rigidBody );

        // add a wrapper for this body
        _simobj = new SimObj( _rigidBody );
        m_simObjs.push_back( _simobj );

        // configure graphics object
        {
            auto _mesh = _simobj->graphicsObj();
            auto _material = _mesh->getMaterial();
            _material->setColor( { color.x(), color.y(), color.z() } );

            m_graphicsScene->addRenderable( _mesh );
        }

        return _simobj;
    }

    /* SimpleTestApplication ***************************************************/

    SimpleTestApplication::SimpleTestApplication()
        : ITestApplication()
    {
        // nothing extra to do for now
    }

        SimpleTestApplication::~SimpleTestApplication()
    {
        // nothing extra to do for now
    }

    void SimpleTestApplication::_initPhysicsInternal()
    {
        m_btBroadphaseInterfacePtr      = new btDbvtBroadphase();
        m_btCollisionConfigurationPtr   = new btDefaultCollisionConfiguration();
        m_btCollisionDispatcherPtr      = new btCollisionDispatcher( m_btCollisionConfigurationPtr );
        m_btConstraintSolverPtr         = new btSequentialImpulseConstraintSolver();

        m_btWorldPtr = new btDiscreteDynamicsWorld( m_btCollisionDispatcherPtr,
                                                    m_btBroadphaseInterfacePtr,
                                                    m_btConstraintSolverPtr,
                                                    m_btCollisionConfigurationPtr );

        m_btWorldPtr->setGravity( btVector3( 0, 0, -10 ) );
    }

    void SimpleTestApplication::_startInternal()
    {
        // nothing to do here
    }

    void SimpleTestApplication::_stepInternal()
    {
        // nothing to do here
    }

    /* SimMultibodyLink ***************************************************/
    
    SimMultibodyLink::SimMultibodyLink( btCollisionObject* colObj,
                                        int linkIndx,
                                        SimMultibodyLink* parentObj )
        : SimObj( colObj )
    {
        m_linkIndx = linkIndx;
        m_parentObj = parentObj;
    }

    SimMultibodyLink::~SimMultibodyLink()
    {
        m_parentObj = NULL;
    }

    SimMultibodyLink* SimMultibodyLink::parentObj()
    {
        return m_parentObj;
    }

    int SimMultibodyLink::getIndx()
    {
        return m_linkIndx;
    }

    /* SimMultibody ********************************************************/

    SimMultibody::SimMultibody( size_t numLinks, 
                                const btVector3& position,
                                const std::string& baseShape,
                                const btVector3& baseSize,
                                float baseMass, 
                                bool baseIsFixed )
    {
        // create the shape of the base
        auto _bShape = createCollisionShape( baseShape, baseSize );

        // compute inertial properties required for base when creating a multibody
        btScalar _bMass( baseMass );
        btVector3 _bInertia;

        if ( _bMass != 0.0f )
            _bShape->calculateLocalInertia( _bMass, _bInertia );

        // create the multibody
        m_btMultibody = new btMultiBody( numLinks, _bMass, _bInertia, baseIsFixed, false );
        
        // create a SimMultibodyLink for the base
        auto _bCollider = new btMultiBodyLinkCollider( m_btMultibody, -1 );
        _bCollider->setCollisionShape( _bShape );

        auto _bSimMultibodyLink = new SimMultibodyLink( _bCollider, -1, NULL );

        m_btMultibody->setBaseCollider( _bCollider );
        m_btMultibody->setBasePos( position );

        m_simLinks.push_back( _bSimMultibodyLink );
    }

    SimMultibody::~SimMultibody()
    {
        // nothing for now
    }

    SimMultibodyLink* SimMultibody::setupLinkSingleJoint( int linkIndx,
                                                          const std::string& shapeType,
                                                          const btVector3& shapeSize,
                                                          const btTransform& localTransform,
                                                          SimMultibodyLink* parentObj,
                                                          const std::string& jointType,
                                                          const btVector3& jointAxis,
                                                          const btVector3& jointPivot )
    {
        if ( !m_btMultibody )
        {
            std::cout << "ERROR> no btMultibody created for this object" << std::endl;
            return NULL;
        }

        if ( !parentObj )
        {
            std::cout << "ERROR> this link (non-root) has no parent" << std::endl;
            return NULL;
        }

        // grab parent index for latex usage 
        int _linkParentIndx = parentObj->getIndx();

        // first just create the shape required for the link
        auto _linkShape = createCollisionShape( shapeType, shapeSize );

        // compute inertial properties from this shape
        btScalar _linkMass = computeMassFromShape( _linkShape );
        btVector3 _linkInertia;
        if ( _linkMass != 0.0f )
        {
            _linkShape->calculateLocalInertia( _linkMass, _linkInertia );
        }

        // grab localPos and localRot from localTransform. Recall this=local, ...
        // which is respect to the parent link. Also, get the inverse of these ...
        // quantities, as the setupXYZ methods required them
        auto _this2parent_quat  = localTransform.getRotation();
        auto _this2parent_pos   = localTransform.getOrigin();
        auto _parent2this_quat  = _this2parent_quat.inverse();
        auto _parent2this_pos   = -quatRotate( _parent2this_quat, _this2parent_pos );

        // compute pivot w.r.t. parent COM
        auto _pivot2parent_pos = _this2parent_pos - quatRotate( _this2parent_quat, -jointPivot );

        // setup the constraint for this link
        if ( jointType == "hinge" || jointType == "continuous" || jointType == "revolute" )
        {
            m_btMultibody->setupRevolute( linkIndx, 
                                          _linkMass, 
                                          _linkInertia,
                                          _linkParentIndx,
                                          _parent2this_quat,
                                          jointAxis,
                                          _pivot2parent_pos,
                                          -jointPivot,
                                          true );
        }
//         else if ( jointType == "ball" || jointType == "spheric" || jointType == "spherical" )
//         {
// 
//         }
        else
        {
            std::cout << "ERROR> joint type: " << jointType << " not supported" << std::endl;
        }

        // collider managment
        auto _linkCollider = new btMultiBodyLinkCollider( m_btMultibody, linkIndx );
        _linkCollider->setCollisionShape( _linkShape );
        m_btMultibody->getLink( linkIndx ).m_collider = _linkCollider;

        // create SimMultibodyLink wrapper
        auto _linkSimMultibodyLink = new SimMultibodyLink( _linkCollider, linkIndx, parentObj );


        // cache the simlink for later usage
        m_simLinks.push_back( _linkSimMultibodyLink );

        return _linkSimMultibodyLink;
    }

    void SimMultibody::update()
    {
        for ( size_t i = 0; i < m_simLinks.size(); i++ )
            m_simLinks[i]->update();
    }

    std::vector< SimMultibodyLink* > SimMultibody::linksPtrs()
    {
        return m_simLinks;
    }

    SimMultibodyLink* SimMultibody::ptrRootLink()
    {
        return m_simLinks[0];
    }

    btMultiBody* SimMultibody::ptrBtMultibody()
    {
        return m_btMultibody;
    }

    /* MultibodyTestApplication ***************************************************/

    MultibodyTestApplication::MultibodyTestApplication()
        : ITestApplication()
    {
        // nothing to do here for now
    }

    MultibodyTestApplication::~MultibodyTestApplication()
    {
        // nothing to do here for now
    }

    void MultibodyTestApplication::_initPhysicsInternal()
    {
        m_btBroadphaseInterfacePtr      = new btDbvtBroadphase();
        m_btCollisionConfigurationPtr   = new btDefaultCollisionConfiguration();
        m_btCollisionDispatcherPtr      = new btCollisionDispatcher( m_btCollisionConfigurationPtr );
        m_btConstraintSolverPtr         = new btMultiBodyConstraintSolver();

        m_btWorldPtr = new btMultiBodyDynamicsWorld( m_btCollisionDispatcherPtr,
                                                     m_btBroadphaseInterfacePtr,
                                                     (btMultiBodyConstraintSolver*) m_btConstraintSolverPtr,
                                                     m_btCollisionConfigurationPtr );

        m_btWorldPtr->setGravity( btVector3( 0, 0, -10 ) );
    }

    void MultibodyTestApplication::_startInternal()
    {
        // nothing to do here
    }

    void MultibodyTestApplication::_stepInternal()
    {
        for ( size_t i = 0; i < m_simMultibodies.size(); i++ )
            m_simMultibodies[i]->update();
    }

    void MultibodyTestApplication::addSimMultibody( SimMultibody* simMultibodyPtr )
    {
        // add internal btMultibody resource
        reinterpret_cast< btMultiBodyDynamicsWorld* >
                            ( m_btWorldPtr )->addMultiBody( simMultibodyPtr->ptrBtMultibody() );

        // add all graphics resources to the scene
        auto _linksPtrs = simMultibodyPtr->linksPtrs();
        for ( size_t q = 0; q < _linksPtrs.size(); q++ )
        {
            auto _mesh = _linksPtrs[q]->graphicsObj();
            auto _material = _mesh->getMaterial();
            _material->setColor( { 0.7, 0.5, 0.3 } );

            m_graphicsScene->addRenderable( _mesh );
        }

        // add all multibody-links resources (even base) to the world
        for ( size_t q = 0; q < _linksPtrs.size(); q++ )
        {
            m_btWorldPtr->addCollisionObject( _linksPtrs[q]->colObj() );
        }

        // cache the reference for later usage
        m_simMultibodies.push_back( simMultibodyPtr );
    }

}