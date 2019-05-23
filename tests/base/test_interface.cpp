
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
        else if ( shape == "none" )
            _colShape = new btCompoundShape();
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
        else
        {
            m_graphicsObj = NULL;
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

        m_isRunning = true;
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

        // Initialize UI resources
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& _io = ImGui::GetIO(); (void) _io;
    #ifdef __APPLE__
        ImGui_ImplOpenGL3_Init( "#version 150" );
    #else
        ImGui_ImplOpenGL3_Init( "#version 130" );
    #endif
        ImGui_ImplGlfw_InitForOpenGL( m_graphicsApp->window()->getGLFWwindow(), false );
        ImGui::StyleColorsDark();
    }

    void ITestApplication::start()
    {
        _init();
        // Create wrappers for SimObj or other necessary resources
        _startInternal();
    }

    void ITestApplication::step()
    {
        if ( m_btWorldPtr && m_isRunning )
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

        // render the UI
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if ( true )
        {
            _renderUI();
        }

        ImGui::Render();
        int _ww, _wh;
        glfwGetFramebufferSize( m_graphicsApp->window()->getGLFWwindow(), &_ww, &_wh );
        glViewport( 0, 0, _ww, _wh );
        ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );

        m_graphicsApp->end();

    }

    void ITestApplication::_renderUI()
    {
        // Override this
    }

    void ITestApplication::togglePause()
    {
        m_isRunning = ( m_isRunning ) ? false : true;
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
        btVector3 _bInertia = btVector3( 0, 0, 0 );

        if ( _bMass != 0.0f )
            _bShape->calculateLocalInertia( _bMass, _bInertia );

        // create the multibody
        m_btMultibody = new btMultiBody( numLinks, _bMass, _bInertia, baseIsFixed, false );
        
        // create a SimMultibodyLink for the base
        auto _bCollider = new btMultiBodyLinkCollider( m_btMultibody, -1 );
        _bCollider->setCollisionShape( _bShape );
        _bCollider->getWorldTransform().setOrigin( position );

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
        btVector3 _linkInertia = btVector3( 0., 0., 0. );
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

            {
                m_btMultibody->getLink( linkIndx ).m_jointLowerLimit = -1.57;
                m_btMultibody->getLink( linkIndx ).m_jointUpperLimit = 1.57;
                
                auto _constraint = new btMultiBodyJointMotor( m_btMultibody, linkIndx, 0, 0, 5. );
                _constraint->setErp( 0.1 );
                _constraint->setPositionTarget( 0.25 );

                m_constraints.push_back( _constraint );
                m_motors.push_back( _constraint );
            }
        }
        else if ( jointType == "slider" || jointType == "slide" || jointType == "prismatic" )
        {
            m_btMultibody->setupPrismatic( linkIndx,
                                           _linkMass,
                                           _linkInertia,
                                           _linkParentIndx,
                                           _parent2this_quat,
                                           jointAxis,
                                           _pivot2parent_pos,
                                           -jointPivot,
                                           true );

            {
                m_btMultibody->getLink( linkIndx ).m_jointLowerLimit = -10.;
                m_btMultibody->getLink( linkIndx ).m_jointUpperLimit = 10.;
                
                auto _constraint = new btMultiBodyJointMotor( m_btMultibody, linkIndx, 0, 0, 5. );
                _constraint->setErp( 0.1 );
                _constraint->setPositionTarget( 0. );

                m_constraints.push_back( _constraint );
                m_motors.push_back( _constraint );
            }
        }
        else if ( jointType == "planar" )
        {
            m_btMultibody->setupPlanar( linkIndx,
                                        _linkMass,
                                        _linkInertia,
                                        _linkParentIndx,
                                        _parent2this_quat,
                                        jointAxis,
                                        _pivot2parent_pos,
                                        true );
        }
        else if ( jointType == "ball" || jointType == "spheric" || jointType == "spherical" )
        {
            m_btMultibody->setupSpherical( linkIndx,
                                           _linkMass,
                                           _linkInertia,
                                           _linkParentIndx,
                                           _parent2this_quat,
                                           _pivot2parent_pos,
                                           -jointPivot,
                                           true );
        }
        else if ( jointType == "fixed" )
        {
            m_btMultibody->setupFixed( linkIndx,
                                       _linkMass,
                                       _linkInertia,
                                       _linkParentIndx,
                                       _parent2this_quat,
                                       _pivot2parent_pos,
                                       -jointPivot,
                                       true );
        }
        else
        {
            std::cout << "ERROR> joint type: " << jointType << " not supported" << std::endl;
        }

        // collider managment
        auto _linkCollider = new btMultiBodyLinkCollider( m_btMultibody, linkIndx );
        _linkCollider->setCollisionShape( _linkShape );
        m_btMultibody->getLink( linkIndx ).m_collider = _linkCollider;

        // initialize worldtransform from parent
        auto _parentWorldTransform = parentObj->colObj()->getWorldTransform();
        _linkCollider->setWorldTransform( _parentWorldTransform * localTransform );

        // create SimMultibodyLink wrapper
        auto _linkSimMultibodyLink = new SimMultibodyLink( _linkCollider, linkIndx, parentObj );

        // cache the simlink for later usage
        m_simLinks.push_back( _linkSimMultibodyLink );

        return _linkSimMultibodyLink;
    }

    std::vector< SimMultibodyLink* > SimMultibody::setupLinkMultiDof( int linkIndx,
                                                                      const std::string& shapeType,
                                                                      const btVector3& shapeSize,
                                                                      const btTransform& localTransform,
                                                                      SimMultibodyLink* parentObj,
                                                                      const std::vector< std::string >& jointsTypes,
                                                                      const std::vector< btVector3 >& jointsAxis,
                                                                      const std::vector< btVector3 >& jointsPivots )
    {
        std::vector< SimMultibodyLink* > _links;

        if ( !m_btMultibody )
        {
            std::cout << "ERROR> no btMultibody created for this object" << std::endl;
            return _links;
        }

        if ( !parentObj )
        {
            std::cout << "ERROR> this link (non-root) has no parent" << std::endl;
            return _links;
        }

        // sanity check (joints info arrays should be of the same size)
        if ( jointsTypes.size() != jointsAxis.size() ||
             jointsAxis.size() != jointsPivots.size() )
        {
            std::cout << "ERROR> joints info arrays should have the same size" << std::endl;
            return _links;
        }

        // make sure the user sent some actual info
        if ( jointsTypes.size() == 0 )
        {
            std::cout << "ERROR> must pass joint data for multidof setup" << std::endl;
            return _links;
        }

        // also, make sure that the user is not passing a single joint, as it ...
        // would be better of using the setupLinkSingleJoint method.
        if ( jointsTypes.size() == 1 )
        {
            std::cout << "ERROR> should pass more than one dof for multidof setup" << std::endl;
            return _links;
        }

        // Create as many links as dofs requested (dummies) + 1 for ...
        // the actual link (fixed) which does have inertial props
        size_t _numLinks = jointsTypes.size() + 1;

        // grab the origin and orientation from the local transform, as ...
        // we will use them to define the transforms of the links
        auto _localOrigin   = localTransform.getOrigin();
        auto _localRotation = localTransform.getRotation();

        // a running reference to the parent, initialized to the given parent
        auto _parentObj = parentObj;
        for ( size_t q = 0; q < _numLinks; q++ )
        {
            SimMultibodyLink* _link = NULL;

            if ( q == 0 )
            {
                /*  The first dummy link is constructed w.r.t. the parent, so its ...
                *   transforms should be computed w.r.t. the given parent link
                */

                // The transform for this dummy link has to the same orientation ...
                // (w.r.t. the parent body) of the true link, and as origin the ... 
                // pivot of each dof w.r.t. the parent
                btTransform _transform;
                _transform.setIdentity();
                // set same rotation as true body
                _transform.setRotation( _localRotation );
                // set the origin of the dummy body
                auto _origin = _localOrigin + quatRotate( _localRotation, jointsPivots[q] );
                _transform.setOrigin( _origin );

                // setup a dummy link
                _link = setupLinkSingleJoint( linkIndx + q,
                                              "none",               // recall: dummy link
                                              { 0., 0., 0. },       // recall: dummy link
                                              _transform,
                                              _parentObj,
                                              jointsTypes[q],
                                              jointsAxis[q],        // axis remains in true local-frame
                                              { 0., 0., 0., } );    // pivot coincides with origin
            }
            else if ( q < ( _numLinks - 1 ) )
            {
                /*  The next dummy links are constructed w.r.t. the previous dummy ...
                *   links. These have basically the same orientation and only differ ...
                *   in the pivot.
                */

                // The transform for this dummy link has the same orientation ...
                // (w.r.t. the parent body) as the previous dummy link, and as ...
                // origin the relative position of its pivot w.r.t. the pivot ...
                // of the previous dummy link.
                btTransform _transform;
                _transform.setIdentity();
                // set only relative position w.r.t. previous dummy's pivot
                auto _origin = jointsPivots[q] - jointsPivots[q-1];
                _transform.setOrigin( _origin );

                // setup a dummy link
                _link = setupLinkSingleJoint( linkIndx + q,
                                              "none",               // recall: dummy link
                                              { 0., 0., 0. },       // recall: dummy link
                                              _transform,
                                              _parentObj,
                                              jointsTypes[q],
                                              jointsAxis[q],        // axis remains in true local-frame
                                              { 0., 0., 0., } );    // pivot coincides with origin
            }
            else
            {
                /*  The last link (which is not a dummy) is constructed w.r.t. the ...
                *   previous dummy link. These two have basically the same orientation ...
                *   and only differ in the pivot.
                */

                // The transform for this last link has the same orientation ...
                // (w.r.t. the parent body) as the previous dummy link, and as ...
                // origin the relative position of its pivot w.r.t. the pivot ...
                // of the previous dummy link.
                btTransform _transform;
                _transform.setIdentity();
                // set only relative position w.r.t. previous dummy's pivot
                auto _origin = -jointsPivots[q-1];
                _transform.setOrigin( _origin );

                // setup the actual link (has mass/inertia info)
                _link = setupLinkSingleJoint( linkIndx + q,
                                              shapeType,
                                              shapeSize,
                                              _transform,
                                              _parentObj,
                                              "fixed",
                                              { 1., 0., 0. },
                                              { 0., 0., 0. } );
            }

            if ( !_link )
            {
                std::cout << "ERROR> something went wrong creating the multidof links" << std::endl;
                break;
            }

            _parentObj = _link;

            _links.push_back( _link );
        }

        return _links;
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

    std::vector< btMultiBodyConstraint* > SimMultibody::constraintsPtrs()
    {
        return m_constraints;
    }

    std::vector< btMultiBodyJointMotor* > SimMultibody::motorsPtrs()
    {
        return m_motors;
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

            if ( !_mesh )
                continue;

            auto _material = _mesh->getMaterial();
            _material->setColor( { 0.7, 0.5, 0.3 } );

            m_graphicsScene->addRenderable( _mesh );
        }

        // add all multibody-links resources (even base) to the world
        for ( size_t q = 0; q < _linksPtrs.size(); q++ )
        {
            m_btWorldPtr->addCollisionObject( _linksPtrs[q]->colObj() );
        }

        // add all constraints resources
        auto _constraintsPtrs = simMultibodyPtr->constraintsPtrs();
        for ( size_t q = 0; q < _constraintsPtrs.size(); q++ )
        {
            reinterpret_cast< btMultiBodyDynamicsWorld* >
                            ( m_btWorldPtr )->addMultiBodyConstraint( _constraintsPtrs[q] );
        }

        // cache the reference for later usage
        m_simMultibodies.push_back( simMultibodyPtr );
    }

    void MultibodyTestApplication::_renderUI()
    {
        ImGui::Begin( "Kinematic Tree actuator options" );

        auto _motors = m_simMultibodies.back()->motorsPtrs();

        for ( size_t q = 0; q < _motors.size(); q++ )
        {
            std::string _motorId = "motor(" + std::to_string( q ) + ")";
            float _val = 0.0f;
            ImGui::SliderFloat( _motorId.c_str(), 
                                &_val,
                                -1.57,
                                1.57 );
            
            _motors[q]->setPositionTarget( _val );
        }

        ImGui::End();
    }

}