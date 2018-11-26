
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <memory>

#include <btBulletDynamicsCommon.h>

struct Vec3
{
    float x;
    float y;
    float z;
};

struct BulletContext
{
    btDiscreteDynamicsWorld*                world;
    btSequentialImpulseConstraintSolver*    constraintSolver;
    btCollisionDispatcher*                  collisionDispatcher;
    btDefaultCollisionConfiguration*        collisionConfiguration;
    btBroadphaseInterface*                  broadphaseInterface;
    bool                                    active;
};

BulletContext g_bContext;


struct BodyWrapper;
struct JointWrapper;

struct BodyWrapper
{
    btCollisionShape*               collShapeObj;
    btRigidBody*                    bodyObj;
    engine::LMesh*                  meshObj;
    JointWrapper*                   parentJoint;
    std::vector< JointWrapper* >    childJoints;
    std::string                     name;
};

enum BodyType // Assumed to be equivalent to collision type
{
    BODY_TYPE_INFINITE_PLANE,
    BODY_TYPE_SPHERE,
    BODY_TYPE_BOX,
    BODY_TYPE_CYLINDER,
    BODY_TYPE_CAPSULE,
    BODY_TYPE_TRIANGLE_MESH
};

// @TODO: Just in case, check that the matrices are aligned, to
// potentionally avoid weird behaviour (maybe pragma struct pack)
union BodyShapeParams
{
    struct { float nx; float ny; float nz; } infplane;
    struct { float radius; } sphere;
    struct { float width; float depth; float height; } box;
    struct { float radius; float length; } cylinder;
    struct { float radius; float length; } capsule;
    struct 
    { 
        engine::u8* vertexData; 
        engine::u8* indexData;
        int vertexStride;
        int indexStride;
    } triangleMesh;
};

struct BodyConstructionInfo
{
    BodyType btype;
    BodyShapeParams shapeData;
    float mass;
    float restitution;
    float friction;
    struct { float r; float g; float b; } color;
};

void mat4Tobtmat3( const engine::LMat4& mat4, btMatrix3x3& btmat3 )
{
    for ( size_t i = 0; i < 3; i++ )
    {
        for ( size_t j = 0; j < 3; j++ )
        {
            btmat3[i][j] = mat4.get( i, j );
        }
    }
}

void vec3Tobtvec3( const engine::LVec3& vec3, btVector3& btvec3 )
{
    btvec3[0] = vec3.x;
    btvec3[1] = vec3.y;
    btvec3[2] = vec3.z;
}

void vec3Tobtvec3( const Vec3& vec3, btVector3& btvec3 )
{
    btvec3[0] = vec3.x;
    btvec3[1] = vec3.y;
    btvec3[2] = vec3.z;
}

engine::LVec3 btvec3Tovec3( const btVector3& btvec3 )
{
    return engine::LVec3( btvec3.x(), btvec3.y(), btvec3.z() );
}

engine::LMat4 btmat3Tomat4( const btMatrix3x3& btmat3 )
{
    engine::LMat4 _mat4;

    for ( size_t i = 0; i < 3; i++ )
    {
        for ( size_t j = 0; j < 3; j++ )
        {
            _mat4.set( i, j, btmat3[i][j] );
        }
    }

    return _mat4;
}



BodyWrapper* createBody( const std::string& name,
                         const BodyConstructionInfo& cInfo,
                         const engine::LVec3& position,
                         const engine::LMat4& rotation = engine::LMat4() )
{
    auto _body = new BodyWrapper();
    _body->name = name;

    if ( cInfo.btype == BODY_TYPE_INFINITE_PLANE )
    {
        _body->collShapeObj = new btStaticPlaneShape( btVector3( cInfo.shapeData.infplane.nx,
                                                                 cInfo.shapeData.infplane.ny,
                                                                 cInfo.shapeData.infplane.nz ), 0 );
        _body->meshObj = engine::LMeshBuilder::createPlane( 30.0f, 30.0f );
    }
    else if ( cInfo.btype == BODY_TYPE_SPHERE )
    {
        _body->collShapeObj = new btSphereShape( cInfo.shapeData.sphere.radius );
        _body->meshObj = engine::LMeshBuilder::createSphere( cInfo.shapeData.sphere.radius );
    }
    else if ( cInfo.btype == BODY_TYPE_BOX )
    {
        _body->collShapeObj = new btBoxShape( btVector3( cInfo.shapeData.box.width / 2,
                                                         cInfo.shapeData.box.depth / 2,
                                                         cInfo.shapeData.box.height / 2 ) );
        _body->meshObj = engine::LMeshBuilder::createBox( cInfo.shapeData.box.width,
                                                          cInfo.shapeData.box.depth,
                                                          cInfo.shapeData.box.height );
    }
    else if ( cInfo.btype == BODY_TYPE_CYLINDER )
    {
        _body->collShapeObj = new btCylinderShape( btVector3( cInfo.shapeData.cylinder.radius,
                                                              cInfo.shapeData.cylinder.length / 2,
                                                              cInfo.shapeData.cylinder.radius ) );
        _body->meshObj = engine::LMeshBuilder::createCylinder( cInfo.shapeData.cylinder.radius,
                                                               cInfo.shapeData.cylinder.length );
    }
    else if ( cInfo.btype == BODY_TYPE_CAPSULE )
    {
        _body->collShapeObj = new btCapsuleShape( cInfo.shapeData.capsule.radius,
                                                  cInfo.shapeData.capsule.length );
        _body->meshObj = engine::LMeshBuilder::createCapsule( cInfo.shapeData.capsule.radius,
                                                              cInfo.shapeData.capsule.length );
    }
    // else if ( cInfo.btype == BODY_TYPE_TRIANGLE_MESH )
    // {
        
    // }
    else
    {
        std::cout << "WARNING> body with type: " 
                  << cInfo.btype << " not supported" << std::endl;
        return NULL;
    }

    _body->meshObj->getMaterial()->setColor( cInfo.color.r, cInfo.color.g, cInfo.color.b );

    btDefaultMotionState* _rbMotionState;
    {
        btTransform _rbTransform;
        btVector3 _btvec3;
        btMatrix3x3 _btmat3;
        mat4Tobtmat3( rotation, _btmat3 );
        vec3Tobtvec3( position, _btvec3 );

        _rbTransform.setIdentity();
        _rbTransform.setOrigin( _btvec3 );
        _rbTransform.setBasis( _btmat3 );

        _rbMotionState = new btDefaultMotionState( _rbTransform );

        _body->meshObj->pos = position;
        _body->meshObj->rotation = rotation;
    }

    btScalar _rbMass = cInfo.mass;
    btVector3 _rbInertia( 0, 0, 0 );
    {
        if ( cInfo.mass != 0.0f )
        {
            _body->collShapeObj->calculateLocalInertia( _rbMass, _rbInertia );
        }
    }

    btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo( 
                                                    _rbMass,
                                                    _rbMotionState,
                                                    _body->collShapeObj,
                                                    _rbInertia );

    _body->bodyObj = new btRigidBody( _rbConstructionInfo );
    _body->bodyObj->setRestitution( cInfo.restitution );
    _body->bodyObj->setFriction( cInfo.friction );

    return _body;
}

enum JointType
{
    JOINT_TYPE_REVOLUTE
};

struct JointWrapper
{
    btTypedConstraint*  jointObj;
    engine::LMesh*      meshObj;
    BodyWrapper*        parentBody;
    BodyWrapper*        childBody;
    JointType           jtype;
    std::string         name;
};

union JointParams
{
    struct 
    { 
        Vec3 pivotInA; 
        Vec3 pivotInB; 
        Vec3 axisInA;
        Vec3 axisInB;
        bool useReferenceFrameA;
        float vradius;
        float vlength;
    } hinge;
};

struct JointConstructionInfo
{
    JointType       jtype;
    JointParams     jointData;
    struct { float r; float g; float b; } color;
};

JointWrapper* createJoint( const std::string& name, BodyWrapper* parent, BodyWrapper* child, JointConstructionInfo& cInfo )
{
    auto _joint = new JointWrapper();

    _joint->name        = name;
    _joint->parentBody  = parent;
    _joint->parentBody  = child;
    _joint->jtype       = cInfo.jtype;

    if ( cInfo.jtype == JOINT_TYPE_REVOLUTE )
    {
        btVector3 _pivA, _pivB, _axA, _axB;
        vec3Tobtvec3( cInfo.jointData.hinge.pivotInA, _pivA );
        vec3Tobtvec3( cInfo.jointData.hinge.pivotInB, _pivB );
        vec3Tobtvec3( cInfo.jointData.hinge.axisInA, _axA );
        vec3Tobtvec3( cInfo.jointData.hinge.axisInB, _axB );
        

        _joint->jointObj = new btHingeConstraint( *parent->bodyObj, *child->bodyObj,
                                                  _pivA, _pivB,  _axA, _axB,
                                                  cInfo.jointData.hinge.useReferenceFrameA );

        _joint->meshObj = engine::LMeshBuilder::createCylinder( cInfo.jointData.hinge.vradius,
                                                                cInfo.jointData.hinge.vlength );
        // _joint->meshObj = engine::LMeshBuilder::createSphere( cInfo.jointData.hinge.vradius );
        _joint->meshObj->getMaterial()->setColor( cInfo.color.r, cInfo.color.g, cInfo.color.b );
    }
    else
    {
        std::cout << "WARNING> joint with type: " 
                  << cInfo.jtype << " not supported" << std::endl;
        return NULL;
    }

    return _joint;
}

struct Pendulum
{
    BodyWrapper*                    root;
    std::vector< BodyWrapper* >     bodies;
    std::vector< JointWrapper* >    joints;
};

const float PENDULUM_BASE_WIDTH = 0.25f;
const float PENDULUM_BASE_DEPTH = 0.25f;
const float PENDULUM_BASE_HEIGHT = 0.5f;

const float PENDULUM_LINK_WIDTH = 0.1f;
const float PENDULUM_LINK_DEPTH = 0.1f;
const float PENDULUM_LINK_HEIGHT = 0.5f;

void _createRoot( Pendulum& pendulum, const Vec3& position = { 0, 0, 2 } )
{
    BodyConstructionInfo _cInfo;
    _cInfo.btype = BODY_TYPE_BOX;
    _cInfo.mass = 0.0f;
    _cInfo.restitution = 0.8;
    _cInfo.friction = 1.0;
    _cInfo.shapeData.box.width = PENDULUM_BASE_WIDTH;
    _cInfo.shapeData.box.depth = PENDULUM_BASE_DEPTH;
    _cInfo.shapeData.box.height = PENDULUM_BASE_HEIGHT;
    _cInfo.color = { 0.1f, 0.9f, 0.2f };

    auto _root = createBody( "root", _cInfo, engine::LVec3( position.x, position.y, position.z ) );
    _root->bodyObj->setDamping( 0, 0 );
    pendulum.bodies.push_back( _root );

    pendulum.root = _root;
}

#define RANDOM_COLOR { ( (float)rand() / RAND_MAX ), ( (float)rand() / RAND_MAX ), ( (float)rand() / RAND_MAX ) }

void _createLink( Pendulum& pendulum, BodyWrapper* parent, const Vec3& position = { 0, 0, 1 } )
{
    BodyConstructionInfo _linkInfo;
    _linkInfo.btype = BODY_TYPE_BOX;
    _linkInfo.mass = 1.0f;
    _linkInfo.restitution = 0.8;
    _linkInfo.friction = 1.0;
    _linkInfo.shapeData.box.width = PENDULUM_LINK_WIDTH;
    _linkInfo.shapeData.box.depth = PENDULUM_LINK_DEPTH;
    _linkInfo.shapeData.box.height = PENDULUM_LINK_HEIGHT;
    _linkInfo.color = RANDOM_COLOR;

    auto _link = createBody( std::string( "link_" ) + std::to_string( pendulum.bodies.size() ) ,_linkInfo, engine::LVec3( position.x, position.y, position.z ) );
    _link->bodyObj->setDamping( 0, 0 );
    pendulum.bodies.push_back( _link );

    float _parentOffset = ( parent == pendulum.root ) ? -0.5f * PENDULUM_BASE_HEIGHT :
                                                        -0.5f * PENDULUM_LINK_HEIGHT;

    JointConstructionInfo _jointInfo;
    _jointInfo.jtype = JOINT_TYPE_REVOLUTE;
    _jointInfo.jointData.hinge.axisInA = { 1, 0, 0 };
    _jointInfo.jointData.hinge.axisInB = { 1, 0, 0 };
    _jointInfo.jointData.hinge.pivotInA = { 0, 0, _parentOffset };
    _jointInfo.jointData.hinge.pivotInB = { 0, 0, 0.5f * PENDULUM_LINK_HEIGHT };
    _jointInfo.jointData.hinge.useReferenceFrameA = true;
    _jointInfo.jointData.hinge.vradius = 0.1f;
    _jointInfo.jointData.hinge.vlength = 0.15f;
    _jointInfo.color = { 0.5f, 0.5f, 0.1f };

    auto _joint = createJoint( std::string( "joint_" ) + std::to_string( pendulum.joints.size() ), parent, _link, _jointInfo );
    pendulum.joints.push_back( _joint );
}

void createPendulum( Pendulum& pendulum )
{
    Vec3 _position = { 2, 2, 4 };
    _createRoot( pendulum, _position );
    _position.z -= 0.5 * PENDULUM_BASE_HEIGHT;

    for ( size_t i = 0; i < 6; i++ )
    {
        _position.z -= 0.5 * PENDULUM_LINK_HEIGHT;
        _createLink( pendulum, pendulum.bodies.back(), _position );
        _position.z -= 0.5 * PENDULUM_LINK_HEIGHT;
    }
}

void _printbtvec3( const btVector3& btvec3 )
{
    std::cout << "v: " << btvec3.x() << " - " << btvec3.y() << " - " << btvec3.z() << std::endl;
}

void _printbtmat3( const btMatrix3x3& btmat3 )
{
    for ( size_t i = 0; i < 3; i++ )
    {
        _printbtvec3( btmat3[i] );
    }
}

void _printTransform( const btTransform& transform )
{
    auto _position = transform.getOrigin();
    auto _rotation = transform.getBasis();

    std::cout << "origin" << std::endl;
    _printbtvec3( _position );
    std::cout << "rotation" << std::endl;
    _printbtmat3( _rotation );
}

void _drawJointRevolute( JointWrapper* joint )
{
    // std::cout << "joint: " << joint->name << std::endl;
    auto _parent2jointTransform = reinterpret_cast<btHingeConstraint*>( joint->jointObj )->getFrameOffsetA();
    auto _parentMotionState = reinterpret_cast<btHinge2Constraint*>( joint->jointObj )->getRigidBodyA().getMotionState();
    btTransform _parentTransform;
    _parentMotionState->getWorldTransform( _parentTransform );
    // auto _child2jointTransform = reinterpret_cast<btHingeConstraint*>( joint->jointObj )->getFrameOffsetB();

    // std::cout << "parent2joint" << std::endl;
    // _printTransform( _parent2jointTransform );

    // std::cout << "child2joint" << std::endl;
    // _printTransform( _child2jointTransform );

    auto _jointWorldTransform = _parentTransform * _parent2jointTransform;
    joint->meshObj->pos = btvec3Tovec3( _jointWorldTransform.getOrigin() );
    joint->meshObj->rotation = btmat3Tomat4( _jointWorldTransform.getBasis() );
}

void drawJoint( JointWrapper* joint )
{
    if ( joint->jtype == JOINT_TYPE_REVOLUTE )
    {
        _drawJointRevolute( joint );
    }
}


engine::LApp* g_app;


int main()
{
    g_bContext.broadphaseInterface      = new btDbvtBroadphase();
    g_bContext.collisionConfiguration   = new btDefaultCollisionConfiguration();
    g_bContext.collisionDispatcher      = new btCollisionDispatcher( g_bContext.collisionConfiguration );
    g_bContext.constraintSolver         = new btSequentialImpulseConstraintSolver();
    g_bContext.world                    = new btDiscreteDynamicsWorld(
                                                g_bContext.collisionDispatcher,
                                                g_bContext.broadphaseInterface,
                                                g_bContext.constraintSolver,
                                                g_bContext.collisionConfiguration );

    g_bContext.world->setGravity( btVector3( 0, 0, -10 ) );

    g_app = engine::LApp::GetInstance();
    auto _scene = g_app->scene();
    
    // make a sample camera
    // auto _camera = new engine::LFpsCamera( "fps",
    //                                        engine::LVec3( 1.0f, 2.0f, -1.0f ),
    //                                        engine::LVec3( -2.0f, -4.0f, -2.0f ),
    //                                        engine::LICamera::UP_Z );
    auto _camera = new engine::LFixedCamera3d( "fixed",
                                               engine::LVec3( 6.0f, 12.0f, 6.0f ),
                                               engine::LVec3( 0.0f, 0.0f, 0.0f ),
                                               engine::LICamera::UP_Z );

    // make a sample light source
    auto _light = new engine::LLightDirectional( engine::LVec3( 0.2, 0.2, 0.2 ), 
                                                 engine::LVec3( 0.8, 0.8, 0.8 ),
                                                 engine::LVec3( 0.15, 0.15, 0.15 ), 
                                                 0, 
                                                 engine::LVec3( -1, -1, -1 ) );
    _light->setVirtualPosition( engine::LVec3( 5, 0, 5 ) );

    // add these components to the scene
    _scene->addCamera( _camera );
    _scene->addLight( _light );

    // create some bodies and joints
    std::vector< BodyWrapper* > _bodies;
    std::vector< JointWrapper* > _joints;

    // A plane
    {
        BodyConstructionInfo _planeInfo;
        _planeInfo.btype = BODY_TYPE_INFINITE_PLANE;
        _planeInfo.mass = 0.0f;
        _planeInfo.restitution = 0.8;
        _planeInfo.friction = 1.0;
        _planeInfo.shapeData.infplane.nx = 0;
        _planeInfo.shapeData.infplane.ny = 0;
        _planeInfo.shapeData.infplane.nz = 1;
        _planeInfo.color = { 0.2f, 0.1f, 0.8f };

        _bodies.push_back( createBody( "plane",
                                       _planeInfo,
                                       engine::LVec3( 0, 0, 0 ) ) );
    }

    const float BOX_WIDTH = 0.25f;
    const float BOX_DEPTH = 0.25f;
    const float BOX_HEIGHT = 0.5f;

    const float LINK_WIDTH = 0.1f;
    const float LINK_DEPTH = 0.1f;
    const float LINK_HEIGHT = 0.5f;

    // A box
    BodyWrapper* _base;
    {
        BodyConstructionInfo _boxInfo;
        _boxInfo.btype = BODY_TYPE_BOX;
        _boxInfo.mass = 0.0f;
        _boxInfo.restitution = 0.8;
        _boxInfo.friction = 1.0;
        _boxInfo.shapeData.box.width = BOX_WIDTH;
        _boxInfo.shapeData.box.depth = BOX_DEPTH;
        _boxInfo.shapeData.box.height = BOX_HEIGHT;
        _boxInfo.color = { 0.1f, 0.9f, 0.2f };

        _base = createBody( "base", _boxInfo, { 0, 0, 2 } );
        _bodies.push_back( _base );

        _base->bodyObj->setDamping( 0, 0 );
    }

    // Create a child box attached to the previous box by a hinge joint
    BodyWrapper* _link;
    JointWrapper* _joint;
    {
        BodyConstructionInfo _linkInfo;
        _linkInfo.btype = BODY_TYPE_BOX;
        _linkInfo.mass = 1.0f;
        _linkInfo.restitution = 0.8;
        _linkInfo.friction = 1.0;
        _linkInfo.shapeData.box.width = LINK_WIDTH;
        _linkInfo.shapeData.box.depth = LINK_DEPTH;
        _linkInfo.shapeData.box.height = LINK_HEIGHT;
        _linkInfo.color = { 0.8f, 0.7f, 0.1f };

        _link = createBody( "link", _linkInfo, { 0, 0, 1 } );
        _bodies.push_back( _link );

        _link->bodyObj->setDamping( 0, 0 );

        JointConstructionInfo _jointInfo;
        _jointInfo.jtype = JOINT_TYPE_REVOLUTE;
        _jointInfo.jointData.hinge.axisInA = { 1, 0, 0 };
        _jointInfo.jointData.hinge.axisInB = { 1, 0, 0 };
        _jointInfo.jointData.hinge.pivotInA = { 0, 0, -0.5f * BOX_HEIGHT };
        _jointInfo.jointData.hinge.pivotInB = { 0, 0, 0.5f * LINK_HEIGHT };
        _jointInfo.jointData.hinge.useReferenceFrameA = true;
        _jointInfo.jointData.hinge.vradius = 0.1f;
        _jointInfo.jointData.hinge.vlength = 0.1f;
        _jointInfo.color = { 0.5f, 0.5f, 0.1f };

        _joint = createJoint( "joint", _base, _link, _jointInfo );
        g_bContext.world->addConstraint( _joint->jointObj, true );
    }

    Pendulum _pendulum;
    createPendulum( _pendulum );

    for ( size_t i = 0; i < _pendulum.bodies.size(); i++ )
    {
        _bodies.push_back( _pendulum.bodies[i] );
    }
    for( size_t i = 0; i < _pendulum.joints.size(); i++ )
    {
        _joints.push_back( _pendulum.joints[i] );
    }

    for ( size_t i = 0; i < _bodies.size(); i++ )
    {
        g_bContext.world->addRigidBody( _bodies[i]->bodyObj );
        _bodies[i]->bodyObj->setActivationState( DISABLE_DEACTIVATION );
        _scene->addRenderable( _bodies[i]->meshObj );
    }

    for ( size_t i = 0; i < _joints.size(); i++ )
    {
        g_bContext.world->addConstraint( _joints[i]->jointObj, true );
        _scene->addRenderable( _joints[i]->meshObj );
    }

    g_bContext.active = true;

    while( g_app->isActive() )
    {
        if ( g_bContext.active )
        {
            g_bContext.world->stepSimulation( 1.0f / 60.0f );
        }

        {
            // Draw xyz axes
            engine::DebugSystem::drawLine( { 0, 0, 0 }, { 10, 0, 0 }, { 1, 0, 0 } );
            engine::DebugSystem::drawLine( { 0, 0, 0 }, { 0, 10, 0 }, { 0, 1, 0 } );
            engine::DebugSystem::drawLine( { 0, 0, 0 }, { 0, 0, 10 }, { 0, 0, 1 } );
        }

        for ( size_t i = 0; i < _bodies.size(); i++ )
        {
            btTransform _transform;
            _bodies[i]->bodyObj->getMotionState()->getWorldTransform( _transform );
            
            _bodies[i]->meshObj->pos = btvec3Tovec3( _transform.getOrigin() );
            _bodies[i]->meshObj->rotation = btmat3Tomat4( _transform.getBasis() );
        }

        for ( size_t i = 0; i < _joints.size(); i++ )
        {
            drawJoint( _joints[i] );
        }

        g_app->update();

        if ( engine::InputSystem::isKeyDown( GLFW_KEY_ESCAPE ) )
        {
            g_app->window()->requestClose();
        }
        else if ( engine::InputSystem::isKeyDown( GLFW_KEY_SPACE ) )
        {
            g_bContext.active = false;
        }
        else if ( engine::InputSystem::isKeyDown( GLFW_KEY_ENTER ) )
        {
            g_bContext.active = true;
        }
        else if ( engine::InputSystem::isKeyDown( GLFW_KEY_T ) )
        {
            _link->bodyObj->applyTorque( btVector3( 1, 0, 0 ) );
            // _pendulum.bodies[1]->bodyObj->applyTorque( btVector3( 1, 0, 0 ) );
            for ( size_t i = 0; i < _pendulum.bodies.size(); i++ )
            {
                _pendulum.bodies[i]->bodyObj->applyTorque( btVector3( 4, 0, 0 ) );
            }
        }
    }

    // delete g_app;

    return 0;   
}