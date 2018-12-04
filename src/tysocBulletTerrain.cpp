
#include <tysocBulletTerrain.h>


namespace tysocBullet
{


    TBulletTerrainGenWrapper::TBulletTerrainGenWrapper( const std::string& name,
                                                        tysocterrain::TTerrainGenerator* terrainGenPtr )
    {
        m_name = name;
        m_terrainGenPtr = terrainGenPtr;

        // initialize mujoco resources
        for ( size_t i = 0; i < BULLET_TERRAIN_POOL_SIZE; i++ )
        {
            // @CHECK: This part should be generalized in order to accomodate ...
            // other types of primitives as needed (like meshes, add joints for doors, etc.)
            // For example, to make a door, instead of having a geomtype, we could use a generic type ...
            // like bulletPrimitiveType, which would be a "meta" for object creation. We then dispatch this ...
            // request of creation to an appropiate factory that builds the door for us, and we just wrapped it
            // For a mesh, it could be the same. The underlying data structure we want to wrap already has the info ...
            // (must have it) to build the mesh, so we only need to request that to the factory and voila, we have ...
            // our mesh. The same should apply for other types of terrain objects : the factories are the providers ...
            // for object creation, and they only need the information of what we want to build (and they could be ...
            // abstract factories, and could be instantiated for a concrete API implementation, either bullet or mujoco)
            auto _bulletPrimitive = new TBulletTerrainPrimitive();

            _bulletPrimitive->bulletBodyName        = std::string( "terrainGen_" ) + name + std::string( "_" ) + std::to_string( i );
            _bulletPrimitive->bulletGeomType        = "box";
            _bulletPrimitive->bulletGeomSize.x      = 0.5f * BULLET_TERRAIN_PATH_DEFAULT_WIDTH;
            _bulletPrimitive->bulletGeomSize.y      = 0.5f * BULLET_TERRAIN_PATH_DEFAULT_DEPTH;
            _bulletPrimitive->bulletGeomSize.z      = 0.5f * BULLET_TERRAIN_PATH_DEFAULT_TICKNESS;
            _bulletPrimitive->isAvailable           = true;

            float _startX = 0.0f;
            float _startY = 0.0f;
            float _startZ = 100.0f + i * ( BULLET_TERRAIN_PATH_DEFAULT_TICKNESS + 1.0f );

            _bulletPrimitive->bulletRigidBodyObj = _createBodyResource( _bulletPrimitive,
                                                                        _startX, _startY, _startZ );
            _bulletPrimitive->tysocPrimitiveObj = NULL;

            m_bulletTerrainPrimitives.push_back( _bulletPrimitive );
            m_bulletAvailablePrimitives.push( _bulletPrimitive );
        }
    }

    TBulletTerrainGenWrapper::~TBulletTerrainGenWrapper()
    {
        if ( m_terrainGenPtr )
        {
            // deletion of the base reosurces is in charge of the scenario
            m_terrainGenPtr = NULL;
        }

        while ( !m_bulletAvailablePrimitives.empty() )
        {
            m_bulletAvailablePrimitives.pop();
        }

        while ( !m_bulletWorkingPrimitives.empty() )
        {
            m_bulletWorkingPrimitives.pop();
        }

        for ( size_t i = 0; i < m_bulletTerrainPrimitives.size(); i++ )
        {
            delete m_bulletTerrainPrimitives[i];
            m_bulletTerrainPrimitives[i] = NULL;
        }
        m_bulletTerrainPrimitives.clear();
    }

    btRigidBody* TBulletTerrainGenWrapper::_createBodyResource( TBulletTerrainPrimitive* bPrimitivePtr,
                                                                float x, float y, float z )
    {
        // @CHECK: this should be delegated to an appropiate factory

        // Create the collision shape requested
        btCollisionShape* _rbCollisionShape = NULL;
        if ( bPrimitivePtr->bulletGeomType == "box" )
        {
            _rbCollisionShape = new btBoxShape( btVector3( 0.5f, 0.5f, 0.5f ) );
        }

        if ( !_rbCollisionShape )
        {
            std::cout << "WARNING> something went wrong while " 
                      << "creating the bodyresource in the TBulletTerrainGenWrapper" << std::endl;;
            return NULL;
        }

        // create the starting motion state
        btDefaultMotionState* _rbMotionState;
        {
            btTransform _rbTransform;
            _rbTransform.setIdentity();
            _rbTransform.setOrigin( btVector3( x, y, z ) );

            _rbMotionState = new btDefaultMotionState( _rbTransform );
        }

        // @CHECK: for now we are just using static objects. We have to change this
        // with some construction information from the underlying objects, like mass, etc.
        btScalar _rbMass = 0.0;// @TODO: For now just 0, will have to change this later
        btVector3 _rbInertia( 0.0, 0.0, 0.0 );
        if ( _rbMass != 0.0 )
        {
            _rbCollisionShape->calculateLocalInertia( _rbMass, _rbInertia );
        }
        
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo(
                                                            _rbMass,
                                                            _rbMotionState,
                                                            _rbCollisionShape,
                                                            _rbInertia );
        btRigidBody* _rbObjPtr = new btRigidBody( _rbConstructionInfo );
        _rbObjPtr->setRestitution( 0.9f );// @TODO: Again, this should be passes from the base
        _rbObjPtr->setFriction( 0.5f );// @TODO: Again, this should be passes from the base

        return _rbObjPtr;
    }

    void TBulletTerrainGenWrapper::initialize()
    {
        // @TODO: Add assert here
        if ( !m_bulletWorldPtr )
        {
            return;
        }

        // collect starting info from generator
        _collectFromGenerator();
        _collectFixedFromGenerator();

        // add all bullet resources to bullet world
        for ( size_t i = 0; i < m_bulletTerrainPrimitives.size(); i++ )
        {
            if ( m_bulletTerrainPrimitives[i]->bulletRigidBodyObj )
            {
                m_bulletWorldPtr->addRigidBody( m_bulletTerrainPrimitives[i]->bulletRigidBodyObj );
            }
        }
    }

    void TBulletTerrainGenWrapper::setBulletWorld( btDynamicsWorld* worldPtr )
    {
        m_bulletWorldPtr = worldPtr;
    }

    void TBulletTerrainGenWrapper::preStep()
    {
        _collectFromGenerator();

        // update the properties of all objects (if they have a primitive as reference)
        for ( size_t i = 0; i < m_bulletTerrainPrimitives.size(); i++ )
        {
            if ( m_bulletTerrainPrimitives[i]->tysocPrimitiveObj )
            {
                _updateProperties( m_bulletTerrainPrimitives[i] );
            }
        }
    }

    void TBulletTerrainGenWrapper::_collectFromGenerator()
    {
        auto _newPrimitivesQueue = m_terrainGenPtr->getJustCreated();

        while ( !_newPrimitivesQueue.empty() )
        {
            auto _newPrimitive = _newPrimitivesQueue.front();
            _newPrimitivesQueue.pop();

            _wrapNewPrimitive( _newPrimitive, true );
        }

        // flush the creation queue to avoid double references everywhere
        m_terrainGenPtr->flushJustCreatedQueue();
    }

    void TBulletTerrainGenWrapper::_collectFixedFromGenerator()
    {
        auto _fixedPrimitivesQueue = m_terrainGenPtr->getFixed();

        while ( !_fixedPrimitivesQueue.empty() )
        {
            auto _fixedPrimitive = _fixedPrimitivesQueue.front();
            _fixedPrimitivesQueue.pop();

            _wrapNewPrimitive( _fixedPrimitive, false );
        }

        // flush the fixed queue to avoid double references (the wrapper now holds the ref.)
        m_terrainGenPtr->flushFixedQueue();
    }

    void TBulletTerrainGenWrapper::_updateProperties( TBulletTerrainPrimitive* bulletTerrainPritimivePtr )
    {
        // @TODO: Refactor a bit to avoid doing repeated work (like changing sizes or activating revery step)
        // Even the position and orientation should be set only once (when the object is wrapped)

        auto _primitiveObj          = bulletTerrainPritimivePtr->tysocPrimitiveObj;
        auto _bulletRigidBodyObj    = bulletTerrainPritimivePtr->bulletRigidBodyObj;

        btTransform _transform;
        _transform.setIdentity();

        // @PORT: use the appropiate body position function
        btVector3 _position;
        _position.setX( _primitiveObj->pos.x );
        _position.setY( _primitiveObj->pos.y );
        _position.setZ( _primitiveObj->pos.z );
        _transform.setOrigin( _position );

        // @PORT: use the appropiate body orientation function
        btMatrix3x3 _rotation;
        createBtMat3( _primitiveObj->rotmat, _rotation );
        _transform.setBasis( _rotation );

        _bulletRigidBodyObj->setWorldTransform( _transform );

        // @PORT: use the appropiate body change size function
        btVector3 _scaling;
        _scaling.setX( _primitiveObj->size.x );
        _scaling.setY( _primitiveObj->size.y );
        _scaling.setZ( _primitiveObj->size.z );

        _bulletRigidBodyObj->getCollisionShape()->setLocalScaling( _scaling );

        // @PORT: use the appropiate activate function (disable deactivation, or similar)
        _bulletRigidBodyObj->setActivationState( DISABLE_DEACTIVATION );
    }

    void TBulletTerrainGenWrapper::_wrapNewPrimitive( tysocterrain::TTerrainPrimitive* primitivePtr, bool isReusable )
    {
        // if the pool is empty, force to recycle the last object
        if ( m_bulletAvailablePrimitives.empty() )
        {
            auto _oldest = m_bulletWorkingPrimitives.front();
            if ( _oldest->tysocPrimitiveObj )
            {
                m_terrainGenPtr->recycle( _oldest->tysocPrimitiveObj );
                _oldest->tysocPrimitiveObj = NULL;
                m_bulletWorkingPrimitives.pop();
            }
            m_bulletAvailablePrimitives.push( _oldest );
        }

        // grab the oldest available object
        auto _bulletPrimitive = m_bulletAvailablePrimitives.front();
        m_bulletAvailablePrimitives.pop();

        // link an object from the working queue with the requested new primitive
        _bulletPrimitive->tysocPrimitiveObj = primitivePtr;

        if ( isReusable )
        {
            // put it into the working queue
            m_bulletWorkingPrimitives.push( _bulletPrimitive );
        }
        else
        {
            // put it into the fixed queue
            m_bulletFixedPrimitives.push( _bulletPrimitive );
        }
    }
}