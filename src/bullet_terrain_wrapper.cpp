
#include <bullet_terrain_wrapper.h>


namespace tysoc {
namespace bullet {


    TBtTerrainGenWrapper::TBtTerrainGenWrapper( terrain::TITerrainGenerator* terrainGenPtr, 
                                                const std::string& workingDir )
        : TTerrainGenWrapper( terrainGenPtr, workingDir )
    {
        m_btWorldPtr = NULL;

        // Create resources that will be fixed|static (no pool needed)
        _collectStaticFromGenerator();

        // @TODO: Commented due to some compatibility we wanted to try out with ...
        // the statis terrain generators and the mujoco-py environments. The ...
        // problem was that those environments extract vectorized information ...
        // using directly the mjData structure, and if we enabled pooling, some ...
        // information would go into this vectorized information grabbed from ...
        // mjData (like mjData->qpos, etc.). We will fix this issue later by ...
        // filtering out the non-working primitives (which remain in the pool) ...
        // and pass the information required only from the active primities.

//        // Create resources that will be reused *********************************
//        for ( size_t i = 0; i < BT_TERRAIN_POOL_SIZE; i++ )
//        {
//            auto _btPrimitive = new TBtTerrainPrimitive();
//            auto _name = std::string( "tGen_" ) + name() + 
//                         std::string( "_" ) + std::to_string( i + m_btTerrainPrimitives.size() );
//
//            _btPrimitive->btBodyId        = -1;
//            _btPrimitive->btGeomName      = _name;
//            _btPrimitive->btGeomType      = "box";
//            _btPrimitive->btGeomSize      = { 3, { 0.5f * BT_TERRAIN_PATH_DEFAULT_WIDTH, 
//                                                    0.5f * BT_TERRAIN_PATH_DEFAULT_DEPTH, 
//                                                    0.5f * BT_TERRAIN_PATH_DEFAULT_TICKNESS } };
//
//            _btPrimitive->tysocPrimitiveObj = NULL;
//
//            m_btTerrainPrimitives.push_back( _btPrimitive );
//            m_btAvailablePrimitives.push( _btPrimitive );
//        }
//
//        // collect starting info from generator
//        _collectReusableFromGenerator();
//        // **********************************************************************
    }

    TBtTerrainGenWrapper::~TBtTerrainGenWrapper()
    {
        m_btWorldPtr = NULL;

        while ( !m_btAvailablePrimitives.empty() )
        {
            m_btAvailablePrimitives.pop();
        }

        while ( !m_btWorkingPrimitives.empty() )
        {
            m_btWorkingPrimitives.pop();
        }

        for ( size_t i = 0; i < m_btTerrainPrimitives.size(); i++ )
        {
            delete m_btTerrainPrimitives[i];
            m_btTerrainPrimitives[i] = NULL;
        }
        m_btTerrainPrimitives.clear();
    }

    void TBtTerrainGenWrapper::setBtWorld( btMultiBodyDynamicsWorld* btWorldPtr )
    {
        m_btWorldPtr = btWorldPtr;
    }

    void TBtTerrainGenWrapper::_initializeInternal()
    {
        // Check if the caller (TBtSimulation) set the btWorld reference
        if ( !m_btWorldPtr )
        {
            std::cout << "ERROR> bullet-sim object must pass a reference of the"
                      << " btWorld to this terrain generator" << std::endl;
            return;
        }

        // add all geometry resources into this element
        for ( size_t i = 0; i < m_btTerrainPrimitives.size(); i++ )
        {
            auto _btPrimitivePtr = m_btTerrainPrimitives[i];

            // Make sure the primitive has bullet resources already created
            if ( !_btPrimitivePtr->btPrimitiveBodyPtr )
                continue;

            // Define the default position and rotation
            TVec3 _position = { 0.0f, 0.0f, 100.0f + i * ( BT_TERRAIN_PATH_DEFAULT_TICKNESS + 1.0f ) };
            TMat3 _rotation; // identity

            // check if this primitive is comes from a static generator
            bool _isStatic = ( ( _btPrimitivePtr->tysocPrimitiveObj != NULL ) && 
                               ( _btPrimitivePtr->tysocPrimitiveObj->type == terrain::TERRAIN_TYPE_STATIC ) );

            // If so, grab the initial position/rotation given by the user
            if ( _isStatic )
            {
                // Grab the primitive that is being wrapped
                auto _tysocPrimitiveObj = _btPrimitivePtr->tysocPrimitiveObj;
                // extract the position @TODO: Change struct{x,y,z} by TVec3 (T_T)
                _position = { _tysocPrimitiveObj->pos.x,
                              _tysocPrimitiveObj->pos.y,
                              _tysocPrimitiveObj->pos.z };
                // extract the rotation matrix @TODO: Change float[9] by TMat3 (T_T)
                for ( size_t i = 0; i < 9; i++ )
                    _rotation.buff[i] = _tysocPrimitiveObj->rotmat[i];
            }

            // construct the world-transform for the bullet body
            btTransform _rbTransform;
            _rbTransform.setOrigin( utils::toBtVec3( _position ) );
            _rbTransform.setBasis( utils::toBtMat3( _rotation ) );

            // set the world-transform of the wrapped bullet body
            _btPrimitivePtr->btPrimitiveBodyPtr->setWorldTransform( _rbTransform );

            // add the bullet resource to the world
            m_btWorldPtr->addRigidBody( _btPrimitivePtr->btPrimitiveBodyPtr );
        }
    }

    void TBtTerrainGenWrapper::_resetInternal()
    {
        // @TODO: add reset functionality
    }

    void TBtTerrainGenWrapper::_preStepInternal()
    {
        _collectReusableFromGenerator();

        // update the properties of all objects (if they have a primitive as reference)
        for ( size_t i = 0; i < m_btTerrainPrimitives.size(); i++ )
        {
            if ( !m_btTerrainPrimitives[i]->tysocPrimitiveObj )
                return;

            if ( m_btTerrainPrimitives[i]->tysocPrimitiveObj->type == terrain::TERRAIN_TYPE_STATIC )
                return;

            _updateProperties( m_btTerrainPrimitives[i] );
        }
    }

    void TBtTerrainGenWrapper::_postStepInternal()
    {
        // @TODO: should contact checking be done here? (store contacts)
    }

    void TBtTerrainGenWrapper::_collectReusableFromGenerator()
    {
        auto _newPrimitivesQueue = m_terrainGenPtr->getJustCreated();

        while ( !_newPrimitivesQueue.empty() )
        {
            auto _newPrimitive = _newPrimitivesQueue.front();
            _newPrimitivesQueue.pop();

            _wrapReusablePrimitive( _newPrimitive );
        }

        // flush the creation queue to avoid double references everywhere
        m_terrainGenPtr->flushJustCreatedQueue();
    }

    void TBtTerrainGenWrapper::_collectStaticFromGenerator()
    {
        auto _fixedPrimitivesQueue = m_terrainGenPtr->getFixed();

        while ( !_fixedPrimitivesQueue.empty() )
        {
            auto _fixedPrimitive = _fixedPrimitivesQueue.front();
            _fixedPrimitivesQueue.pop();

            _wrapStaticPrimitive( _fixedPrimitive );
        }

        // flush the fixed queue to avoid double references (the wrapper now holds the ref.)
        m_terrainGenPtr->flushFixedQueue();
    }

    void TBtTerrainGenWrapper::_wrapReusablePrimitive( terrain::TTerrainPrimitive* primitivePtr )
    {
        // if the pool is empty, force to recycle the last object
        if ( m_btAvailablePrimitives.empty() )
        {
            auto _oldest = m_btWorkingPrimitives.front();
            if ( _oldest->tysocPrimitiveObj )
            {
                m_terrainGenPtr->recycle( _oldest->tysocPrimitiveObj );
                _oldest->tysocPrimitiveObj = NULL;
                m_btWorkingPrimitives.pop();
            }
            m_btAvailablePrimitives.push( _oldest );
        }

        // grab the oldest available object
        auto _btPrimitive = m_btAvailablePrimitives.front();
        m_btAvailablePrimitives.pop();

        // link an object from the working queue with the requested new primitive
        _btPrimitive->tysocPrimitiveObj = primitivePtr;

        // put it into the working queue
        m_btWorkingPrimitives.push( _btPrimitive );
    }

    void TBtTerrainGenWrapper::_wrapStaticPrimitive( terrain::TTerrainPrimitive* primitivePtr )
    {
        auto _btPrimitive = new TBtTerrainPrimitive();
        auto _btName = "tGen_" + name() + "_" + std::to_string( m_btTerrainPrimitives.size() );

        _btPrimitive->btPrimitiveName       = _btName;
        _btPrimitive->btPrimitiveType       = primitivePtr->geomType;
        _btPrimitive->btPrimitiveSize       = { primitivePtr->size.x,
                                                primitivePtr->size.y,
                                                primitivePtr->size.z };
        _btPrimitive->btPrimitiveFilename   = primitivePtr->filename;
        _btPrimitive->tysocPrimitiveObj     = primitivePtr;

        // create the btRigidBody to be used for this primitive in simulation
        _createBtBody( _btPrimitive );

        // and save the primitive in store for later usage
        m_btTerrainPrimitives.push_back( _btPrimitive );
    }

    void TBtTerrainGenWrapper::_createBtBody( TBtTerrainPrimitive* btTerrainPrimitivePtr )
    {
        // create the collision shape
        auto _collisionShapePtr = utils::createCollisionShape( btTerrainPrimitivePtr->btPrimitiveType,
                                                               btTerrainPrimitivePtr->btPrimitiveSize );

        // if something went wrong, the error messages were ...
        // sent during the creation of the collision
        if ( !_collisionShapePtr )
        {
            btTerrainPrimitivePtr->btPrimitiveBodyPtr = NULL;
            return;
        }

        // mass set to 0 such that this body is static @TODO: link ...
        // volume-density for throwables, and other similar requirements
        btScalar _rbMass     = 0.0f;
        btVector3 _rbInertia = { 0.0f, 0.0f, 0.0f };

        // create a default motion state to Identity, as it ...
        // will be set later during initialization
        btTransform _rbTransform;
        _rbTransform.setIdentity();

        auto _rbMotionState = new btDefaultMotionState( _rbTransform );
        
        // assemble the struct used for rigid body creation
        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo(
                                                            _rbMass,
                                                            _rbMotionState,
                                                            _collisionShapePtr,
                                                            _rbInertia );

        // construct the rigid body with the previous struct
        auto _rbBodyPtr = new btRigidBody( _rbConstructionInfo );

        // set friction
        _rbBodyPtr->setFriction( 1.0f );

        // make sure the object is going to be simulated by forcing activation
        _rbBodyPtr->forceActivationState( DISABLE_DEACTIVATION );

        // and save the bullet body reference for later
        btTerrainPrimitivePtr->btPrimitiveBodyPtr = _rbBodyPtr;
    }

    void TBtTerrainGenWrapper::_updateProperties( TBtTerrainPrimitive* btTerrainPritimivePtr )
    {
        auto _tysocPrimitiveObj = btTerrainPritimivePtr->tysocPrimitiveObj;
        auto _btBodyPtr = btTerrainPritimivePtr->btPrimitiveBodyPtr;

        if ( !_tysocPrimitiveObj || !_btBodyPtr )
            return;

        /* construct the position/orientation to our data types ...
           (T_T it should already be in our data types T_T) */
        TVec3 _position;
        TMat3 _rotation;

        // extract the position @TODO: Change struct{x,y,z} by TVec3 (T_T)
        _position = { _tysocPrimitiveObj->pos.x,
                      _tysocPrimitiveObj->pos.y,
                      _tysocPrimitiveObj->pos.z };

        // extract the rotation matrix @TODO: Change float[9] by TMat3 (T_T)
        for ( size_t i = 0; i < 9; i++ )
            _rotation.buff[i] = _tysocPrimitiveObj->rotmat[i];

        // construct the world transform appropriately
        btTransform _rbTransform;
        _rbTransform.setOrigin( utils::toBtVec3( _position ) );
        _rbTransform.setBasis( utils::toBtMat3( _rotation ) );

        // and set it to the bullet-body related to this primitive
        _btBodyPtr->setWorldTransform( _rbTransform );
    }

    extern "C" TTerrainGenWrapper* terrain_createFromAbstract( terrain::TITerrainGenerator* terrainGenPtr,
                                                               const std::string& workingDir )
    {
        return new TBtTerrainGenWrapper( terrainGenPtr, workingDir );
    }

    extern "C" TTerrainGenWrapper* terrain_createFromParams( const std::string& name,
                                                             const TGenericParams& params,
                                                             const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }

}}