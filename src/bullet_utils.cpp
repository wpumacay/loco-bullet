
#include <bullet_utils.h>

namespace tysoc {
namespace bullet {
namespace utils {


    btVector3 toBtVec3( const TVec3& vec )
    {
        return btVector3( vec.x, vec.y, vec.z );
    }

    TVec3 fromBtVec3( const btVector3& vec )
    {
        return TVec3( vec.x(), vec.y(), vec.z() );
    }

    btMatrix3x3 toBtMat3( const TMat3& mat )
    {
        btMatrix3x3 _res;

        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res[i][j] = mat.buff[i + 3 * j];

        return _res;
    }

    TMat3 fromBtMat3( const btMatrix3x3& mat )
    {
        TMat3 _res;

        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res.buff[i + 3 * j] = mat[i][j];

        return _res;
    }

    TMat4 fromBtTransform( const btTransform& tf )
    {
        auto _pos = fromBtVec3( tf.getOrigin() );
        auto _rot = fromBtMat3( tf.getBasis() );

        return TMat4::fromPositionAndRotation( _pos, _rot );
    }

    btTransform toBtTransform( const TMat4& mat )
    {
        auto _origin = toBtVec3( mat.getPosition() );
        auto _basis = toBtMat3( mat.getRotation() );

        return btTransform( _basis, _origin );
    }

    btCollisionShape* createCollisionShape( const std::string& type, const TVec3& size )
    {
        btCollisionShape* _collisionShapePtr = NULL;

        auto _modSize = size;

        if ( type == "box" )
        {
            _collisionShapePtr = new btBoxShape( btVector3( 0.5, 0.5, 0.5 ) );
        }
        else if ( type == "sphere" )
        {
            _collisionShapePtr = new btSphereShape( 1.0 );
        }
        else if ( type == "capsule" )
        {
            _modSize = { size.x, size.x, size.y };
            _collisionShapePtr = new btCapsuleShapeZ( 1.0, 1.0 );
        }
        else if ( type == "cylinder" )
        {
            _modSize = { size.x, size.x, size.y };
            _collisionShapePtr = new btCylinderShapeZ( btVector3( 1.0, 1.0, 0.5 ) );
        }
        else if ( type == "none" )
        {
            _collisionShapePtr = new btCompoundShape();
        }

        if ( !_collisionShapePtr )
            std::cout << "ERROR> could not create shape of type: " << type << std::endl;
        else if ( type != "plane" )
            _collisionShapePtr->setLocalScaling( utils::toBtVec3( _modSize ) );

        return _collisionShapePtr;
    }

    btScalar computeVolumeFromShape( btCollisionShape* colShape )
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

        return _volume;
    }

    size_t calculateNumOfLinksForMultibody( agent::TAgentKinTree* kinTreePtr )
    {
        size_t _numLinks = 0;

        std::stack< agent::TKinTreeBody* > _bodiesToConsider;
        _bodiesToConsider.push( kinTreePtr->getRootBody() );

        while ( !_bodiesToConsider.empty() )
        {
            auto _currentBodyPtr = _bodiesToConsider.top();
            _bodiesToConsider.pop();

            // check how many dofs does this body has. Ball and spherical are ...
            // considered as 1, as the setup(TypeOfConstraint) considers ball ...
            // as only one constraint
            auto _numJoints = _currentBodyPtr->childJoints.size();
            if ( _numJoints > 1 )
            {
                // MultiDof case, so we have to use dummies. We are using ...
                // 1 dummy per joint, and then connect to the actual collider ...
                // via a fixed constraint
                _numLinks += _numJoints;
            }
            else
            {
                // For any other case, this single joint (or none at all, which ...
                // makes it fixed) is taken into account by the links created ...
                // by the colliders
                _numLinks += 0;
            }

            // add as many links as colliders we have
            _numLinks += _currentBodyPtr->childCollisions.size();

            for ( size_t q = 0; q < _currentBodyPtr->childBodies.size(); q++ )
                _bodiesToConsider.push( _currentBodyPtr->childBodies[q] );
        }

        return _numLinks;
    }

    bool shouldBaseBeFixed( agent::TAgentKinTree* kinTreePtr )
    {
        auto _joints = kinTreePtr->getRootBody()->childJoints;

        if ( _joints.size() == 0 )
        {
            // the root body has no dofs, so just fix the base
            return true;
        }
        else if ( _joints.size() == 1 &&
                 _joints[0]->type == "free" )
        {
            // for free joints (full 6dofs) just leave the base free
            return false;
        }

        // all other cases have limited dofs, so the next body to the base ...
        // configures the required dofs
        return true;
    }

    void loadMesh( const std::string& filePath, TMeshObject& mesh )
    {
        auto _assimpScenePtr = aiImportFile( filePath.c_str(),
                                             aiProcessPreset_TargetRealtime_MaxQuality );

        if ( !_assimpScenePtr )
        {
            return;
        }

        // recursively copy the data from assimp to our data structure
        _processAssimpNode( _assimpScenePtr, _assimpScenePtr->mRootNode, mesh );

        // make sure we release the assimp resources
        aiReleaseImport( _assimpScenePtr );
    }

    void _processAssimpNode( const aiScene* assimpScenePtr,
                             aiNode* assimpNodePtr,
                             TMeshObject& mesh )
    {
        for ( size_t i = 0; i < assimpNodePtr->mNumMeshes; i++ )
        {
            aiMesh* _assimpMeshPtr = assimpScenePtr->mMeshes[ assimpNodePtr->mMeshes[i] ];
            _processAssimpMesh( _assimpMeshPtr, mesh );
        }

        for ( size_t i = 0; i < assimpNodePtr->mNumChildren; i++ )
        {
            _processAssimpNode( assimpScenePtr,
                                assimpNodePtr->mChildren[i], 
                                mesh );
        }
    }

    void _processAssimpMesh( aiMesh* assimpMeshPtr, TMeshObject& mesh )
    {
        std::vector< TVec3 > _vertices;

        for ( size_t i = 0; i < assimpMeshPtr->mNumVertices; i++ )
        {
            _vertices.push_back( TVec3( assimpMeshPtr->mVertices[i].x,
                                        assimpMeshPtr->mVertices[i].y,
                                        assimpMeshPtr->mVertices[i].z ) );
        }

        for ( size_t i = 0; i < assimpMeshPtr->mNumFaces; i++ )
        {
            aiFace _assimpFace = assimpMeshPtr->mFaces[i];
            // grab only in tris. we are assuming it comes this way
            // @TODO: Check this part as may have to support quads
            for ( size_t j = 0; j < _assimpFace.mNumIndices / 3; j++ )
            {
                auto _p1 = _vertices[ _assimpFace.mIndices[ 3 * j + 0 ] ];
                auto _p2 = _vertices[ _assimpFace.mIndices[ 3 * j + 1 ] ];
                auto _p3 = _vertices[ _assimpFace.mIndices[ 3 * j + 2 ] ];

                mesh.vertices.push_back( _p1 );
                mesh.vertices.push_back( _p2 );
                mesh.vertices.push_back( _p3 );
            }
        }
    }

    // Debug drawer

    TBtDebugDrawer::TBtDebugDrawer()
    {
        m_visualizerPtr = NULL;
        m_debugMode = btIDebugDraw::DBG_DrawWireframe | 
                      btIDebugDraw::DBG_DrawAabb |
                      btIDebugDraw::DBG_DrawFrames |
                      btIDebugDraw::DBG_DrawConstraints;
    }

    TBtDebugDrawer::~TBtDebugDrawer()
    {
        m_visualizerPtr = NULL;
    }

    void TBtDebugDrawer::setVisualizer( viz::TIVisualizer* visualizerPtr )
    {
        m_visualizerPtr = visualizerPtr;
    }

    void TBtDebugDrawer::drawLine( const btVector3& from, const btVector3& to, const btVector3& color )
    {
        if ( !m_visualizerPtr )
            return;

        m_visualizerPtr->drawLine( fromBtVec3( from ), fromBtVec3( to ), fromBtVec3( color ) );
    }

    void TBtDebugDrawer::drawContactPoint( const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color )
    {
        if ( !m_visualizerPtr )
            return;

        m_visualizerPtr->drawLine( fromBtVec3( PointOnB ), 
                                   fromBtVec3( PointOnB + normalOnB * distance ), 
                                   fromBtVec3( color ) );

        m_visualizerPtr->drawLine( fromBtVec3( PointOnB ), 
                                   fromBtVec3( PointOnB + normalOnB * 0.01 ), 
                                   { 0.2, 0.4, 0.5 } );
    }

    void TBtDebugDrawer::reportErrorWarning( const char* warningString )
    {
        std::cout << "WARNING> BtDebugDrawer says: " << warningString << std::endl;
    }

    void TBtDebugDrawer::draw3dText( const btVector3& location, const char* textString )
    {
        // do nothing
    }

    void TBtDebugDrawer::setDebugMode( int debugMode )
    {
        m_debugMode = debugMode;
    }

    int TBtDebugDrawer::getDebugMode() const
    {
        return m_debugMode;
    }

    bool TBtOverlapFilterCallback::needBroadphaseCollision( btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1 ) const 
    {
        // tysoc::log( "Testing broadphase collision checking" );

        bool _proxy0Affinity1 = ( proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask ) != 0;
        bool _proxy1Affinity0 = ( proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask ) != 0;

        return _proxy0Affinity1 || _proxy1Affinity0;
    }


}}}