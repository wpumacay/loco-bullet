
#include <tysocBulletKinTree.h>


namespace tysocBullet
{


    TBulletKinTree::TBulletKinTree( urdf::UrdfModel* urdfModelPtr )
    {
        m_rootBodyPtr = NULL;

        m_startWorldTransform.setIdentity();
        m_startWorldTransform.setOrigin( btVector3( 0.0, 0.0, 2.0 ) );

        _processUrdfModel( urdfModelPtr );
    }

    TBulletKinTree::~TBulletKinTree()
    {

    }

    void TBulletKinTree::_processUrdfModel( urdf::UrdfModel* urdfModelPtr )
    {
        // grab the first root link (there should be just one in the agent's urdf)
        auto _rootLinkNode = urdfModelPtr->m_rootLinks[0];
        
        // cache materials before full parsing
        _collectUrdfMaterials( urdfModelPtr );

        // start recursive processing
        m_rootBodyPtr = _processUrdfLink( _rootLinkNode, NULL );

        // start bodies-recursive internal processing
        m_rootBodyPtr->bulletInitialBodyWorldTransform = m_startWorldTransform;
        _processBodyInternal( m_rootBodyPtr );

        // start joints-recursive internal processing
        for ( size_t i = 0; i < m_rootBodyPtr->childJoints.size(); i++ )
        {
            _processJointInternal( m_rootBodyPtr->childJoints[i] );
        }
    }

    void TBulletKinTree::_collectUrdfMaterials( urdf::UrdfModel* urdfModelPtr )
    {
        auto _urdfMaterials = urdfModelPtr->m_materials;
        for ( size_t i = 0; i < _urdfMaterials.size(); i++ )
        {
            auto _urdfMaterial  = *_urdfMaterials.getAtIndex( i );

            TMaterial _material;
            _material.name      = _urdfMaterial->m_name;
            _material.texture   = _urdfMaterial->m_textureFilename;
            _material.diffuse   = { _urdfMaterial->m_matColor.m_rgbaColor.x(),
                                    _urdfMaterial->m_matColor.m_rgbaColor.y(),
                                    _urdfMaterial->m_matColor.m_rgbaColor.z() };

            _material.specular  = { _urdfMaterial->m_matColor.m_specularColor.x(),
                                    _urdfMaterial->m_matColor.m_specularColor.y(),
                                    _urdfMaterial->m_matColor.m_specularColor.z() };

            m_materials[ _material.name ] = _material;
        }
    }

    TBulletKinTreeBody* TBulletKinTree::_processUrdfLink( urdf::UrdfLink* urdfLinkPtr, 
                                                          TBulletKinTreeJoint* parentJointPtr )
    {
        auto _kinTreeLink = new TBulletKinTreeBody();
        m_bodies.push_back( _kinTreeLink );

        // set parent joint
        _kinTreeLink->parentJointPtr = parentJointPtr;

        // collect name
        _kinTreeLink->name = urdfLinkPtr->m_name;

        // collect inertia information
        getVec3Array( urdfLinkPtr->m_inertia.m_linkLocalFrame.getOrigin(), _kinTreeLink->inertia.relPosition );
        getMat3Array( urdfLinkPtr->m_inertia.m_linkLocalFrame.getBasis(), _kinTreeLink->inertia.relRotation );
        _kinTreeLink->inertia.mass  = urdfLinkPtr->m_inertia.m_mass;
        _kinTreeLink->inertia.ixx   = urdfLinkPtr->m_inertia.m_ixx;
        _kinTreeLink->inertia.ixy   = urdfLinkPtr->m_inertia.m_ixy;
        _kinTreeLink->inertia.ixz   = urdfLinkPtr->m_inertia.m_ixz;
        _kinTreeLink->inertia.iyy   = urdfLinkPtr->m_inertia.m_iyy;
        _kinTreeLink->inertia.iyz   = urdfLinkPtr->m_inertia.m_iyz;
        _kinTreeLink->inertia.izz   = urdfLinkPtr->m_inertia.m_izz;

        // collect collision information
        for ( size_t i = 0; i < urdfLinkPtr->m_collisionArray.size(); i++ )
        {
            auto _kinTreeCollision = new TBulletKinTreeCollision();

            auto _urdfCollision     = urdfLinkPtr->m_collisionArray[i];
            auto _urdfGeometry      = _urdfCollision.m_geometry;

            // grab local transform
            getVec3Array( _urdfCollision.m_linkLocalFrame.getOrigin(), _kinTreeCollision->relPosition );
            getMat3Array( _urdfCollision.m_linkLocalFrame.getBasis(), _kinTreeCollision->relRotation );

            // set reference to parent body
            _kinTreeCollision->parentBodyPtr = _kinTreeLink;

            // grab geometry type
            if ( _urdfGeometry.m_type == urdf::URDF_GEOM_BOX )
            {
                _kinTreeCollision->type = "box";
                _kinTreeCollision->size.x = _urdfGeometry.m_boxSize.x();
                _kinTreeCollision->size.y = _urdfGeometry.m_boxSize.y();
                _kinTreeCollision->size.z = _urdfGeometry.m_boxSize.z();
            }
            else if ( _urdfGeometry.m_type == urdf::URDF_GEOM_SPHERE )
            {
                _kinTreeCollision->type = "sphere";
                _kinTreeCollision->size.x = _urdfGeometry.m_sphereRadius;
            }
            else if ( _urdfGeometry.m_type == urdf::URDF_GEOM_CAPSULE )
            {
                _kinTreeCollision->type = "capsule";
                _kinTreeCollision->size.x = _urdfGeometry.m_capsuleRadius;
                _kinTreeCollision->size.y = _urdfGeometry.m_capsuleHeight;
                // @TODO: Add check for fromto
            }
            else if ( _urdfGeometry.m_type == urdf::URDF_GEOM_CYLINDER )
            {
                _kinTreeCollision->type = "cylinder";
                _kinTreeCollision->size.x = _urdfGeometry.m_capsuleRadius;
                _kinTreeCollision->size.y = _urdfGeometry.m_capsuleHeight;
                // @TODO: Add check for fromto
            }
            else if ( _urdfGeometry.m_type == urdf::URDF_GEOM_MESH )
            {
                _kinTreeCollision->type = "mesh";
                _kinTreeCollision->size.x = _urdfGeometry.m_meshScale.x();
                _kinTreeCollision->size.y = _urdfGeometry.m_meshScale.y();
                _kinTreeCollision->size.z = _urdfGeometry.m_meshScale.z();
                _kinTreeCollision->meshFileName = _urdfGeometry.m_meshFileName;
            }
            else
            {
                _kinTreeCollision->type = "undefined";
                std::cout << "WARNING> collision-geometry of type: " 
                          << _urdfGeometry.m_type 
                          << " not supported" << std::endl;
            }

            // add this collision as a child of this current link
            _kinTreeLink->childCollisions.push_back( _kinTreeCollision );
        }

        // collect visual information
        for ( size_t i = 0; i < urdfLinkPtr->m_visualArray.size(); i++ )
        {
            auto _kinTreeVisual = new TBulletKinTreeVisual();

            auto _urdfVisual        = urdfLinkPtr->m_visualArray[i];
            auto _urdfMaterialName  = _urdfVisual.m_materialName;
            auto _urdfGeometry      = _urdfVisual.m_geometry;

            // grab local transform
            getVec3Array( _urdfVisual.m_linkLocalFrame.getOrigin(), _kinTreeVisual->relPosition );
            getMat3Array( _urdfVisual.m_linkLocalFrame.getBasis(), _kinTreeVisual->relRotation );

            // set reference to parent body
            _kinTreeVisual->parentBodyPtr = _kinTreeLink;

            // grab geometry type
            if ( _urdfGeometry.m_type == urdf::URDF_GEOM_BOX )
            {
                _kinTreeVisual->type = "box";
                _kinTreeVisual->size.x = _urdfGeometry.m_boxSize.x();
                _kinTreeVisual->size.y = _urdfGeometry.m_boxSize.y();
                _kinTreeVisual->size.z = _urdfGeometry.m_boxSize.z();
            }
            else if ( _urdfGeometry.m_type == urdf::URDF_GEOM_SPHERE )
            {
                _kinTreeVisual->type = "sphere";
                _kinTreeVisual->size.x = _urdfGeometry.m_sphereRadius;
            }
            else if ( _urdfGeometry.m_type == urdf::URDF_GEOM_CAPSULE )
            {
                _kinTreeVisual->type = "capsule";
                _kinTreeVisual->size.x = _urdfGeometry.m_capsuleRadius;
                _kinTreeVisual->size.y = _urdfGeometry.m_capsuleHeight;
                // @TODO: Add check for fromto
            }
            else if ( _urdfGeometry.m_type == urdf::URDF_GEOM_CYLINDER )
            {
                _kinTreeVisual->type = "cylinder";
                _kinTreeVisual->size.x = _urdfGeometry.m_capsuleRadius;
                _kinTreeVisual->size.y = _urdfGeometry.m_capsuleHeight;
                // @TODO: Add check for fromto
            }
            else if ( _urdfGeometry.m_type == urdf::URDF_GEOM_MESH )
            {
                _kinTreeVisual->type = "mesh";
                _kinTreeVisual->size.x = _urdfGeometry.m_meshScale.x();
                _kinTreeVisual->size.y = _urdfGeometry.m_meshScale.y();
                _kinTreeVisual->size.z = _urdfGeometry.m_meshScale.z();
                _kinTreeVisual->meshFileName = _urdfGeometry.m_meshFileName;
            }
            else
            {
                _kinTreeVisual->type = "undefined";
                std::cout << "WARNING> visual-geometry of type: " 
                          << _urdfGeometry.m_type 
                          << " not supported" << std::endl;
            }

            _kinTreeVisual->material = m_materials[ _urdfMaterialName ];

            // add this visual as a child of this current link
            _kinTreeLink->childVisuals.push_back( _kinTreeVisual );
        }

        // collect joints (the parser makes the following arrays with the same dimension)
        for ( size_t i = 0; i < urdfLinkPtr->m_childJoints.size(); i++ )
        {
            auto _kinTreeJoint = _processUrdfJoint( urdfLinkPtr->m_childJoints[i],
                                                    urdfLinkPtr->m_childLinks[i],
                                                    _kinTreeLink );
            _kinTreeLink->childJoints.push_back( _kinTreeJoint );
        }


        return _kinTreeLink;
    }

    TBulletKinTreeJoint* TBulletKinTree::_processUrdfJoint( urdf::UrdfJoint* urdfJointPtr, 
                                                            urdf::UrdfLink* urdfChildLinkPtr,
                                                            TBulletKinTreeBody* parentBodyPtr )
    {
        auto _kinTreeJoint = new TBulletKinTreeJoint();
        m_joints.push_back( _kinTreeJoint );

        // set parent body
        _kinTreeJoint->parentBodyPtr = parentBodyPtr;

        // collect joint name and type
        _kinTreeJoint->name = urdfJointPtr->m_name;
        if ( urdfJointPtr->m_type == urdf::URDFRevoluteJoint )
        {
            _kinTreeJoint->type = "revolute";
        }
        else if ( urdfJointPtr->m_type == urdf::URDFPrismaticJoint )
        {
            _kinTreeJoint->type = "prismatic";
        }
        else if ( urdfJointPtr->m_type == urdf::URDFContinuousJoint )
        {
            _kinTreeJoint->type = "continuous";
        }
        else if ( urdfJointPtr->m_type == urdf::URDFFloatingJoint )
        {
            _kinTreeJoint->type = "floating";
        }
        else if ( urdfJointPtr->m_type == urdf::URDFPlanarJoint )
        {
            _kinTreeJoint->type = "planar";
        }
        else if ( urdfJointPtr->m_type == urdf::URDFFixedJoint )
        {
            _kinTreeJoint->type = "fixed";
        }
        else if ( urdfJointPtr->m_type == urdf::URDFSphericalJoint )
        {
            _kinTreeJoint->type = "spherical";
        }
        else
        {
            _kinTreeJoint->type = "undefined";
            std::cout << "WARNING> urdf joint type: " 
                      << urdfJointPtr->m_type 
                      << " not supported" << std::endl;
        }

        // collect local axis
        getVec3Array( urdfJointPtr->m_localJointAxis, _kinTreeJoint->axis );

        // collect local transformation from parent body to this joint's frame
        getVec3Array( urdfJointPtr->m_parentLinkToJointTransform.getOrigin(), _kinTreeJoint->relPosition );
        getMat3Array( urdfJointPtr->m_parentLinkToJointTransform.getBasis(), _kinTreeJoint->relRotation );

        auto _kinTreeLink = _processUrdfLink( urdfChildLinkPtr, _kinTreeJoint );
        _kinTreeJoint->childBodyPtr = _kinTreeLink;

        return _kinTreeJoint;
    }

    btCollisionShape* TBulletKinTree::_createCollisionShape( TBulletKinTreeCollision* kinTreeCollisionPtr )
    {
        btCollisionShape* _rbCollisionShape = NULL;

        // @CHECK: It seems that I've already made object creation in many places. Should ...
        // move this to a factory instead and reuse it as needed
        // Create the collision shape
        if ( kinTreeCollisionPtr->type == "box" )
        {
            btVector3 _halfExtents( 0.5f * kinTreeCollisionPtr->size.x,
                                    0.5f * kinTreeCollisionPtr->size.y,
                                    0.5f * kinTreeCollisionPtr->size.z );
            _rbCollisionShape = new btBoxShape( _halfExtents );
        }
        else if ( kinTreeCollisionPtr->type == "sphere" )
        {
            _rbCollisionShape = new btSphereShape( kinTreeCollisionPtr->size.x );
        }
        else if ( kinTreeCollisionPtr->type == "capsule" )
        {
            _rbCollisionShape = new btCapsuleShapeZ( kinTreeCollisionPtr->size.x,   // radius
                                                     kinTreeCollisionPtr->size.y );  // height
        }
        else if ( kinTreeCollisionPtr->type == "cylinder" )
        {
            btVector3 _halfExtents( kinTreeCollisionPtr->size.x,          // radius
                                    1.0f,                               // not used
                                    0.5f * kinTreeCollisionPtr->size.y ); // half height
            _rbCollisionShape = new btCylinderShapeZ( _halfExtents );
        }
        else if ( kinTreeCollisionPtr->type == "mesh" )
        {
            loader::TMeshObject _collisionMesh;
            // load the mesh file @CHANGE-> meshfilename point to?
            loader::loadMesh( kinTreeCollisionPtr->meshFileName, _collisionMesh );
            // create the mesh collision shape
            _rbCollisionShape = new btConvexHullShape( (btScalar*) _collisionMesh.vertices.data(),
                                                       _collisionMesh.vertices.size(), 
                                                       sizeof( loader::TMeshVertex ) );
            reinterpret_cast< btConvexHullShape* >( _rbCollisionShape )->optimizeConvexHull();
            reinterpret_cast< btConvexHullShape* >( _rbCollisionShape )->initializePolyhedralFeatures();
            reinterpret_cast< btConvexHullShape* >( _rbCollisionShape )->setMargin( 0.001 );
            reinterpret_cast< btConvexHullShape* >( _rbCollisionShape )->recalcLocalAabb();
        }
        else
        {
            std::cout << "WARNING> collision of type: " << kinTreeCollisionPtr->type
                      << " not supported. Creating a sphere instead" << std::endl;
        }

        return _rbCollisionShape;
    }

    void TBulletKinTree::_compensateForInertialFrame( TBulletKinTreeBody* kinTreeBodyPtr )
    {
        TBulletKinTreeInertia _kinTreeInertia = kinTreeBodyPtr->inertia;

        // @CHECK: Add error checking (for now we are assuming no messed up urdf filed are passed)
        btScalar _rbMass = _kinTreeInertia.mass;
        // inertia as diagonal components (should make transform if compensation is needed)
        btVector3 _rbInertia;
        _rbInertia.setValue( 0.0, 0.0, 0.0 );
        // inertial frame (identity if diagonal inertia, a compensation transform if not)
        btTransform _rbLocalInertialFrame;
        // inertial basis in which the inertia matrix is diagonal
        btMatrix3x3 _rbInertiaBasis;
        {
            // If the inertia matrix is diagonal, then no compensation needed
            if ( _kinTreeInertia.ixy == 0.0 &&
                 _kinTreeInertia.ixz == 0.0 &&
                 _kinTreeInertia.iyz == 0.0 )
            {
                _rbInertia.setX( _kinTreeInertia.ixx );
                _rbInertia.setY( _kinTreeInertia.iyy );
                _rbInertia.setZ( _kinTreeInertia.izz );

                _rbInertiaBasis.setIdentity();
            }
            else
            {
                btMatrix3x3 _inertiaTensor( _kinTreeInertia.ixx, _kinTreeInertia.ixy, _kinTreeInertia.ixz,
                                            _kinTreeInertia.ixy, _kinTreeInertia.iyy, _kinTreeInertia.iyz,
                                            _kinTreeInertia.ixz, _kinTreeInertia.iyz, _kinTreeInertia.izz );

                // diagonalize to get the basis where the inertia matrix is diagonal
                _inertiaTensor.diagonalize( _rbInertiaBasis, 1.0e-6, 30 );

                // grab the now diagonal inertia tensor
                _rbInertia.setX( _inertiaTensor[0][0] );
                _rbInertia.setY( _inertiaTensor[1][1] );
                _rbInertia.setZ( _inertiaTensor[2][2] );
            }

            btVector3 _origin;
            createBtVec3( _kinTreeInertia.relPosition, _origin );

            btMatrix3x3 _basis;
            createBtMat3( _kinTreeInertia.relRotation, _basis );

            _rbLocalInertialFrame.setOrigin( _origin );
            _rbLocalInertialFrame.setBasis( _basis * _rbInertiaBasis );
        }

        // store the inertial information
        kinTreeBodyPtr->bulletLocalInertialFrame    = _rbLocalInertialFrame;
        kinTreeBodyPtr->bulletDiagonalizedInertia   = _rbInertia;
    }

    void TBulletKinTree::_processBodyInternal( TBulletKinTreeBody* kinTreeBodyPtr )
    {
        kinTreeBodyPtr->bulletBodyPtr = NULL;

        // do the inertia trick @CHECK
        _compensateForInertialFrame( kinTreeBodyPtr );

        // create the compound that will hold all collisionshapes in this body
        auto _rbCompoundShape = new btCompoundShape();

        for ( size_t i = 0; i < kinTreeBodyPtr->childCollisions.size(); i++ )
        {
            auto _rbCollisionShape = _createCollisionShape( kinTreeBodyPtr->childCollisions[i] );

            if ( !_rbCollisionShape )
            {
                continue;
            }

            // grab the collision offset respect to the center of mass
            btTransform _collisionToCOM;
            createBtVec3( kinTreeBodyPtr->childCollisions[i]->relPosition, _collisionToCOM.getOrigin() );
            createBtMat3( kinTreeBodyPtr->childCollisions[i]->relRotation, _collisionToCOM.getBasis() );

            _rbCompoundShape->addChildShape( kinTreeBodyPtr->bulletLocalInertialFrame.inverse() * _collisionToCOM,
                                             _rbCollisionShape );
        }

        // Create a default motion state
        auto _rbMotionState = new btDefaultMotionState();

        // Compute inertia props for bullet in case something went wrong
        btScalar _rbMass        = kinTreeBodyPtr->inertia.mass;
        btVector3 _rbInertia    = kinTreeBodyPtr->bulletDiagonalizedInertia;

        if ( _rbInertia.isZero() )
        {
            if ( _rbMass != 0.0 )
            {
                _rbCompoundShape->calculateLocalInertia( _rbMass,
                                                         _rbInertia );
            }
        }

        btRigidBody::btRigidBodyConstructionInfo _rbConstructionInfo( _rbMass,
                                                                      _rbMotionState,
                                                                      _rbCompoundShape,
                                                                      _rbInertia );

        btRigidBody* _rbody = new btRigidBody( _rbConstructionInfo );
        _rbody->setRestitution( 0.9f );// @TODO: Link from urdf data - contact info
        _rbody->setFriction( 0.5f );// @TODO: Link from urdf data - contact info            

        kinTreeBodyPtr->bulletBodyPtr = _rbody;

        // DO THE THING xD : process the joints' child bodies
        for ( size_t i = 0; i < kinTreeBodyPtr->childJoints.size(); i++ )
        {
            _processBodyInternal( kinTreeBodyPtr->childJoints[i]->childBodyPtr );
        }
    }

    void TBulletKinTree::_processJointInternal( TBulletKinTreeJoint* kinTreeJointPtr )
    {
        kinTreeJointPtr->bulletJointPtr = NULL;

        TBulletKinTreeBody* _parentBodyPtr  = kinTreeJointPtr->parentBodyPtr;
        TBulletKinTreeBody* _childBodyPtr   = kinTreeJointPtr->childBodyPtr;

        // build "parent to joint" btTransform from the parsed data
        createBtVec3( kinTreeJointPtr->relPosition,
                      kinTreeJointPtr->bulletParentToJointTransform.getOrigin() );

        createBtMat3( kinTreeJointPtr->relRotation,
                      kinTreeJointPtr->bulletParentToJointTransform.getBasis() );

        // Remember, we are dealing with URDFs here, so some representations are specific
        // In the case of the rbAFrame and rbBFrame, those are quite general, but because ...
        // of the URDF representation, this is a bit more constraint and specific.
        // Check this link for more info: https://wiki.ros.org/urdf/XML/joint
        // For jointFrameInA : 
        //          
        //          
        // For jointFrameInB :  The frame of this joint respect to the child body is ...
        //                      located 
        //          
        btTransform _jointFrameInA;
        btTransform _jointFrameInB;

        _jointFrameInA = _parentBodyPtr->bulletLocalInertialFrame.inverse() *
                         kinTreeJointPtr->bulletParentToJointTransform;

        _jointFrameInB = _childBodyPtr->bulletLocalInertialFrame.inverse();

        btTypedConstraint* _bulletJointPtr = NULL;
        if ( kinTreeJointPtr->type == "revolute" ||
             kinTreeJointPtr->type == "continuous" )
        {
            // @CHECK - Kintree impl
            btVector3 _axisInA;
            btVector3 _axisInB;
            createBtVec3( kinTreeJointPtr->axis, _axisInA );
            _axisInA = _jointFrameInA * _axisInA;
            createBtVec3( kinTreeJointPtr->axis, _axisInB );
            _bulletJointPtr = new btHingeConstraint( *_parentBodyPtr->bulletBodyPtr,
                                                     *_childBodyPtr->bulletBodyPtr,
                                                     _jointFrameInA.getOrigin(),
                                                     -_jointFrameInB.getOrigin(),
                                                     _axisInA, 
                                                     _axisInB,
                                                     true );
        }
        else if ( kinTreeJointPtr->type == "fixed" )
        {
            // @CHECK - Kintree impl
            _bulletJointPtr = new btFixedConstraint( *_parentBodyPtr->bulletBodyPtr,
                                                     *_childBodyPtr->bulletBodyPtr,
                                                     _jointFrameInA,
                                                     _jointFrameInB );
        }
        else
        {
            std::cout << "WARNING> bullet-joint with type: " 
                      << kinTreeJointPtr->type << " not supported" << std::endl;
        }

        kinTreeJointPtr->bulletJointPtr = _bulletJointPtr;

        // DO THE THING xD : process the bodies' child joints
        for ( size_t i = 0; i < _childBodyPtr->childJoints.size(); i++ )
        {
            _processJointInternal( _childBodyPtr->childJoints[i] );
        }
    }

    void TBulletKinTree::update()
    {
        for ( size_t i = 0; i < m_bodies.size(); i++ )
        {
            _updateBody( m_bodies[i] );
        }

        for ( size_t i = 0; i < m_joints.size(); i++ )
        {
            _updateJoint( m_joints[i] );
        }
    }

    void TBulletKinTree::_updateBody( TBulletKinTreeBody* kinTreeBodyPtr )
    {
        _updateBodyInternal( kinTreeBodyPtr );
        for ( size_t i = 0; i < kinTreeBodyPtr->childVisuals.size(); i++ )
        {
            _updateVisual( kinTreeBodyPtr->childVisuals[i] );
        }
    }

    void TBulletKinTree::_updateBodyInternal( TBulletKinTreeBody* kinTreeBodyPtr )
    {
        // update world transform for each body
        btTransform _bodyTransform = kinTreeBodyPtr->bulletBodyPtr->getCenterOfMassTransform();
        getVec3Array( _bodyTransform.getOrigin(), kinTreeBodyPtr->position );
        getMat3Array( _bodyTransform.getBasis(), kinTreeBodyPtr->rotation );
    }

    void TBulletKinTree::_updateJoint( TBulletKinTreeJoint* kinTreeJointPtr )
    {
        // update world transform for each joint
        btTransform _parentWorldTransform;
        createBtVec3( kinTreeJointPtr->parentBodyPtr->position, _parentWorldTransform.getOrigin() );
        createBtMat3( kinTreeJointPtr->parentBodyPtr->rotation, _parentWorldTransform.getBasis() );

        btTransform _jointWorldTransform = _parentWorldTransform * kinTreeJointPtr->bulletParentToJointTransform;
        getVec3Array( _jointWorldTransform.getOrigin(), kinTreeJointPtr->position );
        getMat3Array( _jointWorldTransform.getBasis(), kinTreeJointPtr->rotation );

        _updateJointInternal( kinTreeJointPtr );
    }

    void TBulletKinTree::_updateJointInternal( TBulletKinTreeJoint* kinTreeJointPtr )
    {

    }

    void TBulletKinTree::_updateVisual( TBulletKinTreeVisual* kinTreeVisualPtr )
    {
        auto _parentPos = kinTreeVisualPtr->parentBodyPtr->position;
        auto _parentRot = kinTreeVisualPtr->parentBodyPtr->rotation;

        auto _relPos = kinTreeVisualPtr->relPosition;
        auto _relRot = kinTreeVisualPtr->relRotation;

        auto& _position = kinTreeVisualPtr->position;
        auto& _rotation = kinTreeVisualPtr->rotation;

        for ( size_t i = 0; i < 3; i++ )
        {
            for ( size_t j = 0; j < 3; j++ )
            {
                _rotation[ j + i * 3 ] = 0;
                for ( size_t k = 0; k < 3; k++ )
                {
                    _rotation[ j + i * 3 ] += _parentRot[ j + k * 3 ] * _relRot[ k + i * 3 ];
                }
            }
        }

        _position[0] = _parentPos[0] + _parentRot[0] * _relPos[0]
                                     + _parentRot[3] * _relPos[1]
                                     + _parentRot[6] * _relPos[2];
        _position[1] = _parentPos[1] + _parentRot[1] * _relPos[0]
                                     + _parentRot[4] * _relPos[1]
                                     + _parentRot[7] * _relPos[2];
        _position[2] = _parentPos[2] + _parentRot[2] * _relPos[0]
                                     + _parentRot[5] * _relPos[1]
                                     + _parentRot[8] * _relPos[2];
    }


}