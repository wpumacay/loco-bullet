
#pragma once

#include <tysocBulletCommon.h>
#include <utils/urdf/UrdfSchema.h>
#include <utils/rlsim/RLSimSchema.h>
#include <utils/loader/tysocMeshLoader.h>

namespace tysocBullet
{
    // forward declaration to allow composition
    struct TBulletKinTreeBody;
    struct TBulletKinTreeJoint;
    struct TBulletKinTreeVisual;

    struct TMaterial
    {
        std::string                             name;
        struct { float r; float g; float b; }   diffuse;
        struct { float r; float g; float b; }   specular;
        std::string                             texture;
    };

    struct TBulletKinTreeCollision
    {
        std::string                             type;
        struct { float x; float y; float z; }   size;
        std::string                             meshFileName;
        TBulletKinTreeBody*                     parentBodyPtr;// parent body
        float                                   relPosition[3];// relative transform to parent body
        float                                   relRotation[9];// relative transform to parent body
        float                                   position[3];// world transform - position
        float                                   rotation[9];// world transform - rotation
    };

    struct TBulletKinTreeVisual
    {
        std::string                             type;
        struct { float x; float y; float z; }   size;
        TMaterial                               material;
        std::string                             meshFileName;
        TBulletKinTreeBody*                     parentBodyPtr;// parent body
        float                                   relPosition[3];// relative transform to parent body
        float                                   relRotation[9];// relative transform to parent body
        float                                   position[3];// world transform - position
        float                                   rotation[9];// world transform - rotation
    };

    struct TBulletKinTreeInertia
    {
        float relPosition[3];// relative transform to parent body
        float relRotation[9];// relative transform to parent body
        float mass;
        float ixx;
        float ixy;
        float ixz;
        float iyy;
        float iyz;
        float izz;
    };

    /**
    * @brief: Body node for the kinematic tree
    */
    struct TBulletKinTreeBody
    {
        std::string                             name;
        TBulletKinTreeJoint*                    parentJointPtr;// parent joint
        float                                   position[3];// world transform - position
        float                                   rotation[9];// world transform - rotation
        TBulletKinTreeInertia                   inertia;// inertia (if defined. if not, calculated by engine)
        std::vector< TBulletKinTreeCollision* > childCollisions;
        std::vector< TBulletKinTreeVisual* >    childVisuals;// visuals attached to the body
        std::vector< TBulletKinTreeJoint* >     childJoints;// Connections to other bodies

        // Bullet specifics (concretion own stuff)
        btRigidBody*                            bulletBodyPtr;
        btTransform                             bulletLocalInertialFrame;
        btVector3                               bulletDiagonalizedInertia;
        btTransform                             bulletInitialBodyWorldTransform;
    };

    /**
    * @brief: Joint node for the kinematic tree
    */
    struct TBulletKinTreeJoint
    {
        std::string                             name;
        std::string                             type;
        float                                   axis[3];
        TBulletKinTreeBody*                     parentBodyPtr;// parent body
        float                                   position[3];// world transform - position
        float                                   rotation[9];// world transform - rotation
        float                                   relPosition[3];// relative transform - position
        float                                   relRotation[9];// relative transform - rotation
        TBulletKinTreeBody*                     childBodyPtr;

        // Bullet specifics (concretion own stuff)
        btTypedConstraint*                      bulletJointPtr;
        btTransform                             bulletParentToJointTransform;
    };


    class TBulletKinTree
    {

        private :

        std::vector< TBulletKinTreeBody* >  m_bodies;
        std::vector< TBulletKinTreeJoint* > m_joints;
        TBulletKinTreeBody*                 m_rootBodyPtr;

        std::map< std::string, TMaterial >  m_materials;

        btTransform                         m_startWorldTransform;

        // URDF processing functionality
        void _processUrdfModel( urdf::UrdfModel* urdfModelPtr );
        void _collectUrdfMaterials( urdf::UrdfModel* urdfModelPtr );
        TBulletKinTreeBody* _processUrdfLink( urdf::UrdfLink* urdfLinkPtr, TBulletKinTreeJoint* parentJointPtr );
        TBulletKinTreeJoint* _processUrdfJoint( urdf::UrdfJoint* urdfJointPtr, 
                                                urdf::UrdfLink* urdfChildLinkPtr,
                                                TBulletKinTreeBody* parentBodyPtr );

        void _processBodyInternal( TBulletKinTreeBody* kinTreeBodyPtr );
        void _processJointInternal( TBulletKinTreeJoint* kinTreeJointPtr );

        // steps for full compound rigid body creation
        btCollisionShape* _createCollisionShape( TBulletKinTreeCollision* kinTreeCollisionPtr );
        void _compensateForInertialFrame( TBulletKinTreeBody* kinTreeBodyPtr );


        void _updateBody( TBulletKinTreeBody* kinTreeBodyPtr );
        void _updateBodyInternal( TBulletKinTreeBody* kinTreeBodyPtr );

        void _updateJoint( TBulletKinTreeJoint* kinTreeJointPtr );
        void _updateJointInternal( TBulletKinTreeJoint* kinTreeJointPtr );

        void _updateVisual( TBulletKinTreeVisual* kinTreeVisualPtr );

        public :

        // create from urdf model
        TBulletKinTree( urdf::UrdfModel* urdfModelPtr );
        // create from rlsim model
        TBulletKinTree( rlsim::RLSimModel* rlsimModelPtr );

        ~TBulletKinTree();


        void update();
    };
}