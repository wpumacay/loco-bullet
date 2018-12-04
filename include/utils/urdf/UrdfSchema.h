
/* Adapted from bullet examples. Check type-definitions in files located in Importers/ImportURDFDemo/ */

#pragma once

#include <string>
#include <LinearMath/btTransform.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btHashMap.h>

namespace urdf
{
    /*************************************************************
    *   NATERIAL AND INERTIA INFORMATION
    *************************************************************/

    struct UrdfMaterialColor
    {
        btVector4 m_rgbaColor;
        btVector3 m_specularColor;

        UrdfMaterialColor()
        {
            m_rgbaColor     = btVector4( 0.8, 0.8, 0.8, 1.0 );
            m_specularColor = btVector3( 0.4, 0.4, 0.4 );
        }
    };

    struct UrdfMaterial
    {
        std::string         m_name;
        std::string         m_textureFilename;
        UrdfMaterialColor   m_matColor;

        UrdfMaterial(){}
    };

    struct UrdfInertia
    {
        btTransform     m_linkLocalFrame;
        bool            m_hasLinkLocalFrame;
        double          m_mass;
        double          m_ixx; 
        double          m_ixy;
        double          m_ixz;
        double          m_iyy;
        double          m_iyz;
        double          m_izz;

        UrdfInertia()
        {
            m_hasLinkLocalFrame = false;
            m_linkLocalFrame.setIdentity();
            m_mass = 0.f;
            m_ixx = m_ixy = m_ixz = m_iyy = m_iyz = m_izz = 0.f;
        }
    };

    /*************************************************************
    *   GEOEMTRY TYPES INFORMATION
    *************************************************************/
    enum UrdfGeomTypes
    {
        URDF_GEOM_SPHERE = 2,
        URDF_GEOM_BOX,
        URDF_GEOM_CYLINDER,
        URDF_GEOM_MESH,
        URDF_GEOM_PLANE,
        URDF_GEOM_CAPSULE,  //non-standard URDF
        URDF_GEOM_CDF,      //signed-distance-field, non-standard URDF
        URDF_GEOM_UNKNOWN,
    };

    struct UrdfGeometry
    {
        UrdfGeomTypes m_type;

        double m_sphereRadius;

        btVector3 m_boxSize;

        double m_capsuleRadius;
        double m_capsuleHeight;
        int m_hasFromTo;
        btVector3 m_capsuleFrom;
        btVector3 m_capsuleTo;

        btVector3 m_planeNormal;

        enum
        {
            FILE_STL = 1,
            FILE_COLLADA = 2,
            FILE_OBJ = 3,
            FILE_CDF = 4,
            MEMORY_VERTICES = 5,

        };
        int m_meshFileType;
        std::string m_meshFileName;
        btVector3 m_meshScale;

        UrdfMaterial m_localMaterial;
        bool m_hasLocalMaterial;

        UrdfGeometry()        
        {
            m_type              = URDF_GEOM_UNKNOWN;
            m_sphereRadius      = 1;
            m_boxSize           = btVector3( 1, 1, 1 );
            m_capsuleRadius     = 1;
            m_capsuleHeight     = 1;
            m_hasFromTo         = 0;
            m_capsuleFrom       = btVector3( 0, 1, 0 );
            m_capsuleTo         = btVector3( 1, 0, 0 );
            m_planeNormal       = btVector3( 0, 0, 1 );
            m_meshFileType      = 0;
            m_meshScale         = btVector3( 1, 1, 1 );
            m_hasLocalMaterial  = false;
        }
    };

    struct UrdfShape
    {
        std::string     m_sourceFileLocation;
        btTransform     m_linkLocalFrame;
        UrdfGeometry    m_geometry;
        std::string     m_name;
    };

    struct UrdfVisual : UrdfShape
    {
        std::string m_materialName;
    };

    struct UrdfCollision : UrdfShape
    {
        int m_flags;
        int m_collisionGroup;
        int m_collisionMask;

        UrdfCollision()
        {
            m_flags = 0;
        }
    };

    /*************************************************************
    *   JOINT TYPES INFORMATION
    *************************************************************/
    enum UrdfJointTypes
    {
        URDFRevoluteJoint = 1,
        URDFPrismaticJoint,
        URDFContinuousJoint,
        URDFFloatingJoint,
        URDFPlanarJoint,
        URDFFixedJoint,
        URDFSphericalJoint,

    };

    enum URDF_LinkContactFlags
    {
        URDF_CONTACT_HAS_LATERAL_FRICTION = 1,
        URDF_CONTACT_HAS_INERTIA_SCALING = 2,
        URDF_CONTACT_HAS_CONTACT_CFM = 4,
        URDF_CONTACT_HAS_CONTACT_ERP = 8,
        URDF_CONTACT_HAS_STIFFNESS_DAMPING = 16,
        URDF_CONTACT_HAS_ROLLING_FRICTION = 32,
        URDF_CONTACT_HAS_SPINNING_FRICTION = 64,
        URDF_CONTACT_HAS_RESTITUTION = 128,
        URDF_CONTACT_HAS_FRICTION_ANCHOR = 256,

    };

    enum UrdfCollisionFlags
    {
        URDF_FORCE_CONCAVE_TRIMESH = 1,
        URDF_HAS_COLLISION_GROUP = 2,
        URDF_HAS_COLLISION_MASK = 4,
    };

    struct URDFLinkContactInfo
    {
        btScalar m_lateralFriction;
        btScalar m_rollingFriction;
        btScalar m_spinningFriction;
        btScalar m_restitution;
        btScalar m_inertiaScaling;
        btScalar m_contactCfm;
        btScalar m_contactErp;
        btScalar m_contactStiffness;
        btScalar m_contactDamping;

        int m_flags;

        URDFLinkContactInfo()
        {
            m_lateralFriction   = 0.5;
            m_rollingFriction   = 0;
            m_spinningFriction  = 0;
            m_restitution       = 0;
            m_inertiaScaling    = 1;
            m_contactCfm        = 0;
            m_contactErp        = 0;
            m_contactStiffness  = 1e4;
            m_contactDamping    = 1;

            m_flags = URDF_CONTACT_HAS_LATERAL_FRICTION;
        }
    };

    /**********************************************************************
    *   JOINT AND LINKS DATA HOLDERS
    **********************************************************************/

    struct UrdfJoint;

    struct UrdfLink
    {
        std::string                         m_name;
        UrdfInertia                         m_inertia;
        btTransform                         m_linkTransformInWorld;
        btAlignedObjectArray<UrdfVisual>    m_visualArray;
        btAlignedObjectArray<UrdfCollision> m_collisionArray;
        UrdfLink*                           m_parentLink;
        UrdfJoint*                          m_parentJoint;
        btAlignedObjectArray<UrdfJoint*>    m_childJoints;
        btAlignedObjectArray<UrdfLink*>     m_childLinks;
        int                                 m_linkIndex;
        URDFLinkContactInfo                 m_contactInfo;

        UrdfLink()
        {
            m_parentLink    = 0;
            m_parentJoint   = 0;
            m_linkIndex     = -2;
        }
    };

    struct UrdfJoint
    {
        std::string     m_name;
        UrdfJointTypes  m_type;
        btTransform     m_parentLinkToJointTransform;
        std::string     m_parentLinkName;
        std::string     m_childLinkName;
        btVector3       m_localJointAxis;

        double m_lowerLimit;
        double m_upperLimit;

        double m_effortLimit;
        double m_velocityLimit;

        double m_jointDamping;
        double m_jointFriction;

        UrdfJoint()
        {
            m_lowerLimit    = 0;
            m_upperLimit    = -1;
            m_effortLimit   = 0;
            m_velocityLimit = 0;
            m_jointDamping  = 0;
            m_jointFriction = 0;
        }
    };


    /**********************************************************************
    *   URDF MODEL DATA STRUCTURE
    **********************************************************************/

    struct UrdfModel
    {
        std::string                             m_name;
        std::string                             m_sourceFile;
        btTransform                             m_rootTransformInWorld;
        btHashMap<btHashString, UrdfMaterial*>  m_materials;
        btHashMap<btHashString, UrdfLink*>      m_links;
        btHashMap<btHashString, UrdfJoint*>     m_joints;
        btAlignedObjectArray<UrdfLink*>         m_rootLinks;
        bool                                    m_overrideFixedBase;

        UrdfModel()
            : m_overrideFixedBase(false)
        {
            m_rootTransformInWorld.setIdentity();
        }

        ~UrdfModel()
        {
            for (int i = 0; i < m_materials.size(); i++)
            {
                UrdfMaterial** ptr = m_materials.getAtIndex(i);
                if (ptr)
                {
                    UrdfMaterial* t = *ptr;
                    delete t;
                }
            }
            for (int i = 0; i < m_links.size(); i++)
            {
                UrdfLink** ptr = m_links.getAtIndex(i);
                if (ptr)
                {
                    UrdfLink* t = *ptr;
                    delete t;
                }
            }
            for (int i = 0; i < m_joints.size(); i++)
            {
                UrdfJoint** ptr = m_joints.getAtIndex(i);
                if (ptr)
                {
                    UrdfJoint* t = *ptr;
                    delete t;
                }
            }
        }
    };

}
