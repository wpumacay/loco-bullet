
#pragma once

// some functionality from bullet
#include <bullet_common.h>

// Assimp helper functionality
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// visualizer, for debug drawer
#include <viz/viz.h>

// kintree functionality
#include <agent/agent_base.h>

namespace tysoc {
namespace bullet {
namespace utils {

    /**
    *   Mesh structure for non-primitive collision shapes
    */
    struct TMeshObject
    {
        std::vector< TVec3 > vertices;
    };

    /**
    *   Creates a bullet btVector3 from a TVec3
    *
    *   @param vec  TVec3 to convert
    */
    btVector3 toBtVec3( const TVec3& vec );

    /**
    *   Creates a TVec3 from a bullet btVector3
    *
    *   @param vec  btVector3 to convert
    */
    TVec3 fromBtVec3( const btVector3& vec );

    /**
    *   Creates a bullet btMatrix3x3 from a TMat3
    *
    *   @param mat  TMat3 to convert
    */
    btMatrix3x3 toBtMat3( const TMat3& mat );

    /**
    *   Creates a TMat3 from a bullet btMatrix3x3
    *
    *   @param mat  btMatrix3x3 to convert
    */
    TMat3 fromBtMat3( const btMatrix3x3& mat );

    /**
    *   Creates a TMat4 from a bullet btTransform
    *
    *   @param tf   btTransform to convert
    */
    TMat4 fromBtTransform( const btTransform& tf );

    /**
    *   Creates a btTransform from a bullet TMat4
    *
    *   @param mat      TMat4 to convert
    */
    btTransform toBtTransform( const TMat4& mat );

    /**
    *   Creates a btCollisionShape from the given type and size
    *
    *   @param type     The type of shape to create
    *   @param size     The size of the shape, which is defined as follows:
    *                      > box            : {width,height,depth} <> {dx,dy,dz}
    *                      > capsule        : {radius,height,N/A} -> axis along Z
    *                      > capsule(XYZ)   : same as before, axis along given axis
    *                      > cylinder       : {radius,height,N/A} -> axis along Z
    *                      > cylinder(XYZ)  : same as before, axis along given axis
    *                      > sphere         : {radius,N/A,N/A}
    *
    */
    btCollisionShape* createCollisionShape( const std::string& type, const TVec3& size );

    /**
    *   Computes the volume of a given collision shape using their actual ...
    *   dimensions (including margings as well)
    *
    *   @param colShape     Collision shape from whom we want to compute its volume
    */
    btScalar computeVolumeFromShape( btCollisionShape* colShape );

    /**
    *   Computes how many links should a btMultiBody allocate for a given ...
    *   kinematic tree, based on dummies and actual links to be created
    *
    *   @param kintree  TAgent object in question
    */
    size_t calculateNumOfLinksForMultibody( agent::TAgent* kinTreePtr );

    /**
    *   Decides whether or not the base of a multibody should be fixed, ...
    *   considering the kintree given. Some cases require a non-fixed base, ...
    *   like in the humanoid (it has a free joint), whereas others require ...
    *   a fixed base (like the planar walker).
    *
    *   @param kintree  TAgent object in question
    */
    bool shouldBaseBeFixed( agent::TAgent* kinTreePtr );

    /**
    *   Loads a mesh from a given file
    *
    *   @param filePath     Full path to the file to be loaded
    *   @param mesh         A reference to the mesh to be populated in-place
    */
    void loadMesh( const std::string& filePath, TMeshObject& mesh );

    /**
    *   Internal helper function to process a node of the tree from an assimp scene
    *
    *   @param assimpScenePtr   Scene currently being processed
    *   @param assimpNodePtr    Node from the assimp-scene to be processed
    *   @param mesh             A reference to the mesh to be populated in-place
    */
    void _processAssimpNode( const aiScene* assimpScenePtr,
                             aiNode* assimpNodePtr, 
                             TMeshObject& mesh );

    /**
    *   Internal helper function to process a mesh from an assimp-node
    *
    *   @param assimpMeshPtr    Mesh from the assimp-node to be processed
    *   @param mesh             A reference to the mesh to be populated in-place
    */
    void _processAssimpMesh( aiMesh* assimpMeshPtr, TMeshObject& mesh );

    /**
    *   Simple singleton file-logger used for debugging 
    *   some internal configurations
    */
    class TBtLogger
    {
        private :

        // logs buffer of lines to be logged to a file
        std::vector< std::string > m_logs;

        /* saves the logs list into a .txt file given a file name (abspath) */
        void _saveInternal( const std::string& filename );

        /* stores a single log message into the logs buffer */
        void _logInternal( const std::string& message );

        /* constructor, hiden as we are using a singleton */
        TBtLogger() {}

        /* private global instance of the logger */
        static TBtLogger* _instance;

        public :
        
        /* Saves all logs given by the user into the given file (abspath)  */
        static void save( const std::string& filename );

        /* Stores a log given by the user into the logs buffer */
        static void log( const std::string& message );

    };

    std::string to_string( const btVector3& vec3 );
    std::string to_string( const btQuaternion& quat );
    std::string to_string( const btMatrix3x3& mat3 );
    std::string to_string( const btTransform& transform );

    /**
    *   Debug drawer, implemeting interface btIDebugDraw, to handle drawing of ...
    *   all debugdraw functionality provided (and tested) by bullet itself
    */
    class TBtDebugDrawer : public btIDebugDraw
    {
        private :

        // visualizer reference
        viz::TIVisualizer* m_visualizerPtr;

        // bit fields used for the mode
        int m_debugMode;

        public : 

        TBtDebugDrawer();
        virtual ~TBtDebugDrawer();

        void setVisualizer( viz::TIVisualizer* visualizerPtr );

        void drawLine( const btVector3& from, const btVector3& to, const btVector3& color ) override;
        void drawContactPoint( const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color ) override;
        void reportErrorWarning( const char* warningString ) override;
        void draw3dText( const btVector3& location, const char* textString ) override;
        void setDebugMode( int debugMode ) override;
        int getDebugMode() const override;
    };

    /**
    *   Custom broadphase filer callback, to allow collision checks similar ...
    *   to MuJoCo. We will check in the following way:
    *   
    *   |   MuJoCo     |   Bullet   |
    *   |--------------|------------|
    *   | contype     <-> collgroup | 
    *   | conaffinity <-> collmask  |
    */
    struct TBtOverlapFilterCallback : public btOverlapFilterCallback
    {
        // custom collision checking
        bool needBroadphaseCollision( btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1 ) const override;
    };

}}}