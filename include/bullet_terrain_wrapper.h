
#pragma once

#include <bullet_common.h>
#include <bullet_utils.h>

// Pool size for the number of mjc bodies to use
#define BT_TERRAIN_POOL_SIZE PROCEDURAL_TERRAIN_POOL_SIZE - 1
// Default parameters for the connected-paths section
#define BT_TERRAIN_PATH_DEFAULT_WIDTH 0.5f
#define BT_TERRAIN_PATH_DEFAULT_DEPTH 2.0f
#define BT_TERRAIN_PATH_DEFAULT_TICKNESS 0.025f

#include <terrain_wrapper.h>

namespace tysoc {
namespace bullet {

    /**
    * This is a wrapper on top of the primitives...
    * used by the terrain generators
    */
    struct TBtTerrainPrimitive
    {
        std::string                     btPrimitiveName;    // name of this primitive
        std::string                     btPrimitiveType;    // type of shape of this primitive
        TVec3                           btPrimitiveSize;    // size of this primitive
        std::string                     btPrimitiveFilename;// mesh filename (if applicable)
        btRigidBody*                    btPrimitiveBodyPtr; // bullet rigid body for this primitive
        terrain::TTerrainPrimitive*     tysocPrimitiveObj;  // terrain primitive being wrapped
    };


    class TBtTerrainGenWrapper : public TTerrainGenWrapper
    {
        private :

        // Main storage for the primitives
        std::vector< TBtTerrainPrimitive* > m_btTerrainPrimitives;
        // working queues for the logic
        std::queue< TBtTerrainPrimitive* > m_btAvailablePrimitives;
        std::queue< TBtTerrainPrimitive* > m_btWorkingPrimitives;

        // A reference to the world to add our resources to
        btMultiBodyDynamicsWorld* m_btWorldPtr;

        void _collectReusableFromGenerator();// collects primitives that can be reused and rewrapped in the lifetime of the generator
        void _collectStaticFromGenerator();// collects primitives that are single in the lifetime of the generator
        void _wrapReusablePrimitive( terrain::TTerrainPrimitive* primitivePtr );
        void _wrapStaticPrimitive( terrain::TTerrainPrimitive* primitivePtr );
        void _createBtBody( TBtTerrainPrimitive* btTerrainPrimitivePtr );
        void _updateProperties( TBtTerrainPrimitive* btTerrainPritimivePtr );


        protected :

        void _initializeInternal() override;
        void _resetInternal() override;
        void _preStepInternal() override;
        void _postStepInternal() override;

        public :

        TBtTerrainGenWrapper( terrain::TITerrainGenerator* terrainGenPtr,
                               const std::string& workingDir );
        ~TBtTerrainGenWrapper();

        void setBtWorld( btMultiBodyDynamicsWorld* btWorldPtr );
    };




    extern "C" TTerrainGenWrapper* terrain_createFromAbstract( terrain::TITerrainGenerator* terrainGenPtr,
                                                               const std::string& workingDir );

    extern "C" TTerrainGenWrapper* terrain_createFromParams( const std::string& name,
                                                             const TGenericParams& params,
                                                             const std::string& workingDir );


}}