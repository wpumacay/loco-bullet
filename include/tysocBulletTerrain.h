
#pragma once

#include <tysocBulletCommon.h>

// Pool size for the number of bullet bodies to use
#define BULLET_TERRAIN_POOL_SIZE PROCEDURAL_TERRAIN_POOL_SIZE - 1
// Default parameters for the connected-paths section
#define BULLET_TERRAIN_PATH_DEFAULT_WIDTH 0.5f
#define BULLET_TERRAIN_PATH_DEFAULT_DEPTH 2.0f
#define BULLET_TERRAIN_PATH_DEFAULT_TICKNESS 0.025f

#include <terrain/terrain.h>

namespace tysocBullet
{

    /**
    * This is a wrapper on top of the primitives...
    * used by the terrain generators
    */
    struct TBulletTerrainPrimitive
    {
        std::string                             bulletBodyName;
        std::string                             bulletGeomType;
        struct { float x; float y; float z; }   bulletGeomSize;
        bool                                    isAvailable;
        btRigidBody*                            bulletRigidBodyObj;
        tysocterrain::TTerrainPrimitive*        tysocPrimitiveObj;
    };


    class TBulletTerrainGenWrapper
    {
        private :

        // Main storage for the primitives
        std::vector< TBulletTerrainPrimitive* > m_bulletTerrainPrimitives;
        // working queues for the logic
        std::queue< TBulletTerrainPrimitive* > m_bulletAvailablePrimitives;
        std::queue< TBulletTerrainPrimitive* > m_bulletWorkingPrimitives;
        std::queue< TBulletTerrainPrimitive* > m_bulletFixedPrimitives;

        // terrain generator to wrap
        tysocterrain::TTerrainGenerator* m_terrainGenPtr;

        // a reference to the bullet resources (just dynamicsworld for now)
        btDynamicsWorld* m_bulletWorldPtr;

        // name for this agentwrapper (and underlying agent as well)
        std::string m_name;

        void _collectFromGenerator();// collects primitives that can be reused and rewrapped in the lifetime of the generator
        void _collectFixedFromGenerator();// collects primitives that are single in the lifetime of the generator
        void _wrapNewPrimitive( tysocterrain::TTerrainPrimitive* primitivePtr, bool isReusable );
        void _updateProperties( TBulletTerrainPrimitive* bulletTerrainPritimivePtr );

        btRigidBody* _createBodyResource( TBulletTerrainPrimitive* bPrimitivePtr,
                                          float x, float y, float z );

        public :

        TBulletTerrainGenWrapper( const std::string& name,
                                  tysocterrain::TTerrainGenerator* terrainGenPtr );
        ~TBulletTerrainGenWrapper();

        void setBulletWorld( btDynamicsWorld* worldPtr );
        void initialize();

        std::string name() { return m_name; }
        tysocterrain::TTerrainGenerator* terrainGenerator() { return m_terrainGenPtr; }


        // update the wrapper by collecting all ...
        // information needed for stages on top of ...
        // the wrapper, like its concrete TTysocBulletApi parent
        void preStep();
    };

}