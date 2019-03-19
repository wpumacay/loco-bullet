
#include <bullet_terrain_wrapper.h>




namespace tysoc {
namespace bullet {






    extern "C" TTerrainGenWrapper* terrain_createFromAbstract( terrain::TITerrainGenerator* terrainGenPtr,
                                                               const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }

    extern "C" TTerrainGenWrapper* terrain_createFromParams( const std::string& name,
                                                             const TGenericParams& params,
                                                             const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }

}}