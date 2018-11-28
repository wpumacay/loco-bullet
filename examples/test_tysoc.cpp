
#include <tysocBullet.h>
#include <tysocViz.h>

#define SECTION_DEPTH 3.0f

static int NUM_AGENTS = 1;

int main( int argc, const char** argv )
{
    if ( argc > 1 )
    {
        try
        {
            NUM_AGENTS = std::stoi( argv[1] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> Should pass an int for numagents" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }
    }

    /* ***************************************************************************/
    auto _tysocApi = new tysocBullet::TTysocBulletApi();
    auto _factory = new tysocBullet::TBulletFactory();

    tysocBullet::TGenericParams _terrainParams;
    // // sections - path - perlin profile
    // {
    //     _terrainParams.set( "sectionType", "path" );
    //     _terrainParams.set( "sectionDepth", SECTION_DEPTH );
    //     _terrainParams.set( "pathProfile", "perlin" );
    //     _terrainParams.set( "perlinProfileOctaves", 4 );
    //     _terrainParams.set( "perlinProfilePersistance", 0.5f );
    //     _terrainParams.set( "perlinProfileLacunarity", 2.0f );
    //     _terrainParams.set( "perlinProfileNoiseScale", 10.0f );
    // }
    // // sections - path - sine profile
    // {
    //     _terrainParams.set( "sectionType", "path" );
    //     _terrainParams.set( "sectionDepth", SECTION_DEPTH );
    //     _terrainParams.set( "pathProfile", "sine" );
    //     _terrainParams.set( "sineProfileAmplitude", 1.0f );
    //     _terrainParams.set( "sineProfilePeriod", 10.0f );
    //     _terrainParams.set( "sineProfilePhase", 1.57f );
    // }
    // sections - blocky
    {
        _terrainParams.set( "sectionType", "blocky" );
        _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        _terrainParams.set( "sectionLength", 250.0f );
        _terrainParams.set( "sectionUsesBase", 1 );
        _terrainParams.set( "sectionUsesSides", 1 );
        _terrainParams.set( "sectionBlockyBaseHeight", 0.5f );
        _terrainParams.set( "sectionBlockyBaseWidth", 0.25f );
        _terrainParams.set( "sectionBlockyBaseSpacingX", 4.0f );
        _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.0f );
        _terrainParams.set( "sectionBlockyPercentDepthMin", 0.25f );//1.0f
        _terrainParams.set( "sectionBlockyPercentDepthMax", 0.75f );//1.0f
        _terrainParams.set( "sectionBlockyPercentHeightMin", 0.75f );
        _terrainParams.set( "sectionBlockyPercentHeightMax", 1.25f );
        _terrainParams.set( "sectionBlockyPercentWidthMin", 0.5f );
        _terrainParams.set( "sectionBlockyPercentWidthMax", 2.0f );
        _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.9f );
        _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.1f );
        _terrainParams.set( "sectionBlockyPercentOffsetZMin", 1.0f );
        _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.0f );

        // _terrainParams.set( "sectionType", "blocky" );
        // _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        // _terrainParams.set( "sectionLength", 250.0f );
        // _terrainParams.set( "sectionUsesBase", 1 );
        // _terrainParams.set( "sectionUsesSides", 0 );
        // _terrainParams.set( "sectionBlockyBaseHeight", 0.5f );
        // _terrainParams.set( "sectionBlockyBaseWidth", 0.25f );
        // _terrainParams.set( "sectionBlockyBaseSpacingX", 4.0f );
        // _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.0f );
        // _terrainParams.set( "sectionBlockyPercentDepthMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentDepthMax", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentHeightMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentHeightMax", 1.25f );
        // _terrainParams.set( "sectionBlockyPercentWidthMin", 0.5f );
        // _terrainParams.set( "sectionBlockyPercentWidthMax", 2.0f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.9f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.1f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.0f );

        // _terrainParams.set( "sectionType", "blocky" );
        // _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        // _terrainParams.set( "sectionLength", 250.0f );
        // _terrainParams.set( "sectionUsesBase", 0 );
        // _terrainParams.set( "sectionUsesSides", 0 );
        // _terrainParams.set( "sectionBlockyBaseHeight", 0.1f );
        // _terrainParams.set( "sectionBlockyBaseWidth", 1.0f );
        // _terrainParams.set( "sectionBlockyBaseSpacingX", 2.5f );
        // _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.0f );
        // _terrainParams.set( "sectionBlockyPercentDepthMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentDepthMax", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentHeightMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentHeightMax", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentWidthMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentWidthMax", 1.25f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.25f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMin", 1.0f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.0f );

        // _terrainParams.set( "sectionType", "blocky" );
        // _terrainParams.set( "sectionDepth", SECTION_DEPTH );
        // _terrainParams.set( "sectionLength", 250.0f );
        // _terrainParams.set( "sectionUsesBase", 1 );
        // _terrainParams.set( "sectionUsesSides", 1 );
        // _terrainParams.set( "sectionBlockyBaseHeight", 0.05f );
        // _terrainParams.set( "sectionBlockyBaseWidth", 0.75f );
        // _terrainParams.set( "sectionBlockyBaseSpacingX", 4.0f );
        // _terrainParams.set( "sectionBlockyBaseOffsetZ", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentDepthMin", 0.5f );
        // _terrainParams.set( "sectionBlockyPercentDepthMax", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentHeightMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentHeightMax", 1.25f );
        // _terrainParams.set( "sectionBlockyPercentWidthMin", 0.5f );
        // _terrainParams.set( "sectionBlockyPercentWidthMax", 2.0f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMin", 0.9f );
        // _terrainParams.set( "sectionBlockyPercentSpacingXMax", 1.1f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMin", 0.75f );
        // _terrainParams.set( "sectionBlockyPercentOffsetZMax", 1.25f );
    }

    auto _scenario = new tysoc::TScenario();
    _tysocApi->setScenario( _scenario );

    for ( size_t i = 0; i < NUM_AGENTS; i++ )
    {
        float _startX = 0.0f;
        float _startY = i * ( SECTION_DEPTH + 1.0f );
        float _startZ = 0.0f;

        // create terrain wrapper
        _terrainParams.set( "startX", _startX );
        _terrainParams.set( "startY", _startY );
        _terrainParams.set( "startZ", _startZ );
        auto _terrain = _factory->createTerrainGen( std::string( "terrain_proc" ) + std::to_string( i ),
                                                    "procedural", _terrainParams );
        
        auto _terrainGen        = _terrain->terrainGenerator();
        auto _terrainGenInfo    = _terrainGen->generatorInfo();
        _terrainGenInfo->trackingpoint.x = 0.0f;
        _terrainGenInfo->trackingpoint.y = i * ( SECTION_DEPTH + 1.0f );
        _terrainGenInfo->trackingpoint.z = 0.0f;

        _tysocApi->addTerrainGenWrapper( _terrain );
    }

    if ( !_tysocApi->initializeBulletApi() )
    {
        std::cout << "There was an error initializing the BulletApi" << std::endl;
        return 1;
    }

    /* ***************************************************************************/

    auto _viz = new tysocViz::TVisualizer( _tysocApi );
    _viz->initialize();

    float _currentX = 0.0f;

    while( _viz->isActive() )
    {
        // update api
        _tysocApi->step();

        // update visualizer
        _viz->update();

        _currentX += 0.025f;

        auto _terrainGens = _tysocApi->getTerrainGenerators();
        for ( size_t i = 0; i < _terrainGens.size(); i++ )
        {
            auto _genInfoPtr = _terrainGens[i]->generatorInfo();
            _genInfoPtr->trackingpoint.x = _currentX;
        }

        if ( engine::InputSystem::isKeyDown( GLFW_KEY_SPACE ) )
        {
            std::cout << "Spawned object" << std::endl;
            _tysocApi->getPrimitivesSpawner()->spawnObject( "box",
                                                            0.25f, 0.25f, 0.25f,
                                                            _currentX, 0.0f, 4.0f );
        }
    }

    delete _viz;
    delete _tysocApi;

    return 0;
}
