
#include <tysocBulletFactory.h>

namespace tysocBullet
{


    TBulletFactory::TBulletFactory()
    {

    }

    TBulletFactory::~TBulletFactory()
    {

    }


    TBulletTerrainGenWrapper* TBulletFactory::createTerrainGen( const std::string& name,
                                                                const std::string& type,
                                                                const TGenericParams& params )
    {
        tysocterrain::TTerrainGenerator* _terrainGenerator = NULL;

        if ( type == "procedural" )
        {
            auto _sectionsType = params.getString( "sectionType", "blocky" );

            if ( _sectionsType == "path" )
            {
                tysocterrain::TProfileGenerator* _profileGenerator;
                if ( params.getString( "pathProfile" ) == "sine" )
                {
                    float _ampl     = params.getFloat( "sineProfileAmplitude", 2.0f );
                    float _period   = params.getFloat( "sineProfilePeriod", 10.0f );
                    float _phase    = params.getFloat( "sineProfilePhase", 1.57f );
                    _profileGenerator = new tysocterrain::TSineProfileGenerator( _ampl, 
                                                                                 _period, 
                                                                                 _phase );
                }
                else if ( params.getString( "pathProfile" ) == "perlin" )
                {
                    int _octaves        = params.getInt( "perlinProfileOctaves", 4 );
                    float _persistance  = params.getFloat( "perlinProfilePersistance", 0.5f );
                    float _lacunarity   = params.getFloat( "perlinProfileLacunarity", 2.0f );
                    float _noiseScale   = params.getFloat( "perlinProfileNoiseScale", 10.0f );
                    _profileGenerator = new tysocterrain::TPerlin1DProfileGenerator( _octaves,
                                                                                     _persistance,
                                                                                     _lacunarity,
                                                                                     _noiseScale );
                }
                else
                {
                    _profileGenerator = new tysocterrain::TSineProfileGenerator( 2.0f, 10.0f, 1.57f );
                }

                float _sectionDepth         = params.getFloat( "sectionDepth", 1.0f );
                float _componentsSpacingX   = params.getFloat( "componentsSpacingX", 0.5f );
                float _componentsThickness  = params.getFloat( "componentsThickness", 0.01f );

                float _startX = params.getFloat( "startX" );
                float _startY = params.getFloat( "startY" );
                float _startZ = params.getFloat( "startZ" );

                _terrainGenerator = new tysocterrain::TPathTerrainGenerator( name, 
                                                                             _startX,
                                                                             _startY,
                                                                             _startZ,
                                                                             _sectionDepth,
                                                                             _componentsSpacingX,
                                                                             _componentsThickness,
                                                                             _profileGenerator );

            }
            else
            {
                tysocterrain::TBlockyParams _bparams;

                _bparams.usesBase               = params.getInt( "sectionUsesBase" ) == 1;
                _bparams.usesSides              = params.getInt( "sectionUsesSides" ) == 1;
                _bparams.sectionLength          = params.getFloat( "sectionLength" );
                _bparams.baseDepth              = params.getFloat( "sectionDepth" );
                _bparams.baseHeight             = params.getFloat( "sectionBlockyBaseHeight" );
                _bparams.baseWidth              = params.getFloat( "sectionBlockyBaseWidth" );
                _bparams.baseSpacingX           = params.getFloat( "sectionBlockyBaseSpacingX" );
                _bparams.baseOffsetZ            = params.getFloat( "sectionBlockyBaseOffsetZ" );
                _bparams.percentDepth.min       = params.getFloat( "sectionBlockyPercentDepthMin" );
                _bparams.percentDepth.max       = params.getFloat( "sectionBlockyPercentDepthMax" );
                _bparams.percentHeight.min      = params.getFloat( "sectionBlockyPercentHeightMin" );
                _bparams.percentHeight.max      = params.getFloat( "sectionBlockyPercentHeightMax" );
                _bparams.percentWidth.min       = params.getFloat( "sectionBlockyPercentWidthMin" );
                _bparams.percentWidth.max       = params.getFloat( "sectionBlockyPercentWidthMax" );
                _bparams.percentSpacingX.min    = params.getFloat( "sectionBlockyPercentSpacingXMin" );
                _bparams.percentSpacingX.max    = params.getFloat( "sectionBlockyPercentSpacingXMax" );
                _bparams.percentOffsetZ.min     = params.getFloat( "sectionBlockyPercentOffsetZMin" );
                _bparams.percentOffsetZ.max     = params.getFloat( "sectionBlockyPercentOffsetZMax" );

                float _startX = params.getFloat( "startX" );
                float _startY = params.getFloat( "startY" );
                float _startZ = params.getFloat( "startZ" );

                _terrainGenerator = new tysocterrain::TBlockyTerrainGenerator( name,
                                                                               _startX,
                                                                               _startY,
                                                                               _startZ,
                                                                               _bparams );
            }
        }

        auto _terrainGeneratorWrapper = new TBulletTerrainGenWrapper( name, _terrainGenerator );

        return _terrainGeneratorWrapper;
    }
}