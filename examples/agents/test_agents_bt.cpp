
#include <runtime.h>
#include <model_loader.h>
#include <bullet_config.h>

static std::string MODEL_FORMAT = "mjcf";
static std::string MODEL_NAME = "walker";

static std::string TYSOC_MJCF_TEMPLATES     = std::string( TYSOC_PATH_MJCF_TEMPLATES );
static std::string TYSOC_URDF_TEMPLATES     = std::string( TYSOC_PATH_URDF_TEMPLATES );
static std::string TYSOC_RLSIM_TEMPLATES    = std::string( TYSOC_PATH_RLSIM_TEMPLATES );

tysoc::agent::TAgent* createAgent( const std::string& format,
                                          const std::string& modelName,
                                          const std::string& agentName,
                                          const tysoc::TVec3& position,
                                          const tysoc::TVec3& rotation = tysoc::TVec3() )
{
    auto _modelLoader = tysoc::TModelLoader::Create();

    if ( format == "urdf" )
    {
        auto _modelData = _modelLoader->getUrdfModel( modelName );

        return tysoc::agent::createAgentFromModel( _modelData, agentName, position, rotation );
    }
    else if ( format == "rlsim" )
    {
        auto _modelData = _modelLoader->getRlsimModel( modelName );
        
        return tysoc::agent::createAgentFromModel( _modelData, agentName, position, rotation );
    }
    else if ( format == "mjcf" )
    {
        auto _modelData = _modelLoader->getMjcfModel( modelName );
        
        return tysoc::agent::createAgentFromModel( _modelData, agentName, position, rotation );
    }

    std::cout << "ERROR> format: " << format << " not supported" << std::endl;
    return NULL;
}

int main( int argc, const char** argv )
{
    if ( argc > 2 )
    {
        try
        {
            MODEL_FORMAT = std::string( argv[1] );
            MODEL_NAME = std::string( argv[2] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> should pass FORMAT(mjcf|urdf|rlsim) and MODEL_NAME(see templates)" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }
    }

    /* ***************************************************************************/

    auto _agent = createAgent( MODEL_FORMAT, MODEL_NAME, "agent0", { 0.0f, 0.0f, 2.5f }, { 0.0, 0.0, 0.0 } );

    if ( !_agent )
    {
        std::cout << "ERROR> (format|model): " 
                  << MODEL_FORMAT << "|" << MODEL_NAME 
                  << " not found" << std::endl;
        return 1;
    }

    // auto _bplane = new tysoc::sandbox::TBody();
    // _bplane->name = "bplane";
    // _bplane->type = "box";
    // _bplane->friction = { 1.0, 1.0, 1.0 };
    // _bplane->size = { 100.0, 100.0, 1.0 };
    // _bplane->color = { 0.3, 0.3, 0.3 };
    // _bplane->worldTransform.setPosition( { 0, 0, -0.5 } );

    auto _terrainGenStatic = new tysoc::terrain::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "box", 
                                        { 10.0f, 10.0f, 1.0f }, 
                                        { 0.0f, 0.0f, -0.5f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f } );

    // auto _bcapsule = new tysoc::sandbox::TFreeBody();
    // _bcapsule->name = "bcapsule";
    // _bcapsule->type = "capsule";
    // _bcapsule->mass = 0.1;
    // _bcapsule->size = { 0.1, 0.4, 0.0 };
    // _bcapsule->worldTransform.setPosition( { -1.0, -1.0, 1.0 } );

    auto _scenario = new tysoc::TScenario();
    _scenario->addAgent( _agent );
    _scenario->addTerrainGenerator( _terrainGenStatic );
    // _scenario->addBody( _bplane );
    // _scenario->addBody( _bcapsule );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::BULLET, 
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    _simulation->togglePause();

    while ( _visualizer->isActive() )
    {
        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_P ) )
            _simulation->togglePause();

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_ESCAPE ) )
            break;

        _simulation->step();
        _visualizer->update();
    }

    _runtime->destroyVisualizer();
    _runtime->destroySimulation();
    _visualizer = NULL;
    _simulation = NULL;

    return 0;
}
