
#include <model_loader.h>
#include <bullet_utils.h>

static std::string MODEL_FORMAT = "mjcf";
static std::string MODEL_NAME = "humanoid";

static std::string TYSOC_MJCF_TEMPLATES     = std::string( TYSOC_PATH_MJCF_TEMPLATES );
static std::string TYSOC_URDF_TEMPLATES     = std::string( TYSOC_PATH_URDF_TEMPLATES );
static std::string TYSOC_RLSIM_TEMPLATES    = std::string( TYSOC_PATH_RLSIM_TEMPLATES );

tysoc::agent::TAgentKinTree* createAgent( const std::string& format,
                                          const std::string& modelName,
                                          const std::string& agentName,
                                          const tysoc::TVec3& position,
                                          const tysoc::TVec3& rotation = tysoc::TVec3() )
{
    auto _modelLoader = tysoc::TModelLoader::Create();

    if ( format == "urdf" )
    {
        auto _modelData = _modelLoader->getUrdfModel( modelName );

        return tysoc::agent::createKinTreeAgent( agentName, _modelData, position, rotation );
    }
    else if ( format == "rlsim" )
    {
        auto _modelData = _modelLoader->getRlsimModel( modelName );
        
        return tysoc::agent::createKinTreeAgent( agentName, _modelData, position, rotation );
    }
    else if ( format == "mjcf" )
    {
        auto _modelData = _modelLoader->getMjcfModel( modelName );
        
        return tysoc::agent::createKinTreeAgent( agentName, _modelData, position, rotation );
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

    int _numLinks = tysoc::bullet::utils::calculateNumOfLinksForMultibody( _agent );

    std::cout << "INFO> Num of links for model: " << MODEL_NAME << " is: " << _numLinks << std::endl;

    return 0;
}
