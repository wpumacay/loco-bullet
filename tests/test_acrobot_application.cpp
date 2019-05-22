
#include <test_interface.h>

class AppExample : public bullet::MultibodyTestApplication
{
    protected :

    void _initScenario() override;

    public :

    AppExample();
    ~AppExample();
};

AppExample::AppExample()
    : bullet::MultibodyTestApplication()
{
    // nothing here
}

AppExample::~AppExample()
{
    // nothing here
}

void AppExample::_initScenario()
{
    // Create the base
    auto _base = createBody( "box",
                             btVector3( 10, 10, 1 ),
                             btVector3( 0, 0, -0.5 ),
                             btVector3( 0, 0, 0 ),
                             btVector3( 0.3, 0.3, 0.3 ),
                             0.0f );

    auto _basePosition = btVector3( 0, 0, 2.5 );
    auto _baseShape = std::string( "sphere" );
    auto _baseSize = btVector3( 0.1, 0.1, 0.1 );
    auto _baseMass = 1.0f;

//     auto _acrobot = new bullet::SimMultibody( 3,
//                                               _basePosition,
//                                               _baseShape,
//                                               _baseSize,
//                                               _baseMass,
//                                               false );
// 
//     _acrobot->setupLink(  )
}

int main()
{

    auto _app = new AppExample();

    _app->start();

    while ( true )
    {
        _app->step();
    }

    return 0;
}