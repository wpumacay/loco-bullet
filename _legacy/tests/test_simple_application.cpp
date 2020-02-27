
#include <test_interface.h>

class AppExample : public bullet::SimpleTestApplication
{
    protected :

    void _initScenario() override;

    public :

    AppExample();
    ~AppExample();
};

AppExample::AppExample()
    : bullet::SimpleTestApplication()
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

    // Create a testing capsule
    auto _capsule = createBody( "capsule",
                                btVector3( 0.1, 0.4, 0 ),
                                btVector3( 1, 1, 1 ),
                                btVector3( 0, 0, 0 ),
                                btVector3( 0.7, 0.5, 0.3 ),
                                0.5f );
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