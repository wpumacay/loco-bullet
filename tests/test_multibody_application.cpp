
#include <test_interface.h>

#define TEST_LINK_LENGTH 0.5

class AppExample : public bullet::MultibodyTestApplication
{
    private :

    bullet::SimMultibody* _createMultiPendulum( size_t numLinks );

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
                             btVector3( 0, 0, -2.5 ),
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

    auto _pendulum = _createMultiPendulum( 3 );
    addSimMultibody( _pendulum );
}

bullet::SimMultibody* AppExample::_createMultiPendulum( size_t numLinks )
{
    auto _pendulum = new bullet::SimMultibody( numLinks,
                                               btVector3( 0., 0., 1. + TEST_LINK_LENGTH * numLinks ),
                                               "sphere",
                                               btVector3( 0.1 * TEST_LINK_LENGTH,
                                                          0.1 * TEST_LINK_LENGTH,
                                                          0.1 * TEST_LINK_LENGTH ),
                                               1.,
                                               true );

    auto _currentLink = _pendulum->ptrRootLink();
    for ( size_t q = 0; q < numLinks; q++ )
    {
        // define localTransform (w.r.t. parent)
        btTransform _localTransform;
        _localTransform.setIdentity();
        _localTransform.setOrigin( btVector3( 0., 
                                              0., 
                                              ( q == 0 ) ? -0.5 * TEST_LINK_LENGTH : -1.0 * TEST_LINK_LENGTH ) );

        // define jointAxis and jointPivot (w.r.t. this)
        auto _jointAxis     = btVector3( 1., 0., 0. );
        auto _jointPivot    = btVector3( 0., 0., 0.5 * TEST_LINK_LENGTH );

        _currentLink = _pendulum->setupLinkSingleJoint( q,
                                                        "box",
                                                        btVector3( 0.05 * TEST_LINK_LENGTH ,
                                                                   0.05 * TEST_LINK_LENGTH ,
                                                                   TEST_LINK_LENGTH ),
                                                        _localTransform,
                                                        _currentLink,
                                                        "revolute",
                                                        _jointAxis,
                                                        _jointPivot );
    }

    _pendulum->ptrBtMultibody()->finalizeMultiDof();

    return _pendulum;
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