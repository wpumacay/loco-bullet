
#include <test_interface.h>

#define TEST_LINK_LENGTH 0.5

class AppExample : public bullet::MultibodyTestApplication
{
    private :

    bullet::SimMultibody* _createMultiPendulum( const btVector3& position,
                                                size_t numLinks, 
                                                bool useSpherical = false );

    bullet::SimMultibody* _createPrismaticToy( const btVector3& position,
                                               bool useFixed = false );

    bullet::SimMultibody* _createPlanarToy( const btVector3& position );

    bullet::SimMultibody* _createNoBaseToy( const btVector3& position );

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

    auto _pendulum1 = _createMultiPendulum( { 0, 0, 1. + 3. * TEST_LINK_LENGTH }, 
                                            3,
                                            false );
    auto _pendulum2 = _createMultiPendulum( { 1., 1., 1. + 3 * TEST_LINK_LENGTH },
                                            3,
                                            true );

    auto _prismaticToy = _createPrismaticToy( { -1., 1., 1. }, false );
    auto _fixedToy = _createPrismaticToy( { -1, -1, 1. }, true );
    auto _noBaseToy = _createNoBaseToy( { 1., -1, 1. } );

    addSimMultibody( _pendulum1 );
    addSimMultibody( _pendulum2 );
    addSimMultibody( _prismaticToy );
    addSimMultibody( _fixedToy );
    addSimMultibody( _noBaseToy );
}

bullet::SimMultibody* AppExample::_createMultiPendulum( const btVector3& position ,
                                                        size_t numLinks, 
                                                        bool useSpherical )
{
    auto _pendulum = new bullet::SimMultibody( numLinks,
                                               position,
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

        if ( useSpherical )
        {
            _currentLink = _pendulum->setupLinkSingleJoint( q,
                                                            "box",
                                                            btVector3( 0.05 * TEST_LINK_LENGTH ,
                                                                       0.05 * TEST_LINK_LENGTH ,
                                                                       TEST_LINK_LENGTH ),
                                                            _localTransform,
                                                            _currentLink,
                                                            "spherical",
                                                            _jointAxis,
                                                            _jointPivot );
        }
        else
        {
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
    }

    _pendulum->ptrBtMultibody()->finalizeMultiDof();

    return _pendulum;
}

bullet::SimMultibody* AppExample::_createPrismaticToy( const btVector3& position,
                                                       bool useFixed )
{
    /* The prismatic toy for this example has the following structure
    *              ___  0.05    _______
    *             /__/| .....  /______/|  (b)    
    *             |__|/        |______|/         
    *              .             0.05
    *              .   0.05
    *              .
    *             ___  ____
    *            /__/|   |
    *       (a)  |  || 0.05               
    *            |  ||   |          
    *            |__|/   |        
    *                  ----          
    */

    auto _toy = new bullet::SimMultibody( 2,
                                          position,
                                          "sphere",
                                          btVector3( 0.025, 0.025, 0.025 ),
                                          1.,
                                          false );

    auto _baseLink = _toy->ptrRootLink();

    // Construct link (a) ------------------------------------------------------

    // define the localTransform (w.r.t. base)
    btTransform _localTransformLinkA;
    _localTransformLinkA.setIdentity();
    _localTransformLinkA.setOrigin( { 0., 0.,  - 0.5 * 0.025 - 0.05 - 0.5 * 0.05 } );

    // define the joint axis and pivot
    auto _jointAxisLinkA    = btVector3( 0., 0., 1. );
    auto _jointPivotLinkA   = btVector3( 0., 0., 0.5 * 0.05 );

    if ( useFixed )
    {
        auto _linkA = _toy->setupLinkSingleJoint( 0,
                                                  "box",
                                                  { 0.025, 0.025, 0.05 },
                                                  _localTransformLinkA,
                                                  _baseLink,
                                                  "fixed",
                                                  _jointAxisLinkA,
                                                  _jointPivotLinkA );
    }
    else
    {
        auto _linkA = _toy->setupLinkSingleJoint( 0,
                                                  "box",
                                                  { 0.025, 0.025, 0.05 },
                                                  _localTransformLinkA,
                                                  _baseLink,
                                                  "prismatic",
                                                  _jointAxisLinkA,
                                                  _jointPivotLinkA );        
    }


    // -------------------------------------------------------------------------

    // Construct link (b) ------------------------------------------------------

    // define the localTransform (w.r.t. base)
    btTransform _localTransformLinkB;
    _localTransformLinkB.setIdentity();
    _localTransformLinkB.setOrigin( { 0., 0.5 * 0.025 + 0.05 + 0.5 * 0.05, 0. } );
    _localTransformLinkB.getBasis().setEulerZYX( SIMD_HALF_PI, 0., 0. );

    // define the joint axis and pivot
    auto _jointAxisLinkB    = btVector3( 0., 0., 1. );
    auto _jointPivotLinkB   = btVector3( 0., 0., 0.5 * 0.05 );

    if ( useFixed )
    {
        auto _linkB = _toy->setupLinkSingleJoint( 1,
                                                  "box",
                                                  { 0.025, 0.025, 0.05 },
                                                  _localTransformLinkB,
                                                  _baseLink,
                                                  "fixed",
                                                  _jointAxisLinkB,
                                                  _jointPivotLinkB );
    }
    else
    {
        auto _linkB = _toy->setupLinkSingleJoint( 1,
                                                  "box",
                                                  { 0.025, 0.025, 0.05 },
                                                  _localTransformLinkB,
                                                  _baseLink,
                                                  "prismatic",
                                                  _jointAxisLinkB,
                                                  _jointPivotLinkB );
    }

    // -------------------------------------------------------------------------

    _toy->ptrBtMultibody()->finalizeMultiDof();

    return _toy;
}

// bullet::SimMultibody* _createPlanarToy( const btVector3& position )
// {
//     return NULL;
// }

bullet::SimMultibody* AppExample::_createNoBaseToy( const btVector3& position )
{
    auto _toy = new bullet::SimMultibody( 1,
                                          position,
                                          "none",
                                          btVector3( 0., 0., 0. ),
                                          0.,
                                          false );

    auto _baseLink = _toy->ptrRootLink();

    // Construct a single link (actual object) ---------------------------------

    // the local transform should be identity (at same pos/rot)
    btTransform _localTransformLink;
    _localTransformLink.setIdentity();

    auto _link = _toy->setupLinkSingleJoint( 0,
                                             "capsule",
                                             { 0.05, 0.05, 0.05 },
                                             _localTransformLink,
                                             _baseLink,
                                             "fixed",
                                             { 0., 0., 1. },
                                             { 0., 0., 0. } );

    // -------------------------------------------------------------------------

    _toy->ptrBtMultibody()->finalizeMultiDof();

    return _toy;
}

int main()
{
    auto _app = new AppExample();

    _app->start();
    _app->togglePause();

    while ( true )
    {
        if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_P ) )
            _app->togglePause();

        if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_ESCAPE ) )
            break;

        _app->step();
    }

    return 0;
}