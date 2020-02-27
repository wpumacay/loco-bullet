
#include <test_interface.h>

class AppExample : public bullet::MultibodyTestApplication
{
    private :

    bullet::SimMultibody* _createWalkerPlanar( const std::string& name,
                                               const btVector3& position );

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

    auto _walkerPlanar = _createWalkerPlanar( "walker", { 0., 0., 1. } );

    addSimMultibody( _walkerPlanar );
}

// Considerations: instead of using a full kintree (as in tysoc) we will just ...
// define everything w.r.t. the geom instead. This is possible for the case of ...
// the walker, as it has 1 geom per body. Furthermore, there are no rotations ...
// between joints and geoms w.r.t. the bodies, so we can just move/translate ...
// the reference frames appropriately.

bullet::SimMultibody* AppExample::_createWalkerPlanar( const std::string& name,
                                                       const btVector3& position )
{
    auto _simbody = new bullet::SimMultibody( name,
                                              (3 + 1) + 6,
                                              position,
                                              "none",
                                              { 0., 0., 0. },
                                              0.,
                                              true );

    // define the dofs of the torso
    std::vector< std::string > _jointsTypes = { "prismatic", "prismatic", "revolute" };
    std::vector< btVector3 > _jointsAxes    = { { 1., 0., 0. }, { 0., 0., 1. }, { 0., 1., 0. } };
    std::vector< btVector3 > _jointsPivots  = { { 0., 0., 0. }, { 0., 0., 0. }, { 0., 0., 0. } };
    std::vector< bool > _jointsUseMotors   = { false, false, false };
    std::vector< float > _jointsLowerLimits = { 1., 1., -3.1415 };
    std::vector< float > _jointsUpperLimits = { -1., -1., 3.1415 };

    // define the transform of the torso w.r.t. the base (they coincide, so it's just identity)
    btTransform _trTorsoToBase;
    _trTorsoToBase.setIdentity();
    _trTorsoToBase.setOrigin( { 0., 0., 0. } ); // torso geom has same position as torso body

    auto _torsoLinks = _simbody->setupLinkMultiDof( 0,
                                                    "capsule",
                                                    { 0.07, 0.6, 0.07 },
                                                    _trTorsoToBase,
                                                    _simbody->ptrRootLink(),
                                                    _jointsTypes,
                                                    _jointsAxes,
                                                    _jointsPivots,
                                                    _jointsLowerLimits,
                                                    _jointsUpperLimits,
                                                    _jointsUseMotors );

    auto _currentLinkIndx = 0 + _torsoLinks.size();

    // define the transform of the right-thigh w.r.t. the torso
    btTransform _trRightThighToTorso;
    _trRightThighToTorso.setIdentity();
    _trRightThighToTorso.setOrigin( { 0., -0.05, -0.3 - 0.225 } ); // geom is slightly to -z of the body

    auto _rightThigh = _simbody->setupLinkSingleJoint( _currentLinkIndx,
                                                       "capsule",
                                                       { 0.05, 0.45, 0.05 },
                                                       _trRightThighToTorso,
                                                       _torsoLinks.back(),
                                                       "revolute",
                                                       { 0., -1., 0. },
                                                       { 0., 0., 0. - (-0.225) },
                                                       -SIMD_PI / 9., SIMD_PI * 5. / 9. );

    // next link please
    _currentLinkIndx += 1;

    // define the transform of the right-leg w.r.t. the right-thigh
    btTransform _trRightLegToRightThigh;
    _trRightLegToRightThigh.setIdentity();
    _trRightLegToRightThigh.setOrigin( { 0., 0., -0.7 - 0. + 0.225 } ); // geom has the same position as body

    auto _rightLeg = _simbody->setupLinkSingleJoint( _currentLinkIndx,
                                                     "capsule",
                                                     { 0.04, 0.5, 0.04 },
                                                     _trRightLegToRightThigh,
                                                     _rightThigh,
                                                     "revolute",
                                                     { 0., -1., 0. },
                                                     { 0., 0., 0.25 - (-0.) },
                                                     -SIMD_PI * 5. / 6., 0. );

    // next link please
    _currentLinkIndx += 1;

    // define the transform of the right-foot w.r.t. the right-leg
    btTransform _trRightFootToRightLeg;
    _trRightFootToRightLeg.setIdentity();
    _trRightFootToRightLeg.setOrigin( { 0.06, 0., -0.25 - 0. + 0. } ); // geom has the same position as body

    auto _rightFoot = _simbody->setupLinkSingleJoint( _currentLinkIndx,
                                                      "capsulex",
                                                      { 0.05, 0.2, 0.05 },
                                                      _trRightFootToRightLeg,
                                                      _rightLeg,
                                                      "revolute",
                                                      { 0., -1., 0. },
                                                      { -0.06 - 0., 0., 0. },
                                                      -SIMD_PI / 4., SIMD_PI / 4. );

    // next link please
    _currentLinkIndx += 1;

    // define the transform of the left-thigh w.r.t. the torso
    btTransform _trLeftThighToTorso;
    _trLeftThighToTorso.setIdentity();
    _trLeftThighToTorso.setOrigin( { 0., 0.05, -0.3 - 0.225 } ); // geom is slightly to -z of the body

    auto _leftThigh = _simbody->setupLinkSingleJoint( _currentLinkIndx,
                                                      "capsule",
                                                      { 0.05, 0.45, 0.05 },
                                                      _trLeftThighToTorso,
                                                      _torsoLinks.back(),
                                                      "revolute",
                                                      { 0., -1., 0. },
                                                      { 0., 0., -(-0.225) },
                                                      -SIMD_PI / 9., SIMD_PI * 5. / 9. );    

    // next link please
    _currentLinkIndx += 1;

    // define the transform of the left-leg w.r.t. the lef-thigh
    btTransform _trLeftLegToLeftThigh;
    _trLeftLegToLeftThigh.setIdentity();
    _trLeftLegToLeftThigh.setOrigin( { 0., 0., -0.7 - 0. + 0.225 } ); // geom has the same position as body

    auto _leftLeg = _simbody->setupLinkSingleJoint( _currentLinkIndx,
                                                    "capsule",
                                                    { 0.04, 0.5, 0.04 },
                                                    _trLeftLegToLeftThigh,
                                                    _leftThigh,
                                                    "revolute",
                                                    { 0., -1., 0. },
                                                    { 0., 0., 0.25 - (-0.) },
                                                    -SIMD_PI * 5. / 6., 0. );

    // next link please
    _currentLinkIndx += 1;

    // define the transform of the left-foot w.r.t. the left-leg
    btTransform _trLeftFootToLeftLeg;
    _trLeftFootToLeftLeg.setIdentity();
    _trLeftFootToLeftLeg.setOrigin( { 0.06, 0., -0.25 - 0. + 0. } ); // geom has the same position as body

    auto _leftFoot = _simbody->setupLinkSingleJoint( _currentLinkIndx,
                                                     "capsulex",
                                                     { 0.05, 0.2, 0.05 },
                                                     _trLeftFootToLeftLeg,
                                                     _leftLeg,
                                                     "revolute",
                                                     { 0., -1., 0. },
                                                     { -0.06 - 0., 0., 0. },
                                                     -SIMD_PI / 4., SIMD_PI / 4. );

    _simbody->ptrBtMultibody()->setHasSelfCollision( true );
    _simbody->ptrBtMultibody()->finalizeMultiDof();

    return _simbody;
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