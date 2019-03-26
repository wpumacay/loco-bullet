
// Includes from core functionality
#include <runtime.h>
// Includes from bullet functionality
#include <bullet_config.h>

int main()
{

    auto _bbox = new tysoc::sandbox::TBody();
    _bbox->name = "bbox";
    _bbox->type = "box";
    _bbox->mass = 0.2;
    _bbox->size = { 0.2, 0.2, 1.0 };
    _bbox->worldTransform.setPosition( { 1.0, 1.0, 2.0 } );
    _bbox->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 1.57, 1.0, 1.0 } ) );

    auto _jball = new tysoc::sandbox::TJoint();
    _jball->name = "jhinge";
    _jball->type = "ball";
    _jball->limits = { -0.5 * TYSOC_PI, 0.5 * TYSOC_PI };
    _jball->parentBodyPtr = _bbox;
    _jball->relTransform.setPosition( { 0.0, 0.0, 0.5 } );

    _bbox->joints.push_back( _jball );

    auto _bplane = new tysoc::sandbox::TBody();
    _bplane->name = "bplane";
    _bplane->type = "plane";
    _bplane->size = { 10.0, 10.0, 0.1 };
    _bplane->color = { 0.3, 0.3, 0.3 };
    _bplane->worldTransform.setPosition( { 0, 0, 0 } );

    auto _scenario = new tysoc::TScenario();
    _scenario->addBody( _bbox );
    _scenario->addBody( _bplane );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::BULLET,
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    bool _running = false;

    while ( _visualizer->isActive() )
    {
        if ( _visualizer->checkSingleKeyPress( 15 ) )
            _running = ( _running ) ? false : true;

        if ( _running )
            _simulation->step();

        _visualizer->update();
    }

    return 0;
}