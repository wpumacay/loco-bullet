
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
    _bbox->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _jhinge = new tysoc::sandbox::TJoint();
    _jhinge->name = "jhinge";
    _jhinge->type = "hinge";
    _jhinge->axis = { 1, 0, 0 };
    _jhinge->limits = { -180, 180 };
    _jhinge->parentBodyPtr = _bbox;
    _jhinge->relTransform.setPosition( { 0.0, 0.0, 0.5 } );

    _bbox->joints.push_back( _jhinge );

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

    while ( _visualizer->isActive() )
    {
        _simulation->step();

        _visualizer->update();
    }

    return 0;
}