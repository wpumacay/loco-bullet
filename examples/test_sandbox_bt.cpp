
// Includes from core functionality
#include <runtime.h>
// Includes from bullet functionality
#include <bullet_config.h>

int main()
{

    auto _bbox = new tysoc::sandbox::TFreeBody();
    _bbox->name = "bbox";
    _bbox->type = "box";
    _bbox->mass = 0.1;
    _bbox->size = { 0.2, 0.2, 0.4 };
    _bbox->worldTransform.setPosition( { 1.0, 1.0, 1.0 } );
    _bbox->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _bsphere = new tysoc::sandbox::TFreeBody();
    _bsphere->name = "bsphere";
    _bsphere->type = "sphere";
    _bsphere->mass = 0.1;
    _bsphere->size = { 0.2, 0.2, 0.2 };
    _bsphere->worldTransform.setPosition( { 1.0, -1.0, 1.0 } );
    _bsphere->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _bcapsule = new tysoc::sandbox::TFreeBody();
    _bcapsule->name = "bcapsule";
    _bcapsule->type = "capsule";
    _bcapsule->mass = 0.1;
    _bcapsule->size = { 0.1, 0.4, 0.0 };
    _bcapsule->worldTransform.setPosition( { -1.0, 1.0, 1.0 } );
    _bcapsule->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _bcylinder = new tysoc::sandbox::TFreeBody();
    _bcylinder->name = "bcylinder";
    _bcylinder->type = "cylinder";
    _bcylinder->mass = 0.1;
    _bcylinder->size = { 0.1, 0.4, 0.0 };
    _bcylinder->worldTransform.setPosition( { -1.0, -1.0, 1.0 } );
    _bcylinder->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 0.4, 0.6, 0.8 } ) );

    auto _bplane = new tysoc::sandbox::TFreeBody();
    _bplane->name = "bplane";
    _bplane->type = "plane";
    _bplane->size = { 10.0, 10.0, 0.1 };
    _bplane->color = { 0.3, 0.3, 0.3 };
    _bplane->worldTransform.setPosition( { 0, 0, 0 } );

    auto _scenario = new tysoc::TScenario();
    _scenario->addBody( _bbox );
    _scenario->addBody( _bsphere );
    _scenario->addBody( _bcapsule );
    _scenario->addBody( _bcylinder );
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