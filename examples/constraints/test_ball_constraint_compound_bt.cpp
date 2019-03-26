
// Includes from core functionality
#include <runtime.h>
// Includes from bullet functionality
#include <bullet_config.h>

int main()
{

    auto _bbox1 = new tysoc::sandbox::TBody();
    _bbox1->name = "bbox1";
    _bbox1->type = "box";
    _bbox1->mass = 1.0;
    _bbox1->size = { 0.2, 0.2, 1.0 };
    _bbox1->worldTransform.setPosition( { 1.0, 1.0, 3.0 } );
    _bbox1->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 1.57, 1.0, 0.5 } ) );

    auto _jball1 = new tysoc::sandbox::TJoint();
    _jball1->name = "jball1";
    _jball1->type = "ball";
    _jball1->parentBodyPtr = _bbox1;
     _bbox1->joints.push_back( _jball1 );
    _jball1->relTransform.setPosition( { 0.0, 0.0, 0.5 } );

    auto _bbox2 = new tysoc::sandbox::TBody();
    _bbox2->name = "bbox2";
    _bbox2->type = "box";
    _bbox2->mass = 1.0;
    _bbox2->size = { 0.2, 0.2, 1.0 };
    _bbox2->color = { 0.75, 0.5, 0.25 };
    _bbox2->parentBodyPtr = _bbox1;
    _bbox1->bodies.push_back( _bbox2 );
    _bbox2->relTransform.setPosition( { 0.0, 0.0, -1.0 } );

    auto _jball2 = new tysoc::sandbox::TJoint();
    _jball2->name = "jball2";
    _jball2->type = "ball";
    _jball2->parentBodyPtr = _bbox2;
    _bbox2->joints.push_back( _jball2 );
    _jball2->relTransform.setPosition( { 0.0, 0.0, 0.5 } );

    auto _bplane = new tysoc::sandbox::TBody();
    _bplane->name = "bplane";
    _bplane->type = "plane";
    _bplane->size = { 10.0, 10.0, 0.1 };
    _bplane->color = { 0.3, 0.3, 0.3 };
    _bplane->worldTransform.setPosition( { 0, 0, 0 } );

    auto _scenario = new tysoc::TScenario();
    _scenario->addBody( _bbox1 );
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