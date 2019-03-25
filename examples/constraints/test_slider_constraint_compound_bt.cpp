
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
    _bbox1->size = { 0.2, 0.2, 0.2 };
    _bbox1->worldTransform.setPosition( { 0.0, 0.0, 4.0 } );

    auto _jslide1 = new tysoc::sandbox::TJoint();
    _jslide1->name = "jslide1";
    _jslide1->type = "slide";
    _jslide1->axis = { 0, 0, 1 };
    _jslide1->limits = { -1, 1 };
    _jslide1->parentBodyPtr = _bbox1;
    _bbox1->joints.push_back( _jslide1 );
    _jslide1->relTransform.setPosition( { 0.0, 0.0, 0.0 } );

    auto _bbox2 = new tysoc::sandbox::TBody();
    _bbox2->name = "bbox2";
    _bbox2->type = "box";
    _bbox2->mass = 1.0;
    _bbox2->size = { 0.2, 0.2, 0.2 };
    _bbox2->color = { 0.75, 0.5, 0.25 };
    _bbox2->parentBodyPtr = _bbox1;
    _bbox1->bodies.push_back( _bbox2 );
    _bbox2->relTransform.setPosition( { 0.0, 0.0, -2.0 } );

    auto _jslide2 = new tysoc::sandbox::TJoint();
    _jslide2->name = "jslide2";
    _jslide2->type = "slide";
    _jslide2->axis = { 0, 0, 1 };
    _jslide2->limits = { 0, 0.5 };
    _jslide2->parentBodyPtr = _bbox2;
    _bbox2->joints.push_back( _jslide2 );
    _jslide2->relTransform.setPosition( { 0.0, 0.0, 0.0 } );

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