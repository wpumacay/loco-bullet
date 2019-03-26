
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
    _bbox->size = { 0.2, 0.2, 0.2 };
    _bbox->worldTransform.setPosition( { 0.0, 0.0, 2.0 } );
    _bbox->vel = { 3.0, 3.0, 3.0 };

    auto _jdof1 = new tysoc::sandbox::TJoint();
    _jdof1->name = "jdof1";
    _jdof1->type = "slide";
    _jdof1->axis = { 1, 0, 0 };
    _jdof1->limits = { 1.0, -1.0 };
    _jdof1->parentBodyPtr = _bbox;

    auto _jdof2 = new tysoc::sandbox::TJoint();
    _jdof2->name = "jdof2";
    _jdof2->type = "slide";
    _jdof2->axis = { 0, 1, 0 };
    _jdof2->limits = { 1.0, -1.0 };
    _jdof2->parentBodyPtr = _bbox;

    auto _jdof3 = new tysoc::sandbox::TJoint();
    _jdof3->name = "jdof3";
    _jdof3->type = "slide";
    _jdof3->axis = { 0, 0, 1 };
    _jdof3->limits = { 1.0, -1.0 };
    _jdof3->parentBodyPtr = _bbox;

    auto _jdof4 = new tysoc::sandbox::TJoint();
    _jdof4->name = "jdof4";
    _jdof4->type = "hinge";
    _jdof4->axis = { 1, 0, 0 };
    _jdof4->limits = { 1.0, -1.0 };
    _jdof4->parentBodyPtr = _bbox;

    auto _jdof5 = new tysoc::sandbox::TJoint();
    _jdof5->name = "jdof5";
    _jdof5->type = "hinge";
    _jdof5->axis = { 0, 1, 0 };
    _jdof5->limits = { 1.0, -1.0 };
    _jdof5->parentBodyPtr = _bbox;

    auto _jdof6 = new tysoc::sandbox::TJoint();
    _jdof6->name = "jdof6";
    _jdof6->type = "hinge";
    _jdof6->axis = { 0, 0, 1 };
    _jdof6->limits = { 1.0, -1.0 };
    _jdof6->parentBodyPtr = _bbox;

    _bbox->joints.push_back( _jdof1 );
    // _bbox->joints.push_back( _jdof2 );
    _bbox->joints.push_back( _jdof3 );

    // _bbox->joints.push_back( _jdof4 );
    // _bbox->joints.push_back( _jdof5 );
    // _bbox->joints.push_back( _jdof6 );

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