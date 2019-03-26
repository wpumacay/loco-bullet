
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
    _bbox1->worldTransform.setPosition( { 0.0, 0.0, 3.0 } );
    // _bbox1->worldTransform.setRotation( tysoc::TMat3::fromEuler( { 1.57, 0.0, 0.0 } ) );
    _bbox1->vel = { 2.5, 2.5, 2.5 };

    auto _jdof1 = new tysoc::sandbox::TJoint();
    _jdof1->name = "jdof1";
    _jdof1->type = "slide";
    _jdof1->axis = { 1, 0, 0 };
    _jdof1->limits = { 1.0, -1.0 };
    _jdof1->parentBodyPtr = _bbox1;

    auto _jdof2 = new tysoc::sandbox::TJoint();
    _jdof2->name = "jdof2";
    _jdof2->type = "slide";
    _jdof2->axis = { 0, 1, 0 };
    _jdof2->limits = { 1.0, -1.0 };
    _jdof2->parentBodyPtr = _bbox1;

    auto _jdof3 = new tysoc::sandbox::TJoint();
    _jdof3->name = "jdof3";
    _jdof3->type = "slide";
    _jdof3->axis = { 0, 0, 1 };
    _jdof3->limits = { 1.0, -1.0 };
    _jdof3->parentBodyPtr = _bbox1;

    _bbox1->joints.push_back( _jdof1 );
    _bbox1->joints.push_back( _jdof2 );
    _bbox1->joints.push_back( _jdof3 );

    auto _bbox2 = new tysoc::sandbox::TBody();
    _bbox2->name = "bbox2";
    _bbox2->type = "box";
    _bbox2->mass = 1.0;
    _bbox2->size = { 0.2, 0.2, 1.0 };
    _bbox2->color = { 0.75, 0.5, 0.25 };
    _bbox2->parentBodyPtr = _bbox1;
    _bbox1->bodies.push_back( _bbox2 );
    _bbox2->relTransform.setPosition( { 0.0, 0.0, -1.0 } );

    auto _jhinge2 = new tysoc::sandbox::TJoint();
    _jhinge2->name = "jhinge2";
    _jhinge2->type = "hinge";
    _jhinge2->axis = { 1, 0, 0 };
    _jhinge2->limits = { -0.5 * TYSOC_PI, 0.5 * TYSOC_PI };
    _jhinge2->parentBodyPtr = _bbox2;
    _bbox2->joints.push_back( _jhinge2 );
    _jhinge2->relTransform.setPosition( { 0.0, 0.0, 0.5 } );

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