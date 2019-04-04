

// Includes from core functionality
#include <runtime.h>
// Includes from bullet functionality
#include <bullet_config.h>

tysoc::sandbox::TBody* createBody( const std::string& name,
                                   const tysoc::TVec3& position,
                                   const tysoc::TVec3& size,
                                   const tysoc::TVec3& color )
{
    auto _bbox = new tysoc::sandbox::TFreeBody();
    _bbox->name = name;
    _bbox->type = "capsule";
    _bbox->mass = 0.1;
    _bbox->size = size;
    _bbox->color = color;
    _bbox->friction = { 1.0, 1.0, 1.0 };
    _bbox->worldTransform.setPosition( position );

    return _bbox;
}

int main()
{
    auto _bplane = new tysoc::sandbox::TBody();
    _bplane->name = "bplane";
    _bplane->type = "box";
    _bplane->friction = { 1.0, 1.0, 1.0 };
    _bplane->size = { 10.0, 10.0, 1.0 };
    _bplane->color = { 0.3, 0.3, 0.3 };
    _bplane->worldTransform.setPosition( { 0, 0, -0.5 } );

    auto _scenario = new tysoc::TScenario();
    _scenario->addBody( _bplane );

    for ( size_t k = 0; k < 5; k++ )
    {
        for ( size_t i = 0; i < 5; i++ )
        {
            for ( size_t j = 0; j < 5; j++ )
            {
                auto _name = std::string( "bbox" ) + "_" + std::to_string( i ) + "_" + std::to_string( j ) + "_" + std::to_string( k );
                auto _body = createBody( _name,
                                         { 0.2f * i, 0.2f * j, 2 + 0.2f * k },
                                         { 0.1, 0.4, 0.2 },
                                         { 0.25f + 0.75f * ( i / 4.0f ),
                                           0.25f + 0.75f * ( j / 4.0f ),
                                           0.25f + 0.75f * ( k / 4.0f ) } );

                _scenario->addBody( _body );
            }
        }
    }

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::BULLET,
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    while ( _visualizer->isActive() )
    {
        if ( _visualizer->checkSingleKeyPress( 15 ) )
            _simulation->togglePause();

        _simulation->step();

        _visualizer->update();
    }

    return 0;
}