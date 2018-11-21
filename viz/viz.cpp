
#include <viz.h>



namespace viz
{

    Visualizer::Visualizer()
    {
        auto _app = engine::LApp::GetInstance();
        auto _scene = _app->scene();

        auto _camera = new engine::LFixedCamera3d( "fixed",
                                                engine::LVec3( 2.0f, 4.0f, 2.0f ),
                                                engine::LVec3( 0.0f, 0.0f, 0.0f ),
                                                engine::LICamera::UP_Z );
        // make a sample light source
        auto _light = new engine::LLightDirectional( engine::LVec3( 0.2, 0.2, 0.2 ), 
                                                    engine::LVec3( 0.8, 0.8, 0.8 ),
                                                    engine::LVec3( 0.15, 0.15, 0.15 ), 
                                                    0, 
                                                    engine::LVec3( -1, -1, -1 ) );
        _light->setVirtualPosition( engine::LVec3( 5, 0, 5 ) );

        // add these components to the scene
        _scene->addCamera( _camera );
        _scene->addLight( _light );
    }

    Visualizer::~Visualizer()
    {
        // @TODO: Check if should delete meshes in scene or here
    }

    void Visualizer::initialize()
    {
        // Collect meshes from terrain and agent

    }

    void Visualizer::update()
    {
        // Update (and collect if necessary) from terrain and agentmiyu
        

    }

}