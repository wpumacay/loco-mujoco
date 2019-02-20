
#pragma once

#include <pytysoc_common.h>
#include <pytysoc_scenario.h>
#include <viz/viz.h>

namespace py = pybind11;

namespace pytysoc
{

    class PyVisualizer
    {

        private :

        tysoc::viz::TIVisualizer* m_visualizerPtr;

        public :

        /**
        *   Creates a wrapper for a requested visualizer. Objects from this ...
        *   class can only be created through the runtime (the same for the sim.) ...
        *   in the same way TIVisualizer objects are created in C/C++.
        *
        *   @param pyScenarioPtr    The scenario we want to visualize
        *   @notexposed             Not exposed through python API
        */
        PyVisualizer( tysoc::viz::TIVisualizer* visualizerPtr );

        /**
        *   Destructor for this wrapper. It just cleans some references, as the ...
        *   owner of the data is the creator, which in our case is the runtime.
        */
        ~PyVisualizer();

        /**
        *   Requests the initialization of the wrapped visualizer object, and ...
        *   returns whether or not the initialization was successful
        *
        *   @exposed    Exposed through python API
        */
        bool initialize();

        /**
        *   Requests the wrapped visualizer to render the scene
        *
        *   @exposed    Exposed through python API
        */
        void render();

        /**
        *   Checks if the visualizer should close
        *
        *   @exposed    Exposed through python API
        */
        bool isActive();

        /**
        *   Checks if the given key is currently being pressed
        *   
        *   @param keyCode  Int representing the key the user wants to query
        *   @exposed        Exposed through python API
        */
        bool isKeyDown( int keyCode );

        /**
        *   Checks if a given key has been pressed, and clears immediately ...
        *   the key state to listen for just a single press (like a latch of a switch)
        *
        *   @param keyCode  Int representing the key the user wants to query
        *   @exposed        Exposed through python API
        */
        bool checkSingleKeyPress( int keyCode );

        /**
        *   Get the type of rendering backend used for the visualizer
        *
        *   @exposed        Exposed through python API
        */
        std::string type();
    };


}

#define PYTYSOC_VISUALIZER_BINDINGS(m) \
    py::class_<pytysoc::PyVisualizer>( m, "PyVisualizer" ) \
        .def( "initialize", &pytysoc::PyVisualizer::initialize ) \
        .def( "render", &pytysoc::PyVisualizer::render ) \
        .def( "isActive", &pytysoc::PyVisualizer::isActive ) \
        .def( "isKeyDown", &pytysoc::PyVisualizer::isKeyDown ) \
        .def( "checkSingleKeyPress", &pytysoc::PyVisualizer::checkSingleKeyPress ) \
        .def( "type", &pytysoc::PyVisualizer::type );
