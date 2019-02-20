
#pragma once

#include <runtime.h>

#include <pytysoc_common.h>
#include <pytysoc_scenario.h>
#include <pytysoc_simulation.h>
#include <pytysoc_visualizer.h>

namespace py = pybind11;


namespace pytysoc
{

    class PyRuntime
    {

        private :

        tysoc::TRuntime* m_runtimePtr;

        public :

        /**
        *   Creates a wrapper for a requested runtime type, given by the parameters ...
        *   passed for the physics backend and rendering backend.
        *
        *   @param dlpathSim    String representing the physics backend to use
        *   @param dlpathViz    String representing the rendering backend to use
        *   @exposed            Exposed through python API
        */
        PyRuntime( const std::string& dlpathSim,
                   const std::string& dlpathViz );

        /**
        *   Destructor for this wrapper. It deletes the wrapped runtime, as there ...
        *   should not be stored references to the internal runtime ptr anywhere else.
        */
        ~PyRuntime();

        /**
        *   Creates a PySimulation object that wraps a simulation created for the given backend
        *
        *   @exposed    Exposed through python API
        */
        PySimulation* createSimulation( PyScenario* pyScenarioPtr );

        /**
        *   Creates a PyVisualizer object that wraps a visualizer created for the given backend
        *
        *   @exposed    Exposed through python API
        */
        PyVisualizer* createVisualizer( PyScenario* pyScenarioPtr );



        // void createAgentWrapper();
        // void createTerrainGenWrapper();
        // void createSensor();


    };

}

#define PYTYSOC_RUNTIME_BINDINGS(m) \
    py::class_<pytysoc::PyRuntime>( m, "PyRuntime" ) \
        .def( py::init<const std::string&, const std::string& >() ) \
        .def( "createSimulation", &pytysoc::PyRuntime::createSimulation, py::return_value_policy::reference ) \
        .def( "createVisualizer", &pytysoc::PyRuntime::createVisualizer, py::return_value_policy::reference );
