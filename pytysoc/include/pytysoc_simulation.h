
#pragma once

#include <pytysoc_common.h>
#include <pytysoc_scenario.h>
#include <simulation_base.h>

namespace py = pybind11;

namespace pytysoc
{

    class PySimulation
    {
        private :

        tysoc::TISimulation* m_simulationPtr;

        public :

        /**
        *   Creates a wrapper for a requested simulation. Objects from this ...
        *   class can only be created through the runtime (the same for the viz.) ...
        *   in the same way TISimulation objects are created in C/C++.
        *   
        *   @param pyScenarioPtr    The scenario we want to use to instantiate a simulation
        *   @notexposed             Not exposed through python API
        */
        PySimulation( tysoc::TISimulation* simulationPtr );

        /**
        *   Destructor for this wrapper. It just cleans some references, as the ...
        *   owner of the data is the creator, which in our case is the runtime.
        */
        ~PySimulation();

        /**
        *   Requests the initialization of the wrapped simulation object, and ...
        *   returns whether or not the initialization was successful
        *
        *   @exposed    Exposed through python API
        */
        bool initialize();

        /**
        *   Requests an update-step of the wrapped simulation object.
        *
        *   @exposed    Exposed through python API
        */
        void step();

        /**
        *   Requests a reset of the wrapped simulation object.
        *
        *   @exposed    Exposed through python API
        */
        void reset();

        // @TODO: should check deletion when using references
        // /**
        // *   Gets the python-wrapper for the scenario being simulated
        // *   
        // *   @exposed    Exposed through python API
        // */
        // PyScenario* scenario();

        /**
        *   Gets the type of backend being used
        *   
        *   @exposed    Exposed through python API
        */
        std::string type();
    };

}

#define PYTYSOC_SIMULATION_BINDINGS(m) \
    py::class_<pytysoc::PySimulation>( m, "PySimulation" ) \
        .def( "initialize", &pytysoc::PySimulation::initialize ) \
        .def( "step", &pytysoc::PySimulation::step ) \
        .def( "reset", &pytysoc::PySimulation::reset ) \
        .def( "type", &pytysoc::PySimulation::type );
