
#pragma once

#include <scenario.h>
#include <pytysoc_common.h>
#include <pytysoc_agent_core.h>

namespace py = pybind11;

namespace pytysoc
{

    class PyScenario
    {

        private :

        tysoc::TScenario* m_scenarioPtr;

        std::vector< PyCoreAgent* > m_pyCoreAgents;
        std::map< std::string, PyCoreAgent* > m_pyCoreAgentsMap;

        public :
        
        /**
        *   Creates a wrapper for a tysoc::TScenario object. This wrapper ...
        *   is intended to be used along other wrappers (python bindings) ...
        *   like PyCoreAgent, PyCoreTerrainGen, and PyCoreSensor to assemble ...
        *   a scenario in a similar way to the C/C++ way of constructing a ...
        *   scenario from scratch, and then use it when instantiating a runtime.
        *
        *   @exposed    Exposed through python API
        */
        PyScenario();

        /**
        *   Destructor for this wrapper. In a similar way to the C/C++ API ...
        *   this scenario wrapper is intended to have ownership of the agents, ...
        *   terraingens and sensors, so we delete these resources when deleting ...
        *   this scenario.
        */
        ~PyScenario();

        /**
        *   Adds a PyCoreAgent wrapper to this scenario wrapper
        *
        *   @param pyCoreAgentPtr   A pointer to the PyCoreAgent wrapper to add to the scenario
        *   @exposed    Exposed through python API
        */
        void addAgent( PyCoreAgent* pyCoreAgentPtr );


        /**
        *   Gets the PyCoreAgent wrapper of an agent with a given name
        *   
        *   @param name     String representing the name of the wrapped agent the user requests
        *   @exposed        Exposed through python API
        */
        PyCoreAgent* getAgentByName( const std::string& name );

        /**
        *   Gets a list of all agents belonging to this scenario wrapper
        *
        *   @exposed    Exposed through python API
        */
        std::vector< PyCoreAgent* > getAgents();

        /**
        *   Gets a dictionary of all agents belonging to this scenario wrapper
        *
        *   @exposed    Exposed through python API
        */
        std::map< std::string, PyCoreAgent* > getAgentsMap();

        /**
        *   Gets the wrapped scenario object (used internally by runtime to ...
        *   grab the wrapped scenario object)
        *
        *   @notexposed     Not exposed through python API
        */
        tysoc::TScenario* ptr();
    };

}

//@CHECK: does the func. getAgents return a vec. with pure disowned references?
//@CHECK: does the func. getAgentsMap return a dict. with pure disowned references?

#define PYTYSOC_SCENARIO_BINDINGS(m) \
    py::class_<pytysoc::PyScenario>( m, "PyScenario" ) \
        .def( py::init<>() ) \
        .def( "addAgent", &pytysoc::PyScenario::addAgent ) \
        .def( "getAgentByName", &pytysoc::PyScenario::getAgentByName, py::return_value_policy::automatic ) \
        .def( "getAgents", &pytysoc::PyScenario::getAgents ) \
        .def( "getAgentsMap", &pytysoc::PyScenario::getAgentsMap );
