
#pragma once

#include <pytysoc_common.h>
#include <model_loader.h>
#include <agent/types/agent_kintree.h>
#include <agent/types/agent_kintree_mjcf.h>
#include <agent/types/agent_kintree_urdf.h>
#include <agent/types/agent_kintree_rlsim.h>

namespace py = pybind11;

namespace pytysoc
{

    class PyCoreAgent
    {

        private :

        tysoc::agent::TAgentKinTree* m_agentKinTreePtr;

        public :

        /**
        *   Creates a python wrapper for abstract kintree-agent. The wrapped kintree ...
        *   agent is created internally from a given model filename and format passed ...
        *   as parameters to this constructor.
        *
        *   @param name             String representing the name of the agent
        *   @param position         Numpy array representing the position of the agent
        *   @param modelFormat      String representing the format of the model(mjcf|urdf|rlsim)
        *   @param modelName        String representing the filename of the model (or the id if cached)
        *   @exposed                Exposed through python API
        */
        PyCoreAgent( const std::string& name,
                     py::array_t<TScalar>& position,
                     const std::string& modelFormat,
                     const std::string& modelName );

        /**
        *   Creates a python wrapper for an already created abstract kintree-agent.
        *   This wrapper is meant to be used when the runtime itself creates a ...
        *   whole task and allocates the kintree agent for us. We just have to ...
        *   wrap the agent then.
        *
        *   @param kinTreeAgentPtr  Abstract kintree agent to wrap
        *   @notexposed             Not exposed through python API.
        */
        PyCoreAgent( tysoc::agent::TAgentKinTree* kinTreeAgentPtr );

        /**
        *   Destructor for this wrapper. It only NULLs the references wrapped.
        *   The actual deletion of the wrapped resources should be made by ...
        *   the scenario, which has control over all kintrees, terraingens and sensors.
        */
        ~PyCoreAgent();

        /**
        *   Sets the actuator controls of the agent using a given array
        *
        *   @param actions  Numpy array with the actions to send to the actuators
        *   @exposed        Exposed through python API
        */
        void setActions( py::array_t<TScalar>& actions );

        /**
        *   Gets the number of actuators of this agent
        *
        *   @exposed    Exposed through python API
        */
        int getActionDim();

        /**
        *   Gets the position of the agent's root body as a numpy array
        *   
        *   @exposed    Exposed through python API
        */
        py::array_t<TScalar> getPosition();

        /**
        *   Gets the name of the wrapped agent
        *
        *   @exposed    Exposed through python API
        */
        std::string name();

        /**
        *   Gets the wrapped kintree core agent
        *
        *   @notexposed     Not exposed through python API
        */
        tysoc::agent::TAgentKinTree* ptr();

    };

}

#define DOCSTRING_CORE_AGENT "PyCoreAgent(name, position, modelFormat, modelName)\n\r \n\r \
    Creates a wrapper for a kintree core agent. It creates one from the modelFormat \n\r \
    and modelName given. \n\r \n\r \
    Parameters \n\r \
    ---------- \n\r \
    name : str \n\r \
    position : vec3 array (numpy) \n\r \
    modelFormat : str \n\r \
    modelName : str"


#define PYTYSOC_CORE_AGENT_BINDINGS(m) \
    py::class_<pytysoc::PyCoreAgent>( m, "PyCoreAgent", DOCSTRING_CORE_AGENT ) \
        .def( py::init<const std::string&, py::array_t<TScalar>&, const std::string&, const std::string& >() ) \
        .def( "setActions", &pytysoc::PyCoreAgent::setActions ) \
        .def( "getActionDim", &pytysoc::PyCoreAgent::getActionDim ) \
        .def( "getPosition", &pytysoc::PyCoreAgent::getPosition ) \
        .def( "name", &pytysoc::PyCoreAgent::name );
