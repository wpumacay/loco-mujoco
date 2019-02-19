
#pragma once

#include <runtime.h>

#include <pytysoc_common.h>
#include <pytysoc_agent_core.h>

namespace py = pybind11;


namespace pytysoc
{

    class PyRuntime
    {

        private :

        tysoc::TRuntime* m_runtimePtr;

        public :

        // Python exposed API ********************************

        PyRuntime( const std::string& dlpathSim,
                   const std::string& dlpathViz );
        ~PyRuntime();

        void createSimulation();
        void createAgentWrapper();
        // void createTerrainGenWrapper();
        // void createSensor();





        // ***************************************************

    };

}