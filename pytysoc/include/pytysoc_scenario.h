
#pragma once

#include <scenario.h>
#include <pytysoc_common.h>

namespace py = pybind11;

namespace pytysoc
{

    class PyScenario
    {

        private :

        tysoc::TScenario* m_scenarioPtr;

        public :

        
        PyScenario();
        ~PyScenario();

        void addAgent();
    };

}