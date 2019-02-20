
#include <pytysoc_simulation.h>

namespace py = pybind11;

namespace pytysoc
{

    PySimulation::PySimulation( tysoc::TISimulation* simulationPtr )
    {
        m_simulationPtr = simulationPtr;
    }

    PySimulation::~PySimulation()
    {
        m_simulationPtr = NULL;
    }

    bool PySimulation::initialize()
    {
        if ( m_simulationPtr )
            return m_simulationPtr->initialize();

        std::cout << "ERROR> could not initialize non-existent "
                  << "wrapped simulation object" << std::endl;
        return false;
    }

    void PySimulation::step()
    {
        if ( m_simulationPtr )
        {
            m_simulationPtr->step();
        }
        else
        {
            std::cout << "ERROR> could not make a simulation step on "
                      << "a non-existent wrapped simulation object" << std::endl;
        }
    }

    void PySimulation::reset()
    {
        if ( m_simulationPtr )
        {
            m_simulationPtr->reset();
        }
        else
        {
            std::cout << "ERROR> could not reset non-existent "
                      << "wrapped simulation object" << std::endl;
        }
    }

//    PyScenario* PySimulation::scenario()
//    {
//        if ( m_simulationPtr )
//        {
//            auto _scenarioPtr = m_simulationPtr->scenario();
//            if ( _scenarioPtr )
//                return new PyScenario( _scenarioPtr );
//        }
//
//        return nullptr;
//    }

    std::string PySimulation::type()
    {
        if ( m_simulationPtr )
            return m_simulationPtr->type();

        std::cout << "ERROR> could not query type of a non-existent "
                  << "wrapped simulation object" << std::endl;
        return "undefined";
    }

}