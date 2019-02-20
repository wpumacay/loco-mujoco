
#include <pytysoc_scenario.h>

namespace py = pybind11;

namespace pytysoc
{

    PyScenario::PyScenario()
    {
        m_scenarioPtr = new tysoc::TScenario();
    }

    PyScenario::~PyScenario()
    {
        m_pyCoreAgentsMap.clear();

        //for ( size_t q = 0; q < m_pyCoreAgents.size(); q++ )
        //{
        //    delete m_pyCoreAgents[q];
        //    m_pyCoreAgents[q] = NULL;
        //}
        m_pyCoreAgents.clear();

        if ( m_scenarioPtr )
        {
            delete m_scenarioPtr;
            m_scenarioPtr = NULL;
        }
    }

    void PyScenario::addAgent( PyCoreAgent* pyCoreAgentPtr )
    {
        if ( !m_scenarioPtr )
            return;

        if ( pyCoreAgentPtr->name() == "undefined" )
        {
            std::cout << "ERROR> tried to add a pyCoreAgent with no wrapped kintree agent" << std::endl;
            return;
        }

        if ( m_pyCoreAgentsMap.find( pyCoreAgentPtr->name() ) != m_pyCoreAgentsMap.end() )
        {
            std::cout << "WARNING> tried to add an existing (same name) pyCoreAgent" << std::endl;
            return;
        }

        m_scenarioPtr->addAgent( pyCoreAgentPtr->ptr() );

        m_pyCoreAgents.push_back( pyCoreAgentPtr );
        m_pyCoreAgentsMap[ pyCoreAgentPtr->name() ] = pyCoreAgentPtr;
    }

    PyCoreAgent* PyScenario::getAgentByName( const std::string& name )
    {
        if ( m_pyCoreAgentsMap.find( name ) == m_pyCoreAgentsMap.end() )
        {
            std::cout << "WARNING> agent: " << name << " not found in scenario" << std::endl;
            return nullptr;
        }

        return m_pyCoreAgentsMap[ name ];
    }

    std::vector< PyCoreAgent* > PyScenario::getAgents()
    {
        return m_pyCoreAgents;
    }

    std::map< std::string, PyCoreAgent* > PyScenario::getAgentsMap()
    {
        return m_pyCoreAgentsMap;
    }

    tysoc::TScenario* PyScenario::ptr()
    {
        return m_scenarioPtr;
    }

}