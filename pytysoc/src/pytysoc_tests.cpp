
#include <pytysoc_tests.h>

namespace pytysoc
{

    PyDummy::PyDummy( const std::string& name )
    {
        m_name = name;
        std::cout << "LOG> created dummy: " << name << std::endl;
    }

    PyDummy::~PyDummy()
    {
        std::cout << "LOG> destroyed dummy: " << m_name << std::endl;
        m_name = "undefined";
    }

    std::string PyDummy::name()
    {
        return m_name;
    }



    PyDummyContainer::PyDummyContainer()
    {
        std::cout << "LOG> created dummy container" << std::endl;
    }

    PyDummyContainer::~PyDummyContainer()
    {
        // for ( size_t q = 0; q < m_dummies.size(); q++ )
        // {
        //     delete m_dummies[q];
        // }
        m_dummies.clear();

        std::cout << "LOG> destroyed dummy container" << std::endl;
    }

    void PyDummyContainer::addDummy( PyDummy* pyDummyPtr )
    {
        m_dummies.push_back( pyDummyPtr );
    }

    std::vector< PyDummy* > PyDummyContainer::getDummies()
    {
        return m_dummies;
    }

    PyDummy* PyDummyContainer::front()
    {
        return m_dummies.front();
    }

}