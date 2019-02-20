
#pragma once

#include <pytysoc_common.h>

namespace pytysoc
{


    class PyDummy
    {

        private :

        std::string m_name;

        public :

        PyDummy( const std::string& name );
        ~PyDummy();

        std::string name();

    };

    class PyDummyContainer
    {
        private :

        std::vector< PyDummy* > m_dummies;

        public :

        PyDummyContainer();
        ~PyDummyContainer();

        void addDummy( PyDummy* pyDummyPtr );
        std::vector< PyDummy* > getDummies();
        PyDummy* front();

    };


}

#define PYTYSOC_TESTS_BINDINGS(m) \
    py::class_<pytysoc::PyDummy>( m, "PyDummy" ) \
        .def( py::init<const std::string& >() ) \
        .def( "name", &pytysoc::PyDummy::name ); \
    py::class_<pytysoc::PyDummyContainer>( m, "PyDummyContainer" ) \
        .def( py::init<>() ) \
        .def( "addDummy", &pytysoc::PyDummyContainer::addDummy ) \
        .def( "getDummies", &pytysoc::PyDummyContainer::getDummies ) \
        .def( "front", &pytysoc::PyDummyContainer::front );
