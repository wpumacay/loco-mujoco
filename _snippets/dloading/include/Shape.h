
#pragma once

#include <string>
#include <iostream>

namespace core
{


    class Shape
    {
        protected :

        std::string m_name;

        virtual void _drawInternal() = 0;
        virtual void _printInternal( const std::string& msg ) = 0;

        public :

        Shape( const std::string& name );
        ~Shape();

        void draw();
        void print( const std::string& msg );

    };

    typedef Shape* create_t( const std::string& );

}