
#pragma once

#include <Shape.h>


namespace core {
namespace ogre {

    extern "C" Shape* create( const std::string& name );

    class ShapeOgre : public Shape
    {
        protected :

        void _drawInternal() override;
        void _printInternal( const std::string& msg ) override;

        public :

        ShapeOgre( const std::string& name );
        ~ShapeOgre();

    };


}}