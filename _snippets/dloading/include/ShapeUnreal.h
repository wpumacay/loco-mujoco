
#pragma once

#include <Shape.h>


namespace core {
namespace unreal {

    extern "C" Shape* create( const std::string& name );

    class ShapeUnreal : public Shape
    {
        protected :

        void _drawInternal() override;
        void _printInternal( const std::string& msg ) override;

        public :

        ShapeUnreal( const std::string& name );
        ~ShapeUnreal();

    };


}}