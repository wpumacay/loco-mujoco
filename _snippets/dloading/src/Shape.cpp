
#include <Shape.h>



namespace core 
{

    Shape::Shape( const std::string& name )
    {
        m_name = name;
    }

    Shape::~Shape()
    {

    }

    void Shape::draw()
    {
        _drawInternal();
    }

    void Shape::print( const std::string& msg )
    {
        _printInternal( msg );
    }


}