
#include <ShapeUnreal.h>

namespace core {
namespace unreal {


    ShapeUnreal::ShapeUnreal( const std::string& name )
        : Shape( name )
    {

    }

    ShapeUnreal::~ShapeUnreal()
    {

    }

    void ShapeUnreal::_drawInternal()
    {
        std::cout << "DRAW::UNREAL> shape.name: " << m_name << std::endl;
        std::cout << "DRAW::UNREAL> drawing unreal shape" << std::endl;
    }

    void ShapeUnreal::_printInternal( const std::string& msg )
    {
        std::cout << "PRINT::UNREAL> msg: " << msg << std::endl;
    }

    extern "C" Shape* create( const std::string& name )
    {
        return new ShapeUnreal( name );
    }

}}