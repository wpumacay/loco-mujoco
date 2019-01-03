
#include <ShapeOgre.h>

namespace core {
namespace ogre {


    ShapeOgre::ShapeOgre( const std::string& name )
        : Shape( name )
    {

    }

    ShapeOgre::~ShapeOgre()
    {

    }

    void ShapeOgre::_drawInternal()
    {
        std::cout << "DRAW::OGRE> shape.name: " << m_name << std::endl;
        std::cout << "DRAW::OGRE> drawing ogre shape" << std::endl;
    }

    void ShapeOgre::_printInternal( const std::string& msg )
    {
        std::cout << "PRINT::OGRE> msg: " << msg << std::endl;
    }

    extern "C" Shape* create( const std::string& name )
    {
        return new ShapeOgre( name );
    }

}}