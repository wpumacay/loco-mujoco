
#include <iostream>
#include <string>
#include <map>

#include <Shape.h>

#include <dlfcn.h>

#ifndef DLIBRARIES_PATH
    #define DLIBRARIES_PATH "./build/"
#endif

static std::map< std::string, std::string > LIBRARIES = { { "ogre", "libshape_ogre.so" }, { "unreal", "libshape_unreal.so" } };

static std::string LIBRARY_NAME = "libshape_ogre.so";

int main( int argc, const char** argv )
{
    if ( argc > 1 )
    {
        if ( LIBRARIES.find( argv[1] ) != LIBRARIES.end() )
        {
            LIBRARY_NAME = LIBRARIES[ argv[1] ];
        }
        else
        {
            std::cout << "INFO> the library " << argv[1] << " is not supported. Using default: " << LIBRARY_NAME << std::endl;
        }
    }

    std::cout << "DEMO: DYNAMICALLY LOADING LIBRARIES" << std::endl;

    // load library
    void* _shapeLibrary = dlopen( ( std::string( DLIBRARIES_PATH ) + LIBRARY_NAME ).c_str(), RTLD_NOW );
    if ( !_shapeLibrary )
    {
        std::cout << "ERROR> error while loading library: " << dlerror() << std::endl;
        return -1;
    }

    // load creation functions
    core::create_t* _createShapeFcn = ( core::create_t* ) dlsym( _shapeLibrary, "create" );
    if ( !_createShapeFcn )
    {
        std::cout << "ERROR> error while loading symbol: " << dlerror() << std::endl;
        return -1;
    }

    auto _shape = _createShapeFcn( "fooooo" );
    _shape->draw();
    _shape->print( "bar" );

    std::cout << "FINISHED DEMO" << std::endl;

    return 0;
}