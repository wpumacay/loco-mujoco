
#include <iostream>
#include <string>
#include <map>

#include <Shape.h>

#include <dlfcn.h>

#ifndef DLIBRARIES_PATH
    #define DLIBRARIES_PATH "./build/"
#endif

static std::map< std::string, std::string > LIBRARIES = { { "ogre", "libshape_ogre.so" }, { "unreal", "libshape_unreal.so" } };

core::create_t* loadCreationFunction( const std::string& libraryPath )
{
    void* _library = dlopen( libraryPath.c_str(), RTLD_NOW );
    if ( !_library )
    {
        std::cout << "ERROR> could not load library: " << dlerror() << std::endl;
        return NULL;
    }

    core::create_t* _creationFcn = ( core::create_t* ) dlsym( _library, "create" );
    if ( !_creationFcn )
    {
        std::cout << "ERROR> error while loading symbol: " << dlerror() << std::endl;
        return NULL;
    }

    return _creationFcn;
}

int main( int argc, const char** argv )
{
    std::cout << "DEMO: DYNAMICALLY LOADING LIBRARIES (BOTH AT SAME TIME)" << std::endl;

    auto _createShapeFcnOgre    = loadCreationFunction( std::string( DLIBRARIES_PATH ) + "libshape_ogre.so" );
    auto _createShapeFcnUnreal  = loadCreationFunction( std::string( DLIBRARIES_PATH ) + "libshape_unreal.so" );

    auto _shapeOgre = _createShapeFcnOgre( "fooooo" );
    _shapeOgre->draw();
    _shapeOgre->print( "bar" );

    auto _shapeUnreal = _createShapeFcnUnreal( "fuuuuu" );
    _shapeUnreal->draw();
    _shapeUnreal->print( "jojojojo" );

    std::cout << "FINISHED DEMO" << std::endl;

    return 0;
}