
#include <iostream>
#include <string>
#include <tinyxml2.h>

#ifndef MUJOCO_RESOURCES_PATH
    #define MUJOCO_RESOURCES_PATH "../../res/"
#endif






int main()
{
    std::string _modelPath( MUJOCO_RESOURCES_PATH );
    _modelPath += "mjxml/basic.xml";

    tinyxml2::XMLDocument _doc;
    _doc.LoadFile( _modelPath.c_str() );

    auto _root = _doc.FirstChildElement();
    std::cout << "root: " << _root->Value() << std::endl;
    auto _worldbody = _root->FirstChildElement();
    std::cout << "wbody: " << _worldbody->Value() << std::endl;

    if ( std::string( _worldbody->Value() ) == std::string( "worldbody" ) )
    {
        std::cout << "MATCHED worldbody" << std::endl;
        auto _attrib = _worldbody->Attribute( "diffuse" );
        if ( _attrib )
        {
            std::cout << "attrib: " << _attrib << std::endl;
        }
        else
        {
            std::cout << "attrib is empty!" << std::endl;
        }
    }

    auto _baseChild = _worldbody->FirstChildElement();
    auto _lastChild = _worldbody->LastChildElement();

    auto _child = _baseChild;

    while ( true )
    {
        // Do some stuff with child
        std::cout << "child: " << _child->Value() << std::endl;

        _child = _child->NextSiblingElement();
        if ( _child == NULL )
        {
            break;
        }
    }

    return 0;
}