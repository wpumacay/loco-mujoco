
#include "mjcf.h"


namespace mjcf
{
    GenericElement* _parseGenericElement( Schema* schema, tinyxml2::XMLElement* xmlElement )
    {
        auto _gelement = new GenericElement();
        _gelement->collectAttribs( schema, xmlElement );

        auto _currentChild = xmlElement->FirstChildElement();
        while ( _currentChild != NULL )
        {
            // parse child element
            auto _ieChildElement = _parseGenericElement( schema, _currentChild );
            _gelement->children.push_back( _ieChildElement );

            // check next sibling
            _currentChild = _currentChild->NextSiblingElement();
        }

        return _gelement;
    }

    GenericElement* loadGenericModel( Schema* schema, const std::string& modelfile )
    {
        tinyxml2::XMLDocument _doc;
        _doc.LoadFile( modelfile.c_str() );

        auto _root = _parseGenericElement( schema, _doc.FirstChildElement() );
        return _root;
    }

    void saveGenericModel( GenericElement* root, const std::string& modelfile )
    {
        tinyxml2::XMLDocument _doc;
        
        auto _rootXML = _doc.NewElement( root->etype.c_str() );
        _doc.InsertEndChild( _rootXML );

        for ( size_t i = 0; i < root->children.size(); i++ )
        {
            _createElement( _doc, _rootXML, root->children[i] );
        }

        _doc.SaveFile( modelfile.c_str() );
    }

    void _createElement( tinyxml2::XMLDocument& doc,
                         tinyxml2::XMLElement* parentXML,
                         GenericElement* element )
    {
        auto _xmlElement = doc.NewElement( element->etype.c_str() );
        element->insertAttribs( _xmlElement );
        parentXML->InsertEndChild( _xmlElement );

        for ( size_t i = 0; i < element->children.size(); i++ )
        {
            _createElement( doc, _xmlElement, element->children[i] );
        }
    }
}