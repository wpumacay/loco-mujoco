
#include "mjcf_elements.h"



namespace mjcf
{

    void GenericElement::collectAttribs( Schema* schema, tinyxml2::XMLElement* xmlElement )
    {
        etype = xmlElement->Value();

        auto _possibleAttribs = schema->getPossibleAttribs( etype );
        for ( auto const& it : _possibleAttribs )
        {
            auto _attribName = it.first;
            auto _attribType = it.second;

            if ( !xmlElement->Attribute( _attribName.c_str() ) )
            {
                continue;
            }

            if ( _attribType == TYPE_INT )
            {
                _ints[ _attribName ] = safeParseInt( xmlElement, _attribName );
            }
            else if ( _attribType == TYPE_FLOAT )
            {
                _floats[ _attribName ] = safeParseFloat( xmlElement, _attribName );
            }
            else if ( _attribType == TYPE_ARRAY_INT )
            {
                _sizeis[ _attribName ] = safeParseSizei( xmlElement, _attribName );
            }
            else if ( _attribType == TYPE_ARRAY_FLOAT )
            {
                _sizefs[ _attribName ] = safeParseSizef( xmlElement, _attribName );
            }
            else if ( _attribType == TYPE_STRING )
            {
                _strings[ _attribName ] = std::string( xmlElement->Attribute( _attribName.c_str() ) );
            }
        }
    }
    
    void GenericElement::insertAttribs( tinyxml2::XMLElement* xmlElement )
    {
        // insert ints
        for ( auto const& it : _ints )
        {
            xmlElement->SetAttribute( it.first.c_str(), std::to_string( it.second ).c_str() );
        }
        // insert floats
        for ( auto const& it : _floats )
        {
            xmlElement->SetAttribute( it.first.c_str(), std::to_string( it.second ).c_str() );
        }
        // insert sizeis
        for ( auto const& it : _sizeis )
        {
            xmlElement->SetAttribute( it.first.c_str(), toString( it.second ).c_str() );
        }
        // insert sizefs
        for ( auto const& it : _sizefs )
        {
            xmlElement->SetAttribute( it.first.c_str(), toString( it.second ).c_str() );
        }
        // insert strings
        for ( auto const& it : _strings )
        {
            xmlElement->SetAttribute( it.first.c_str(), it.second.c_str() );
        }
    }

    void GenericElement::print()
    {
        std::cout << "<" << etype;

        // insert ints
        for ( auto const& it : _ints )
        {
            std::cout << it.first << "=\"" << std::to_string( it.second ) << "\" ";
        }
        // insert floats
        for ( auto const& it : _floats )
        {
            std::cout << it.first << "=\"" << std::to_string( it.second ) << "\" ";
        }
        // insert sizeis
        for ( auto const& it : _sizeis )
        {
            std::cout << it.first << "=\"" << toString( it.second ) << "\" ";
        }
        // insert sizefs
        for ( auto const& it : _sizefs )
        {
            std::cout << it.first << "=\"" << toString( it.second ) << "\" ";
        }
        // insert strings
        for ( auto const& it : _strings )
        {
            std::cout << it.first << "=\"" << it.second << "\" ";
        }

        std::cout << "/>" << std::endl;
    }

    void GenericElement::setAttributeInt( const std::string& attribName, int val )
    {
        // @TODO
    }

    void GenericElement::setAttributeFloat( const std::string& attribName, float val )
    {
        // @TODO
    }

    void GenericElement::setAttributeArrayInt( const std::string& attribName, const Sizei& arrayint )
    {
        // @TODO
    }

    void GenericElement::setAttributeArrayFloat( const std::string& attribName, const Sizef& arrayfloat )
    {
        // @TODO: Should check with the schema
        _sizefs[ attribName ] = arrayfloat;
    }

    void GenericElement::setAttributeString( const std::string& attribName, const std::string& val )
    {
        _strings[ attribName ] = val;
    }

    void GenericElement::setAttributeVec3( const std::string& attribName, const Vec3& vec )
    {
        Sizef _svec;
        _svec.ndim = 3;
        _svec.buff[0] = vec.x;
        _svec.buff[1] = vec.y;
        _svec.buff[2] = vec.z;

        _sizefs[ attribName ] = _svec;
    }

    void GenericElement::setAttributeVec4( const std::string& attribName, const Vec4& vec )
    {
        Sizef _svec;
        _svec.ndim = 4;
        _svec.buff[0] = vec.x;
        _svec.buff[1] = vec.y;
        _svec.buff[2] = vec.z;
        _svec.buff[3] = vec.w;

        _sizefs[ attribName ] = _svec;
    }

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