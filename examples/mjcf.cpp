
#include "mjcf.h"


namespace mjcf
{

    IElement* loadModel( const std::string& modelfile )
    {
        tinyxml2::XMLDocument _doc;
        _doc.LoadFile( modelfile.c_str() );

        auto _root = parseElement( _doc.FirstChildElement() );
        return _root;
    }

    void saveModel( IElement* root, const std::string& modelfile )
    {
        tinyxml2::XMLDocument _doc;
        
        auto _rootXML = _doc.NewElement( root->etype.c_str() );
        _doc.InsertEndChild( _rootXML );

        for ( size_t i = 0; i < root->children.size(); i++ )
        {
            createElement( _doc, _rootXML, root->children[i] );
        }

        _doc.SaveFile( modelfile.c_str() );
    }

    void createElement( tinyxml2::XMLDocument& doc, 
                        tinyxml2::XMLElement* parentXML, 
                        IElement* element )
    {
        auto _xmlElement = doc.NewElement( element->etype.c_str() );
        element->insertAttribs( _xmlElement );
        parentXML->InsertEndChild( _xmlElement );

        for ( size_t i = 0; i < element->children.size(); i++ )
        {
            createElement( doc, _xmlElement, element->children[i] );
        }
    }

    IElement* parseElement( tinyxml2::XMLElement* xmlElement )
    {
        IElement* _ielement = NULL;

        std::string _etype = xmlElement->Value();
        if ( _etype == "mujoco" )
        {
            _ielement = parseRoot( xmlElement );
        }
        else if ( _etype == "worldbody" )
        {
            _ielement = parseWorldBody( xmlElement );
        }
        else if ( _etype == "body" )
        {
            _ielement = parseBody( xmlElement );
        }
        else if ( _etype == "geom" )
        {
            _ielement = parseGeometry( xmlElement );
        }
        else if ( _etype == "joint" )
        {
            _ielement = parseJoint( xmlElement );
        }
        else if ( _etype == "light" )
        {
            _ielement = parseLight( xmlElement );
        }
        else if ( _etype == "camera" )
        {
            _ielement = parseCamera( xmlElement );
        }

        auto _firstChild = xmlElement->FirstChildElement();
        auto _currentChild = _firstChild;
        while ( _currentChild != NULL )
        {
            // parse child element
            auto _ieChildElement = parseElement( _currentChild );
            _ielement->children.push_back( _ieChildElement );

            // check next sibling
            _currentChild = _currentChild->NextSiblingElement();
        }

        return _ielement;
    }

    std::string toString( const Vec3& vec )
    {
        std::string _res;

        _res += std::to_string( vec.x );
        _res += " ";
        _res += std::to_string( vec.y );
        _res += " ";
        _res += std::to_string( vec.z );

        return _res;
    }

    std::string toString( const Vec4& vec )
    {
        std::string _res;

        _res += std::to_string( vec.x );
        _res += " ";
        _res += std::to_string( vec.y );
        _res += " ";
        _res += std::to_string( vec.z );
        _res += " ";
        _res += std::to_string( vec.w );

        return _res;
    }

    std::string toString( const Size& size )
    {
        std::string _res;

        for ( size_t i = 0; i < size.ndim; i++ )
        {
            _res += std::to_string( size.buff[i] );
            if ( i != ( size.ndim - 1 ) )
            {
                _res += " ";
            }
        }

        return _res;
    }

    std::vector< std::string > split( const std::string& str )
    {
        std::vector< std::string > _res;
                    
        int pos = str.find( ' ' );
        if ( pos == std::string::npos )
        {
            _res.push_back( str );
            return _res;
        }

        int initpos = 0;

        while ( pos != std::string::npos )
        {
            _res.push_back( str.substr( initpos, pos - initpos ) );
            initpos = pos + 1;

            pos = str.find( ' ', initpos );
        }

        _res.push_back( str.substr( initpos, std::min( pos, (int) str.size() ) - initpos ) );
                    
        return _res;
    }

    std::string safeParseString( tinyxml2::XMLElement* xmlElement,
                                 const std::string& attribName,
                                 const std::string& opt )
    {
        auto _attrib = xmlElement->Attribute( attribName.c_str() );

        if ( _attrib )
        {
            return std::string( _attrib );
        }

        return opt;
    }

    Vec3 safeParseVec3( tinyxml2::XMLElement* xmlElement, 
                        const std::string& attribName, 
                        const Vec3& opt )
    {
        auto _attrib = xmlElement->Attribute( attribName.c_str() );

        if ( _attrib )
        {
            return _parseVec3( _attrib );
        }
        return opt;
    }

    Vec4 safeParseVec4( tinyxml2::XMLElement* xmlElement, 
                        const std::string& attribName, 
                        const Vec4& opt )
    {
        auto _attrib = xmlElement->Attribute( attribName.c_str() );

        if ( _attrib )
        {
            return _parseVec4( _attrib );
        }
        return opt;
    }

    Size safeParseSize( tinyxml2::XMLElement* xmlElement, 
                        const std::string& attribName,  
                        const Size& opt )
    {
        auto _attrib = xmlElement->Attribute( attribName.c_str() );

        if ( _attrib )
        {
            return _parseSize( _attrib );
        }
        return opt;
    }

    Vec3 _parseVec3( const std::string& strvec )
    {
        Vec3 _res;
        
        auto _fields = split( strvec );
        _res.x = std::stof( _fields[0] );
        _res.y = std::stof( _fields[1] );
        _res.z = std::stof( _fields[2] );

        return _res;
    }

    Vec4 _parseVec4( const std::string& strvec )
    {
        Vec4 _res;
        
        auto _fields = split( strvec );
        _res.x = std::stof( _fields[0] );
        _res.y = std::stof( _fields[1] );
        _res.z = std::stof( _fields[2] );
        _res.w = std::stof( _fields[3] );

        return _res;
    }

    Size _parseSize( const std::string& strsize )
    {
        Size _res;

        auto _fields = split( strsize );

        // @TOQO: Add asserts
        _res.ndim = _fields.size();
        for ( size_t i = 0; i < _fields.size(); i++ )
        {
            _res.buff[i] = std::stof( _fields[i] );
        }

        return _res;
    }

    void EBody::print()
    {
        std::cout << "el: " << etype
                  << " | name: " << name 
                  << " | type: " << type
                  << " | pos: " << toString( pos )
                  << " | size: " << toString( size )
                  << std::endl;
    }

    void EJoint::print()
    {
        std::cout << "el: " << etype
                  << " | name: " << name 
                  << " | type: " << type
                  << std::endl;
    }

    void EGeom::print()
    {
        std::cout << "el: " << etype
                  << " | name: " << name 
                  << " | type: " << type
                  << " | size: " << toString( size )
                  << " | rgba: " << toString( rgba )
                  << std::endl;
    }

    void ELight::print()
    {
        std::cout << "el: " << etype
                  << " | name: " << name 
                  << " | diffuse: " << toString( diffuse )
                  << " | pos: " << toString( pos )
                  << " | dir: " << toString( dir )
                  << std::endl;
    }

    void ECamera::print()
    {
        std::cout << "el: " << etype
                  << " | name: " << name 
                  << std::endl;
    }

    void IElement::insertAttribs( tinyxml2::XMLElement* xmlElement )
    {

    }

    void EBody::insertAttribs( tinyxml2::XMLElement* xmlElement )
    {
        if ( name != "" )
        {
            xmlElement->SetAttribute( "name", name.c_str() );
        }
        if ( type != "" )
        {
            xmlElement->SetAttribute( "type", type.c_str() );
        }
        if ( size.ndim != 0 )
        {
            xmlElement->SetAttribute( "size", toString( size ).c_str() );
        }
        
        xmlElement->SetAttribute( "pos", toString( pos ).c_str() );
    }

    void EJoint::insertAttribs( tinyxml2::XMLElement* xmlElement )
    {
        if ( name != "" )
        {
            xmlElement->SetAttribute( "name", name.c_str() );
        }
        if ( type != "" )
        {
            xmlElement->SetAttribute( "type", type.c_str() );
        }
    }

    void EGeom::insertAttribs( tinyxml2::XMLElement* xmlElement )
    {
        if ( name != "" )
        {
            xmlElement->SetAttribute( "name", name.c_str() );
        }
        if ( type != "" )
        {
            xmlElement->SetAttribute( "type", type.c_str() );
        }
        if ( size.ndim != 0 )
        {
            xmlElement->SetAttribute( "size", toString( size ).c_str() );
        }

        xmlElement->SetAttribute( "rgba", toString( rgba ).c_str() );
    }

    void ELight::insertAttribs( tinyxml2::XMLElement* xmlElement )
    {
        if ( name != "" )
        {
            xmlElement->SetAttribute( "name", name.c_str() );
        }

        xmlElement->SetAttribute( "diffuse", toString( diffuse ).c_str() );
        xmlElement->SetAttribute( "pos", toString( pos ).c_str() );
        xmlElement->SetAttribute( "dir", toString( dir ).c_str() );
    }

    void ECamera::insertAttribs( tinyxml2::XMLElement* xmlElement )
    {
        if ( name != "" )
        {
            xmlElement->SetAttribute( "name", name.c_str() );
        }
    }

    IElement* parseRoot( tinyxml2::XMLElement* xmlElement )
    {
        auto _ielement = new IElement();
        
        _ielement->etype = "mujoco";

        return _ielement;
    }

    IElement* parseWorldBody( tinyxml2::XMLElement* xmlElement )
    {
        auto _ielement = new IElement();
        
        _ielement->etype = "worldbody";

        return _ielement;
    }

    IElement* parseBody( tinyxml2::XMLElement* xmlElement )
    {
        auto _ielement = new EBody();
        
        _ielement->etype    = "body";
        _ielement->type     = safeParseString( xmlElement, "type", "" );
        _ielement->pos      = safeParseVec3( xmlElement, "pos" );
        _ielement->size     = safeParseSize( xmlElement, "size", { 0, { 0.0, 0.0, 0.0 } } );

        return _ielement;
    }

    IElement* parseGeometry( tinyxml2::XMLElement* xmlElement )
    {
        auto _ielement = new EGeom();
        
        _ielement->etype    = "geom";
        _ielement->type     = safeParseString( xmlElement, "type", "" );
        _ielement->size     = safeParseSize( xmlElement, "size", { 0, { 0.0, 0.0, 0.0 } } );
        _ielement->rgba     = safeParseVec4( xmlElement, "rgba" );

        return _ielement;
    }

    IElement* parseJoint( tinyxml2::XMLElement* xmlElement )
    {
        auto _ielement = new EJoint();
        
        _ielement->etype    = "joint";
        _ielement->type     = safeParseString( xmlElement, "type", "" );

        return _ielement;
    }

    IElement* parseLight( tinyxml2::XMLElement* xmlElement )
    {
        auto _ielement = new ELight();
        
        _ielement->etype    = "light";
        _ielement->diffuse  = safeParseVec3( xmlElement, "diffuse" );
        _ielement->pos      = safeParseVec3( xmlElement, "pos" );
        _ielement->dir      = safeParseVec3( xmlElement, "dir" );

        return _ielement;
    }

    IElement* parseCamera( tinyxml2::XMLElement* xmlElement )
    {
        auto _ielement = new ECamera();
        
        _ielement->etype = "camera";

        return _ielement;
    }

}