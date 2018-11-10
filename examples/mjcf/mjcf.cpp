
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
        // std::cout << "parsing element type: " << xmlElement->Value() << std::endl;

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
        else
        {
            std::cout << "WARNING> not supported type: " << _etype << std::endl;
            _ielement = new IElement();
            _ielement->etype = _etype;
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