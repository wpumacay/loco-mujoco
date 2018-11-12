
#pragma once

#include "mjcf_common.h"
#include "mjcf_utils.h"


namespace mjcf
{

    struct GenericElement
    {
        std::string                     etype;
        GenericElement*                 parent;
        std::vector< GenericElement* >  children;

        std::map< std::string, int >            _ints;
        std::map< std::string, float >          _floats;
        std::map< std::string, Sizef >          _sizefs;
        std::map< std::string, Sizei >          _sizeis;
        std::map< std::string, std::string >    _strings;

        GenericElement();
        GenericElement( const std::string& elementType );

        void collectAttribs( Schema* schema, tinyxml2::XMLElement* xmlElement );
        void insertAttribs( tinyxml2::XMLElement* xmlElement );

        void setAttributeInt( const std::string& attribName, int val );
        void setAttributeFloat( const std::string& attribName, float val );
        void setAttributeArrayInt( const std::string& attribName, const Sizei& arrayint );
        void setAttributeArrayFloat( const std::string& attribName, const Sizef& arrayfloat );
        void setAttributeString( const std::string& attribName, const std::string& val );

        void setAttributeVec3( const std::string& attribName, const Vec3& vec );
        void setAttributeVec4( const std::string& attribName, const Vec4& vec );

        int getAttributeInt( const std::string& attribName );
        float getAttributeFloat( const std::string& attribName );
        Sizef getAttributeArrayFloat( const std::string& attribName );
        Sizei getAttributeArrayInt( const std::string& attribName );
        std::string getAttributeString( const std::string& attribName );
        
        void print();
    };
}