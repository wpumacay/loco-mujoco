
#include "mjcf_utils.h"


namespace mjcf
{

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


    Sizei _parseSizei( const std::string& strsize )
    {
        Sizei _res;

        auto _fields = split( strsize );

        // @TOQO: Add asserts
        _res.ndim = _fields.size();
        for ( size_t i = 0; i < _fields.size(); i++ )
        {
            _res.buff[i] = std::stoi( _fields[i] );
        }

        return _res;
    }

    Sizef _parseSizef( const std::string& strsize )
    {
        Sizef _res;

        auto _fields = split( strsize );

        // @TOQO: Add asserts
        _res.ndim = _fields.size();
        for ( size_t i = 0; i < _fields.size(); i++ )
        {
            _res.buff[i] = std::stof( _fields[i] );
        }

        return _res;
    }

    int safeParseInt( tinyxml2::XMLElement* xmlElement,
                      const std::string& attribName,
                      int opt )
    {
        auto _attrib = xmlElement->Attribute( attribName.c_str() );

        if ( _attrib )
        {
            return std::stoi( _attrib );
        }
        return opt;
    }

    float safeParseFloat( tinyxml2::XMLElement* xmlElement,
                          const std::string& attribName,
                          float opt )
    {
        auto _attrib = xmlElement->Attribute( attribName.c_str() );

        if ( _attrib )
        {
            return std::stof( _attrib );
        }
        return opt;
    }

    Sizei safeParseSizei( tinyxml2::XMLElement* xmlElement,
                          const std::string& attribName,
                          const Sizei& opt )
    {
        auto _attrib = xmlElement->Attribute( attribName.c_str() );

        if ( _attrib )
        {
            return _parseSizei( _attrib );
        }
        return opt;
    }

    Sizef safeParseSizef( tinyxml2::XMLElement* xmlElement,
                          const std::string& attribName,
                          const Sizef& opt )
    {
        auto _attrib = xmlElement->Attribute( attribName.c_str() );

        if ( _attrib )
        {
            return _parseSizef( _attrib );
        }
        return opt;
    }

}