
#pragma once

#include "mjcf_common.h"


namespace mjcf
{

    std::vector< std::string > split( const std::string& str );

    std::string safeParseString( tinyxml2::XMLElement* xmlElement,
                                 const std::string& attribName,
                                 const std::string& opt = "" );

    Vec3 safeParseVec3( tinyxml2::XMLElement* xmlElement, 
                        const std::string& attribName,
                        const Vec3& opt = { 0.0f, 0.0f, 0.0f } );

    Vec4 safeParseVec4( tinyxml2::XMLElement* xmlElement, 
                        const std::string& attribName, 
                        const Vec4& opt = { 0.0f, 0.0f, 0.0f, 1.0f } );

    Size safeParseSize( tinyxml2::XMLElement* xmlElement, 
                        const std::string& attribName, 
                        const Size& opt );

    Vec3 _parseVec3( const std::string& strvec );
    Vec4 _parseVec4( const std::string& strvec );
    Size _parseSize( const std::string& strsize );
    
    Sizei _parseSizei( const std::string& strsize );
    Sizef _parseSizef( const std::string& strsize );

    int safeParseInt( tinyxml2::XMLElement* xmlElement,
                      const std::string& attribName,
                      int opt = 0 );
    float safeParseFloat( tinyxml2::XMLElement* xmlElement,
                          const std::string& attribName,
                          float opt = 0.0 );
    Sizei safeParseSizei( tinyxml2::XMLElement* xmlElement,
                          const std::string& attribName,
                          const Sizei& opt = { 0, { 0 } } );
    Sizef safeParseSizef( tinyxml2::XMLElement* xmlElement,
                          const std::string& attribName,
                          const Sizef& opt = { 0, { 0.0 } } );
}