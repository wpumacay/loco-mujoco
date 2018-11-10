
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <tinyxml2.h>

#define MAX_SIZE 10

namespace mjcf
{
    
    struct Vec3
    {
        float x;
        float y;
        float z;
    };

    struct Vec4
    {
        float x;
        float y;
        float z;
        float w;
    };

    // struct to hold a size xmlElement of max size = 3
    struct Size
    {
        int ndim;
        float buff[3];
    };

    template< class T >
    struct TSize
    {
        int ndim;
        T buff[MAX_SIZE];
    };


    std::string toString( const Vec3& vec );
    std::string toString( const Vec4& vec );
    std::string toString( const Size& size );

    template< class T >
    std::string toString( const TSize<T>& size )
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

    typedef TSize<float>    Sizef;
    typedef TSize<int>      Sizei;

    enum AttribType
    {
        TYPE_UNDEFINED,
        TYPE_INT,
        TYPE_FLOAT,
        TYPE_ARRAY_FLOAT,
        TYPE_ARRAY_INT,
        TYPE_STRING
    };

    class Schema
    {
        private :

        std::map< std::string, std::map< std::string, AttribType > > m_attribsPerElement;

        void _collectAttributesSchema( const std::string& elementName, 
                                       tinyxml2::XMLElement* xmlElement );
        void _traverse( tinyxml2::XMLElement* xmlElement );

        public :

        Schema();
        ~Schema();

        void load( const std::string& filename );
        void print();

        std::map< std::string, AttribType > getPossibleAttribs( const std::string& elementName );
        AttribType getAttribType( const std::string& elementName,
                                  const std::string& attribName );
    };
}