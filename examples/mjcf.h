
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <tinyxml2.h>



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

    std::string toString( const Vec3& vec );
    std::string toString( const Vec4& vec );
    std::string toString( const Size& size );

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

    struct IElement
    {
        std::string                 etype;
        std::vector< IElement* >    children;
        // std::vector< IElement* >    siblings;

        virtual void print() { std::cout << "el: " << etype << std::endl; }
        virtual void insertAttribs( tinyxml2::XMLElement* xmlElement );
    };

    struct EBody : public IElement
    {
        std::string name;
        std::string type;
        Vec3        pos;
        Size        size;

        void print() override;
        void insertAttribs( tinyxml2::XMLElement* xmlElement ) override;
    };

    struct EJoint : public IElement
    {
        std::string name;
        std::string type;

        void print() override;
        void insertAttribs( tinyxml2::XMLElement* xmlElement ) override;
    };

    struct EGeom : public IElement
    {
        std::string name;
        std::string type;
        Size        size;
        Vec4        rgba;

        void print() override;
        void insertAttribs( tinyxml2::XMLElement* xmlElement ) override;
    };

    struct ELight : public IElement
    {
        std::string name;
        Vec3        diffuse;
        Vec3        pos;
        Vec3        dir;

        void print() override;
        void insertAttribs( tinyxml2::XMLElement* xmlElement ) override;
    };

    struct ECamera : public IElement
    {
        std::string name;
        
        void print() override;
        void insertAttribs( tinyxml2::XMLElement* xmlElement ) override;
    };

    // recursive node creation
    void createElement( tinyxml2::XMLDocument& doc, 
                        tinyxml2::XMLElement* parentXML, 
                        IElement* element );

    // recursive node parsing, calls other helpers below
    IElement* parseElement( tinyxml2::XMLElement* xmlElement );

    // node-parsing helpers
    IElement* parseRoot( tinyxml2::XMLElement* xmlElement );
    IElement* parseWorldBody( tinyxml2::XMLElement* xmlElement );
    IElement* parseBody( tinyxml2::XMLElement* xmlElement );
    IElement* parseGeometry( tinyxml2::XMLElement* xmlElement );
    IElement* parseJoint( tinyxml2::XMLElement* xmlElement );
    IElement* parseLight( tinyxml2::XMLElement* xmlElement );
    IElement* parseCamera( tinyxml2::XMLElement* xmlElement );

    IElement* loadModel( const std::string& modelfile );
    void saveModel( IElement* root, const std::string& modelfile );
}