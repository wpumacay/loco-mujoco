
#pragma once

#include "mjcf_common.h"
#include "mjcf_utils.h"


namespace mjcf
{

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