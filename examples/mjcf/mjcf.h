
#pragma once

#include "mjcf_common.h"
#include "mjcf_utils.h"
#include "mjcf_elements.h"


namespace mjcf
{
    /**
     * Parses xml element into a generic, given the schema for error checking
     */
    GenericElement* _parseGenericElement( Schema* schema, tinyxml2::XMLElement* xmlElement );

    /**
     * 
     */
    void _createElement( tinyxml2::XMLDocument& doc,
                         tinyxml2::XMLElement* parentXML,
                         GenericElement* element );

    /**
     * 
     */
    GenericElement* loadGenericModel( Schema* schema, const std::string& modelfile );

    /**
     * 
    */
    void saveGenericModel( GenericElement* root, const std::string& modelfile );
}