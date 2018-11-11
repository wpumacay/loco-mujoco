
#include "mjcf_api.h"

namespace mjcf
{

    mjcf::GenericElement* _createWorldBody()
    {
        return new mjcf::GenericElement( "worldbody" );
    }

    mjcf::GenericElement* _createBody( const std::string& name,
                                    const mjcf::Vec3& pos,
                                    const mjcf::Vec4& quat )
    {
        auto _body = new mjcf::GenericElement( "body" );

        _body->setAttributeString( "name", name );
        _body->setAttributeVec3( "pos", pos );
        _body->setAttributeVec4( "quat", quat );

        return _body;
    }

    mjcf::GenericElement* _createGeometry( const std::string& name,
                                        const std::string& type,
                                        const mjcf::Sizef& size,
                                        float mass,
                                        const mjcf::Vec3& pos,
                                        const mjcf::Vec4& quat )
    {
        auto _geometry = new mjcf::GenericElement( "geom" );

        _geometry->setAttributeString( "name", name );
        _geometry->setAttributeString( "type", type );
        _geometry->setAttributeArrayFloat( "size", size );
        _geometry->setAttributeFloat( "mass", mass );
        _geometry->setAttributeVec3( "pos", pos );
        _geometry->setAttributeVec4( "quat", quat );

        return _geometry;
    }

    mjcf::GenericElement* _createGeometry( const std::string& name,
                                        const std::string& type,
                                        const mjcf::Sizef& size,
                                        const mjcf::Sizef& fromto,
                                        float mass )
    {
        auto _geometry = new mjcf::GenericElement( "geom" );

        _geometry->setAttributeString( "name", name );
        _geometry->setAttributeString( "type", type );
        _geometry->setAttributeArrayFloat( "size", size );
        _geometry->setAttributeFloat( "mass", mass );
        _geometry->setAttributeArrayFloat( "fromto", fromto );

        return _geometry;
    }







}