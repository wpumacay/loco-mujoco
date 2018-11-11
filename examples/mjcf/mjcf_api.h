
#pragma once

#include "mjcf.h"

namespace mjcf
{

    mjcf::GenericElement* _createWorldBody();

    mjcf::GenericElement* _createBody( const std::string& name,
                                    const mjcf::Vec3& pos = { 0, 0, 0 },
                                    const mjcf::Vec4& quat = { 0, 0, 0, 1 } );

    mjcf::GenericElement* _createGeometry( const std::string& name,
                                        const std::string& type,
                                        const mjcf::Sizef& size,
                                        float mass = 0.0f,
                                        const mjcf::Vec3& pos = { 0, 0, 0 },
                                        const mjcf::Vec4& quat = { 0, 0, 0, 1 } );

    mjcf::GenericElement* _createGeometry( const std::string& name,
                                           const std::string& type,
                                           const mjcf::Sizef& size,
                                           const mjcf::Sizef& fromto,
                                           float mass = 0.0f );


}