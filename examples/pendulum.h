
#pragma once

#include "mjcf/mjcf_api.h"

#define PENDULUM_DEFAULT_NUM_LINKS 5
#define PENDULUM_DEFAULT_LINK_LEGTH 0.5f
#define PENDULUM_DEFAULT_LINK_NAME "link_"
#define PENDULUM_DEFAULT_JOINT_NAME "joint_"

namespace pendulum
{

    mjcf::GenericElement* create( int numLinks = PENDULUM_DEFAULT_NUM_LINKS, 
                                  float linkLength = PENDULUM_DEFAULT_LINK_LEGTH );

    mjcf::GenericElement* _createLinks( int numLinks, float linkLength );

    mjcf::GenericElement* _createLink( mjcf::GenericElement* parent, 
                                       int linkIndx, float linkLength );

}