
#pragma once

#include <mujoco.h>
#include "../mjcf/mjcf_common.h"


namespace mjcint
{
    
    void setBodyPosition( mjModel* pModel,
                          const std::string& name, 
                          const mjcf::Vec3& pos );

    mjcf::Vec3 getBodyPosition( mjModel* pModel,
                                const std::string& name );

    void setActuatorCtrl( mjModel* pModel,
                          mjData* pData,
                          const std::string& name, 
                          float val );

    float getActuatorCtrl( mjModel* pModel,
                           mjData* pData,
                           const std::string& name );

    // @TODO: Need functionality to modify body dimensions
}