
#pragma once

#include <mujoco_common.h>

namespace tysoc {
namespace mujoco {
namespace utils {

    void setTerrainBodyPosition( mjModel* pModel,
                                 mjData* pData,
                                 const std::string& name, 
                                 const TVec3& pos );

    void setTerrainBodyOrientation( mjModel* pModel,
                                    mjData* pData,
                                    const std::string& name,
                                    float rotmat[9] );

    void setBodyPosition( mjModel* pModel,
                          mjData* pData,
                          const std::string& name, 
                          const TVec3& pos );

    TVec3 getBodyPosition( mjModel* pModel,
                           mjData* pData,
                           const std::string& name );

    void setBodyOrientation( mjModel* pModel,
                             mjData* pData,
                             const std::string& name,
                             float rotmat[9] );

    void getBodyOrientation( mjModel* pModel,
                             mjData* pData,
                             const std::string& name,
                             float* rotmat );

    void setActuatorCtrl( mjModel* pModel,
                          mjData* pData,
                          const std::string& name, 
                          float val );

    float getActuatorCtrl( mjModel* pModel,
                           mjData* pData,
                           const std::string& name );

    void getGeometryTransform( mjModel* pModel,
                               mjvScene* pScene,
                               const std::string& name,
                               float* pos,
                               float* rotmat );

    void getGeometryColor( mjModel* pModel,
                           mjvScene* pScene,
                           const std::string& name,
                           float* color );

    void getJointSensorReading( mjModel* pModel,
                                mjData* pData,
                                const std::string& name,
                                std::vector< float >& readings );

    void changeSize( mjModel* pModel,
                     const std::string& name,
                     const TVec3& size );

    void setRbound( mjModel* pModel,
                    const std::string& name,
                    float radius );

    void getCOMForces( mjModel* pModel,
                       mjData* pData,
                       const std::string& name,
                       TVec3& forces,
                       TVec3& torques );

}}}