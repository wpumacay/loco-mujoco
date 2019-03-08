
#include <mujoco_common.h>

namespace tysoc {
namespace mujoco {
namespace utils {

    void setTerrainBodyPosition( mjModel* pModel,
                                 mjData* pData,
                                 const std::string& name, 
                                 const TVec3& pos )
    {
        auto _id = mj_name2id( pModel, mjOBJ_BODY, name.c_str() );

        if ( _id != -1 )
        {
            pModel->body_pos[ 3 * _id + 0 ] = pos.x;
            pModel->body_pos[ 3 * _id + 1 ] = pos.y;
            pModel->body_pos[ 3 * _id + 2 ] = pos.z;
        }
        else
        {
            std::cout << "body: " << name << " not found" << std::endl;
        }
    }

    void setTerrainBodyOrientation( mjModel* pModel,
                                    mjData* pData,
                                    const std::string& name,
                                    float rotmat[9] )
    {
        auto _id = mj_name2id( pModel, mjOBJ_BODY, name.c_str() );

        if ( _id != -1 )
        {
            mjtNum _rquat[4];
            mjtNum _rmat[9];
            
            _rmat[0] = rotmat[0];
            _rmat[1] = rotmat[3];
            _rmat[2] = rotmat[6];

            _rmat[3] = rotmat[1];
            _rmat[4] = rotmat[4];
            _rmat[5] = rotmat[7];

            _rmat[6] = rotmat[2];
            _rmat[7] = rotmat[5];
            _rmat[8] = rotmat[8];

            mju_mat2Quat( _rquat, _rmat );
            pModel->body_quat[ 4 * _id + 0 ] = _rquat[0];
            pModel->body_quat[ 4 * _id + 1 ] = _rquat[1];
            pModel->body_quat[ 4 * _id + 2 ] = _rquat[2];
            pModel->body_quat[ 4 * _id + 3 ] = _rquat[3];
        }
    }

    void setBodyPosition( mjModel* pModel,
                          mjData* pData,
                          const std::string& name, 
                          const TVec3& pos )
    {
        auto _id = mj_name2id( pModel, mjOBJ_BODY, name.c_str() );

        if ( _id != -1 )
        {
            pData->xpos[ 3 * _id + 0 ] = pos.x;
            pData->xpos[ 3 * _id + 1 ] = pos.y;
            pData->xpos[ 3 * _id + 2 ] = pos.z;
        }
        else
        {
            std::cout << "body: " << name << " not found" << std::endl;
        }
    }

    TVec3 getBodyPosition( mjModel* pModel,
                                mjData* pData,
                                const std::string& name )
    {
        TVec3 _res;
        
        auto _id = mj_name2id( pModel, mjOBJ_BODY, name.c_str() );
        // std::cout << "id: " << _id << std::endl;

        if ( _id != -1 )
        {
            _res.x = pData->xpos[ 3 * _id + 0 ];
            _res.y = pData->xpos[ 3 * _id + 1 ];
            _res.z = pData->xpos[ 3 * _id + 2 ];
        }
        else
        {
            std::cout << "body: " << name << " not found" << std::endl;
        }

        return _res;
    }

    void setBodyOrientation( mjModel* pModel,
                             mjData* pData,
                             const std::string& name,
                             float rotmat[9] )
    {
        auto _id = mj_name2id( pModel, mjOBJ_BODY, name.c_str() );

        if ( _id != -1 )
        {
            mjtNum _rquat[4];
            mjtNum _rmat[9];
            
            _rmat[0] = rotmat[0];
            _rmat[1] = rotmat[3];
            _rmat[2] = rotmat[6];

            _rmat[3] = rotmat[1];
            _rmat[4] = rotmat[4];
            _rmat[5] = rotmat[7];

            _rmat[6] = rotmat[2];
            _rmat[7] = rotmat[5];
            _rmat[8] = rotmat[8];

            mju_mat2Quat( _rquat, _rmat );
            pData->xquat[ 4 * _id + 0 ] = _rquat[0];
            pData->xquat[ 4 * _id + 1 ] = _rquat[1];
            pData->xquat[ 4 * _id + 2 ] = _rquat[2];
            pData->xquat[ 4 * _id + 3 ] = _rquat[3];
        }
    }

    void getBodyOrientation( mjModel* pModel,
                             mjData* pData,
                             const std::string& name,
                             float* rotmat )
    {
        auto _id = mj_name2id( pModel, mjOBJ_BODY, name.c_str() );

        if ( _id != -1 )
        {
            mjtNum _rquat[4];
            mjtNum _rmat[9];

            _rquat[0] = pData->xquat[ 4 * _id + 0 ];
            _rquat[1] = pData->xquat[ 4 * _id + 1 ];
            _rquat[2] = pData->xquat[ 4 * _id + 2 ];
            _rquat[3] = pData->xquat[ 4 * _id + 3 ];

            mju_quat2Mat( _rmat, _rquat );

            rotmat[0] = _rmat[0];
            rotmat[3] = _rmat[1];
            rotmat[6] = _rmat[2];

            rotmat[1] = _rmat[3];
            rotmat[4] = _rmat[4];
            rotmat[7] = _rmat[5];

            rotmat[2] = _rmat[6];
            rotmat[5] = _rmat[7];
            rotmat[8] = _rmat[8];
        }
    }

    void setActuatorCtrl( mjModel* pModel,
                          mjData* pData,
                          const std::string& name, 
                          float val )
    {
        auto _id = mj_name2id( pModel, mjOBJ_ACTUATOR, name.c_str() );

        if ( _id != -1 )
        {
            pData->ctrl[ _id ] = val;
        }
        else
        {
            std::cout << "actuator: " << name << " not found" << std::endl;
        }
    }

    float getActuatorCtrl( mjModel* pModel,
                           mjData* pData,
                           const std::string& name )
    {
        auto _id = mj_name2id( pModel, mjOBJ_ACTUATOR, name.c_str() );

        if ( _id != -1 )
        {
            return pData->ctrl[ _id ];
        }
        else
        {
            std::cout << "actuator: " << name << " not found" << std::endl;
            return 0.0f;
        }
    }

    void getGeometryTransform( mjModel* pModel,
                               mjvScene* pScene,
                               const std::string& name,
                               float* pos,
                               float* rotmat )
    {
        auto _id = mj_name2id( pModel, mjOBJ_GEOM, name.c_str() );
        
        if ( _id != -1 )
        {
            auto _mjGeom = pScene->geoms[ _id ];

            // grab the position
            pos[0] = _mjGeom.pos[0];
            pos[1] = _mjGeom.pos[1];
            pos[2] = _mjGeom.pos[2];
            
            // grab the orientation
            rotmat[0] = _mjGeom.mat[0];
            rotmat[1] = _mjGeom.mat[1];
            rotmat[2] = _mjGeom.mat[2];
            rotmat[3] = _mjGeom.mat[3];
            rotmat[4] = _mjGeom.mat[4];
            rotmat[5] = _mjGeom.mat[5];
            rotmat[6] = _mjGeom.mat[6];
            rotmat[7] = _mjGeom.mat[7];
            rotmat[8] = _mjGeom.mat[8];
        }
        else
        {
            pos[0] = 0.0f;
            pos[1] = 0.0f;
            pos[2] = 0.0f;

            rotmat[0] = 1.0f;
            rotmat[1] = 0.0f;
            rotmat[2] = 0.0f;
            rotmat[3] = 0.0f;
            rotmat[4] = 1.0f;
            rotmat[5] = 0.0f;
            rotmat[6] = 0.0f;
            rotmat[7] = 0.0f;
            rotmat[8] = 1.0f;
        }
    }

    void getGeometryColor( mjModel* pModel,
                           mjvScene* pScene,
                           const std::string& name,
                           float* color )
    {
        auto _id = mj_name2id( pModel, mjOBJ_GEOM, name.c_str() );
        
        if ( _id != -1 )
        {
            auto _mjGeom = pScene->geoms[ _id ];

            color[0] = _mjGeom.rgba[0];
            color[1] = _mjGeom.rgba[1];
            color[2] = _mjGeom.rgba[2];
        }
        else
        {
            color[0] = 0.0f;
            color[1] = 1.0f;
            color[2] = 0.0f;
        }
    }

    void getJointSensorReading( mjModel* pModel,
                                mjData* pData,
                                const std::string& name,
                                std::vector< float >& readings )
    {
        auto _id = mj_name2id( pModel, mjOBJ_SENSOR, name.c_str() );

        if ( _id != -1 )
        {
            auto _mjSensorAdr = pModel->sensor_adr[_id];
            auto _mjSensorDim = pModel->sensor_dim[_id];

            for ( size_t i = 0; i < _mjSensorDim; i++ )
            {
                readings.push_back( pData->sensordata[ _mjSensorAdr + i ] );
            }
        }
        else
        {
            std::cout << "sensor with name " << name << " does not exist" << std::endl;
        }
    }

    void changeSize( mjModel* pModel,
                     const std::string& name,
                     const TVec3& size )
    {
        auto _id = mj_name2id( pModel, mjOBJ_GEOM, name.c_str() );

        if ( _id != -1 )
        {
            pModel->geom_size[ 3 * _id + 0] = size.x;
            pModel->geom_size[ 3 * _id + 1] = size.y;
            pModel->geom_size[ 3 * _id + 2] = size.z;
        }
        else
        {
            std::cout << "geom: " << name << " not found" << std::endl;
        }
    }

    void setRbound( mjModel* pModel,
                    const std::string& name,
                    float radius )
    {
        auto _id = mj_name2id( pModel, mjOBJ_GEOM, name.c_str() );

        if ( _id != -1 )
        {
            pModel->geom_rbound[ _id ] = radius;
        }
        else
        {
            std::cout << "geom: " << name << " not found" << std::endl;
        }
    }

    void getCOMForces( mjModel* pModel,
                       mjData* pData,
                       const std::string& name,
                       TVec3& forces,
                       TVec3& torques )
    {
        auto _id = mj_name2id( pModel, mjOBJ_BODY, name.c_str() );

        if ( _id != -1 )
        {
            forces.x = pData->cfrc_ext[ 6 * _id + 0 ];
            forces.y = pData->cfrc_ext[ 6 * _id + 1 ];
            forces.z = pData->cfrc_ext[ 6 * _id + 2 ];

            torques.x = pData->cfrc_ext[ 6 * _id + 3 ];
            torques.y = pData->cfrc_ext[ 6 * _id + 4 ];
            torques.z = pData->cfrc_ext[ 6 * _id + 5 ];
        }
        else
        {
            std::cout << "body: " << name << " not found" << std::endl;
        }
    }

}}}