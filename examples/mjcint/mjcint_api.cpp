
#include "mjcint_api.h"


namespace mjcint
{

    void setBodyPosition( mjModel* pModel,
                          const std::string& name, 
                          const mjcf::Vec3& pos )
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

    mjcf::Vec3 getBodyPosition( mjModel* pModel,
                                const std::string& name )
    {
        mjcf::Vec3 _res;
        
        auto _id = mj_name2id( pModel, mjOBJ_BODY, name.c_str() );
        std::cout << "id: " << _id << std::endl;

        if ( _id != -1 )
        {
            _res.x = pModel->body_pos[ 3 * _id + 0 ];
            _res.y = pModel->body_pos[ 3 * _id + 1 ];
            _res.z = pModel->body_pos[ 3 * _id + 2 ];
        }
        else
        {
            std::cout << "body: " << name << " not found" << std::endl;
        }

        return _res;
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

}