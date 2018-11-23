
#pragma once

#include <tysocMjcCommon.h>
#include <agent/agent.h>

namespace tysocMjc
{

    class TMjcAgentWrapper
    {

        private :

        std::string m_name;

        tysocagent::TAgent* m_agentPtr;

        mjcf::GenericElement* m_modelElmPtr;

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;

        void _createWrappedAgentObj();
        void _collectBodyGeometryOrJoint( mjcf::GenericElement* elm );
        void _collectActuator( mjcf::GenericElement* elm );

        void _extractStandardSize( mjcf::GenericElement* geomElm,
                                   float* targetSize );

        float m_startX;
        float m_startY;
        float m_startZ;

        public :

        TMjcAgentWrapper( const std::string& name,
                          mjcf::GenericElement* modelElmPtr,
                          float posX, float posY, float posZ );
        ~TMjcAgentWrapper();

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );

        std::string name();
        tysocagent::TAgent* agent() { return m_agentPtr; }

        void getPosition( float &x, float &y, float &z );
        void setPosition( float x, float y, float z );

        void injectMjcResources( mjcf::GenericElement* root );

        void preStep();// actuator controls
        void postStep();// updates the bodies and joints
    };

}