
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

        public :

        TMjcAgentWrapper( const std::string& name,
                          mjcf::GenericElement* modelElmPtr );
        ~TMjcAgentWrapper();

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );

        std::string name();
        tysocagent::TAgent* agent() { return m_agentPtr; }

        void injectMjcResources( mjcf::GenericElement* root );

        void preStep();// actuator controls
        void postStep();// updates the bodies and joints
    };

}