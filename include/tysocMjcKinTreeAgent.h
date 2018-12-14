
#pragma once

#include <tysocMjcCommon.h>
#include <agent/types/agent_kintree.h>
#include <agent/types/agent_kintree_mjcf.h>
// #include <agent/types/agent_kintree_urdf.h> -> @TODO: urdf as well

namespace tysoc { 
namespace mujoco {

    // @TODO: Add base adapters to inherit easily and expose common API

    class TMjcKinTreeAgentWrapper
    {

        private :

        std::string             m_name;
        mjcf::GenericElement*   m_modelElmPtr;
        agent::TAgentKinTree*   m_kinTreeAgentPtr;

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;

        bool _findAndReplaceRootStartingPos( mjcf::GenericElement* elmPtr );

        void _injectMjcAssets( mjcf::GenericElement* root );
        void _injectMjcContacts( mjcf::GenericElement* root );
        void _injectMjcSensors( mjcf::GenericElement* root );

        public :

        /**
        * Constructor from mjcf
        * @param name                   String representing the name of the agent
        * @param templateModelElmPtr    mjcf with parsed template model
        * @param position               Starting position of the agent
        */
        TMjcKinTreeAgentWrapper( const std::string& name,
                                 mjcf::GenericElement* templateModelElmPtr,
                                 const TVec3& position );
        // @TODO: Add constructor from urdf
        ~TMjcKinTreeAgentWrapper();

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );

        std::string name();
        agent::TAgentKinTree* agent();

        void injectMjcResources( mjcf::GenericElement* root );

        void preStep();
        void postStep();

    };



}}