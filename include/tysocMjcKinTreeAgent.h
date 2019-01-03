
#pragma once

#include <tysocMjcCommon.h>
#include <tysocMjcUtils.h>

#include <agent/types/agent_kintree.h>
#include <agent/types/agent_kintree_mjcf.h>
#include <agent/types/agent_kintree_urdf.h> // @WIP

namespace tysoc { 
namespace mujoco {

    // @TODO: Add base adapters to inherit easily and expose common API

    /**
    * Agent wrapper specific for the "MuJoCo" backend. It should handle ...
    * the abstract kintree initialized by the core functionality, and use it ...
    * to create the appropiate xml data needed for mujoco to compile the required models
    * (It works in this way, so it's ok. It's fine that it does everything at the start, ...
    *  maybe it does various optimizations when compiling models :D)
    */
    class TMjcKinTreeAgentWrapper
    {

        private :

        std::string             m_name;

        // mjcf resource: a copy of the mjcf model passed for construction
        mjcf::GenericElement*   m_mjcfModelTemplatePtr;
        // urdf resource: a reference to the cached urdf model passed for construction
        urdf::UrdfModel* m_urdfModelPtr;

        // underlying kinematic tree
        agent::TAgentKinTree*   m_kinTreeAgentPtr;

        // mujoco data to be sent to the xml
        mjcf::GenericElement*   m_mjcfResourcesPtr;

        // mujoco simulation data
        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;

        void _createMjcResourcesFromKinTree();
        void _createMjcResourcesFromBodyNode( mjcf::GenericElement* parentElmPtr,
                                              agent::TKinTreeBody* kinTreeBodyPtr );
        void _createMjcAssetsFromKinTree();
        void _createMjcSensorsFromKinTree();
        void _createMjcActuatorsFromKinTree();

        TVec3 _extractMjcSizeFromStandardSize( const TGeometry& geometry );

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

        /**
        * Constructor from urdf
        * @param name               String representing the name of the agent
        * @param urdfModelPtr       urdfmodel with parsed template model
        * @param position           Starting position of the agent
        */
        TMjcKinTreeAgentWrapper( const std::string& name,
                                 urdf::UrdfModel* urdfModelPtr,
                                 const TVec3& position );

        // @TODO: Add constructor from urdf
        ~TMjcKinTreeAgentWrapper();

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );

        std::string name();
        agent::TAgentKinTree* agent();

        // @WIP: Should have some functionality to reset the model given ...
        // some information from the user, like making the links a bit larger, ...
        // or changing some properties of the joints
        void reset();

        void injectMjcResources( mjcf::GenericElement* root );

        void preStep();
        void postStep();

    };



}}