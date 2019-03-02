
#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <agent_wrapper.h>

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
    class TMjcKinTreeAgentWrapper : public TKinTreeAgentWrapper
    {

        private :

        // mujoco data to be sent to the xml
        mjcf::GenericElement* m_mjcfResourcesPtr;
        // mjcf data where to send the data from above
        mjcf::GenericElement* m_mjcfTargetResourcesPtr;

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

        protected :

        void _initializeInternal() override;
        void _resetInternal() override;
        void _preStepInternal() override;
        void _postStepInternal() override;

        public :

        TMjcKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                 const std::string& workingDir );
        ~TMjcKinTreeAgentWrapper();

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );
        void setMjcfTargetElm( mjcf::GenericElement* targetResourcesPtr );

    };


    extern "C" TKinTreeAgentWrapper* agent_createFromAbstract( agent::TAgentKinTree* kinTreeAgentPtr,
                                                               const std::string& workingDir );

    extern "C" TKinTreeAgentWrapper* agent_createFromFile( const std::string& name,
                                                           const std::string& filename,
                                                           const std::string& workingDir );

    extern "C" TKinTreeAgentWrapper* agent_createFromId( const std::string& name,
                                                         const std::string& format,
                                                         const std::string& id,
                                                         const std::string& workingDir );

}}