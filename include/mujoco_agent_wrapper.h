#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <agent_wrapper.h>

namespace tysoc { 
namespace mujoco {

    class TMjcBodyWrapper
    {
        private :

        int m_id;
        TKinTreeBody* m_kinTreeBody;

        public :

        TMjcBodyWrapper( int id );
    };

    std::string enumJointToMjcType( const eJointType& type );
    std::string enumShapeToMjcType( const eShapeType& type );
    std::string enumActuatorToMjcType( const eActuatorType& type );

    class TMjcJointWrapper
    {
        private :

        int m_id;
        int m_nqpos;
        int m_nqvel;
        int m_qposAdr;
        int m_qvelAdr;

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;

        TKinTreeJoint* m_kinTreeJointPtr;

        public :

        TMjcJointWrapper( mjModel* mjcModelPtr,
                          mjData* mjcDataPtr,
                          TKinTreeJoint* jointPtr );

        void setQpos( const std::vector< TScalar >& qpos );
        void setQvel( const std::vector< TScalar >& qvel );

        TKinTreeJoint* jointPtr() { return m_kinTreeJointPtr; }
        bool isRootJoint();
    };

    /**
    * Agent wrapper specific for the "MuJoCo" backend. It should handle ...
    * the abstract kintree initialized by the core functionality, and use it ...
    * to create the appropiate xml data needed for mujoco to compile the required models
    * (It works in this way, so it's ok. It's fine that it does everything at the start, ...
    *  maybe it does various optimizations when compiling models :D)
    */
    class TMjcKinTreeAgentWrapper : public TAgentWrapper
    {

        private :

        std::vector< TMjcBodyWrapper > m_bodyWrappers;
        std::vector< TMjcJointWrapper > m_jointWrappers;

        // mujoco data to be sent to the xml
        mjcf::GenericElement* m_mjcfResourcesPtr;
        // mjcf data where to send the data from above
        mjcf::GenericElement* m_mjcfTargetResourcesPtr;

        // mujoco simulation data
        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;

        // A flag to check if we already constructed a summary
        bool m_hasMadeSummary;

        void _createMjcResourcesFromKinTree();
        void _createMjcResourcesFromBodyNode( mjcf::GenericElement* parentElmPtr,
                                              TKinTreeBody* kinTreeBodyPtr );
        void _createMjcAssetsFromKinTree();
        void _createMjcSensorsFromKinTree();
        void _createMjcActuatorsFromKinTree();
        void _createMjcExclusionContactsFromKinTree();

        void _cacheBodyProperties( TKinTreeBody* kinTreeBody );
        void _cacheJointProperties( TKinTreeJoint* kinTreeJoints );

        TVec3 _extractMjcSizeFromStandardSize( const TShapeData& shape );

        protected :

        void _initializeInternal() override;
        void _resetInternal() override;
        void _preStepInternal() override;
        void _postStepInternal() override;

        public :

        TMjcKinTreeAgentWrapper( TAgent* agentPtr );
        ~TMjcKinTreeAgentWrapper();

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );
        void setMjcfTargetElm( mjcf::GenericElement* targetResourcesPtr );

        void finishedCreatingResources();

    };


    extern "C" TAgentWrapper* agent_createFromAbstract( TAgent* agentPtr );

    extern "C" TAgentWrapper* agent_createFromFile( const std::string& name,
                                                    const std::string& filename );

    extern "C" TAgentWrapper* agent_createFromId( const std::string& name,
                                                  const std::string& format,
                                                  const std::string& id );

}}