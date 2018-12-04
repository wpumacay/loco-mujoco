
#include <tysocMjcKinTreeAgent.h>

namespace tysoc {
namespace agent {


    TMjcKinTreeAgentWrapper::TMjcKinTreeAgentWrapper( const std::string& name,
                                                      mjcf::GenericElement* templateModelElmPtr,
                                                      const TVec3& position )
    {
        // grab the name
        m_name = name;
        // and create a fresh mjcf resource element
        m_modelElmPtr = new mjcf::GenericElement();
        // and copy all contents from the template element
        mjcf::deepCopy( m_modelElmPtr, templateModelElmPtr );
        // and replace the ### placeholder with the name of this agent
        mjcf::replaceNameRecursive( m_modelElmPtr, m_name, "name" );// all name attribs
        mjcf::replaceNameRecursive( m_modelElmPtr, m_name, "joint" );// all joint attribs (actuators)
        mjcf::replaceNameRecursive( m_modelElmPtr, m_name, "target" );// all target attribs (cameras)
        mjcf::replaceNameRecursive( m_modelElmPtr, m_name, "body" );// all body attribs (some sensors)
        mjcf::replaceNameRecursive( m_modelElmPtr, m_name, "site" );// all site attribs (some sensors)
        mjcf::replaceNameRecursive( m_modelElmPtr, m_name, "objname");// all objname attribs (some sensors)
        // and set default to make not wild undefined pointers
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;

        // and create the kintree agent that uses mjcf
        m_kinTreeAgentPtr = new TAgentKinTreeMjcf( name, position, m_modelElmPtr );
    }

    TMjcKinTreeAgentWrapper::~TMjcKinTreeAgentWrapper()
    {
        if ( m_modelElmPtr )
        {
            delete m_modelElmPtr;
            m_modelElmPtr = NULL;
        }

        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
    }

    void TMjcKinTreeAgentWrapper::setMjcModel( mjModel* mjcModelPtr )
    {
        m_mjcModelPtr = mjcModelPtr;
    }

    void TMjcKinTreeAgentWrapper::setMjcData( mjData* mjcDataPtr )
    {
        m_mjcDataPtr = mjcDataPtr;
    }

    void TMjcKinTreeAgentWrapper::setMjcScene( mjvScene* mjcScenePtr )
    {
        m_mjcScenePtr = mjcScenePtr;
    }

    std::string TMjcKinTreeAgentWrapper::name()
    {
        return m_name;
    }

    tysoc::agent::TAgentKinTree* TMjcKinTreeAgentWrapper::agent()
    {
        return m_kinTreeAgentPtr;
    }

    void TMjcKinTreeAgentWrapper::injectMjcResources( mjcf::GenericElement* root )
    {
        // grab the resources to inject, namely the worldbody
        auto _worldBodyElmPtr = mjcf::findFirstChildByType( m_modelElmPtr, "worldbody" );
        // and the actuators as well
        auto _actuatorsElmPtr = mjcf::findFirstChildByType( m_modelElmPtr, "actuator" );
        // @TODO: should also place custom sensors (for all joints)

        // then, just add them to the children of the root
        root->children.push_back( _worldBodyElmPtr );
        root->children.push_back( _actuatorsElmPtr );
        // @TODO: should also place custom sensors (for all joints)
    }

    void TMjcKinTreeAgentWrapper::preStep()
    {

    }

    void TMjcKinTreeAgentWrapper::postStep()
    {
        
    }

}}