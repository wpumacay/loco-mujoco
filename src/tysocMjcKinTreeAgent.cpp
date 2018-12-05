
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

    bool TMjcKinTreeAgentWrapper::_findAndReplaceRootStartingPos( mjcf::GenericElement* elmPtr )
    {
        if ( elmPtr->etype == "body" )
        {
            auto _bname = elmPtr->getAttributeString( "name" );
            if ( _bname.find( "tmjcroot" ) != std::string::npos )
            {
                auto _pos = m_kinTreeAgentPtr->getPosition();
                elmPtr->setAttributeVec3( "pos", 
                                          { _pos.x, _pos.y, _pos.z } );
                return true;
            }
        }

        for ( size_t i = 0; i < elmPtr->children.size(); i++ )
        {
            if ( _findAndReplaceRootStartingPos( elmPtr->children[i] ) )
            {
                return true;
            }
        }

        return false;
    }

    void TMjcKinTreeAgentWrapper::injectMjcResources( mjcf::GenericElement* root )
    {
        // grab the resources to inject, namely the worldbody
        auto _worldBodyElmPtr = mjcf::findFirstChildByType( m_modelElmPtr, "worldbody" );
        // and set the starting position
        _findAndReplaceRootStartingPos( _worldBodyElmPtr );

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
        auto _kinBodies = m_kinTreeAgentPtr->getKinTreeBodies();
        for ( size_t i = 0; i < _kinBodies.size(); i++ )
        {
            // grab the position from the mujoco backend
            auto _pos = mjcint::getBodyPosition( m_mjcModelPtr,
                                                 m_mjcDataPtr,
                                                 _kinBodies[i]->name );
            // and the rotation as well
            float _rot[9];
            mjcint::getBodyOrientation( m_mjcModelPtr,
                                        m_mjcDataPtr,
                                        _kinBodies[i]->name, _rot );

            // convert the position/rotation data to our format
            TVec3 _position;
            TMat3 _rotation;

            _position.x = _pos.x;
            _position.y = _pos.y;
            _position.z = _pos.z;

            _rotation.buff[0] = _rot[0];
            _rotation.buff[1] = _rot[1];
            _rotation.buff[2] = _rot[2];
            _rotation.buff[3] = _rot[3];
            _rotation.buff[4] = _rot[4];
            _rotation.buff[5] = _rot[5];
            _rotation.buff[6] = _rot[6];
            _rotation.buff[7] = _rot[7];
            _rotation.buff[8] = _rot[8];

            // then set it to the body's worldtransform
            _kinBodies[i]->worldTransform.setPosition( _position );
            _kinBodies[i]->worldTransform.setRotation( _rotation );
        }

        // and then request an update of the kintree
        m_kinTreeAgentPtr->update( 0 );
    }

}}