
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
        mjcf::replaceNameRecursive( m_modelElmPtr, m_name, "body1" );// for contacts - body1 tag
        mjcf::replaceNameRecursive( m_modelElmPtr, m_name, "body2" );// for contacts - body2 tag
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

    void TMjcKinTreeAgentWrapper::_injectMjcAssets( mjcf::GenericElement* root )
    {
        if ( !mjcf::findFirstChildByType( m_modelElmPtr, "asset" ) )
        {
            // no assets to get here
            return;
        }

        // grab the target element where we are going to place our assets
        auto _targetAssetsElmPtr    = mjcf::findFirstChildByType( root, "asset" );
        auto _assetsInTarget        = _targetAssetsElmPtr->children;
        // and grab the assets used by our model template
        auto _srcAssetsElmPtr   = mjcf::findFirstChildByType( m_modelElmPtr, "asset" );
        auto _assetsInSrc       = _srcAssetsElmPtr->children;

        // create a set out of the current elements in the target assets list
        std::map< std::string, mjcf::GenericElement* > _currentAssetsMap;
        for ( size_t i = 0; i < _assetsInTarget.size(); i++ )
        {
            auto _assetElmPtr   = _assetsInTarget[i];
            auto _assetElmName  = _assetsInTarget[i]->getAttributeString( "name" );

            _currentAssetsMap[ _assetElmName ] = _assetElmPtr;
        }

        // and place the src assets that are not already there
        for ( size_t i = 0; i < _assetsInSrc.size(); i++ )
        {
            auto _assetElmPtr   = _assetsInSrc[i];
            auto _assetElmName  = _assetsInSrc[i]->getAttributeString( "name" );

            if ( _currentAssetsMap.find( _assetElmName ) == _currentAssetsMap.end() )
            {
                // if not in the current assets map, the we can go ahead
                _targetAssetsElmPtr->children.push_back( _assetElmPtr );
            }
        }
    }

    void TMjcKinTreeAgentWrapper::_injectMjcContacts( mjcf::GenericElement* root )
    {
        if ( !mjcf::findFirstChildByType( m_modelElmPtr, "contact" ) )
        {
            // no contacts to grab here
            return;
        }

        // grab the target element where we are going to place our contacts
        auto _targetContactsElmPtr = mjcf::findFirstChildByType( root, "contact" );
        // and grab the contacts defined by our model template
        auto _srcContactsElmPtr = mjcf::findFirstChildByType( m_modelElmPtr, "contact" );

        // now place them inside the target contacts element
        auto _srcContacts = _srcContactsElmPtr->children;
        for ( size_t i = 0; i < _srcContacts.size(); i++ )
        {
            // @CHECK: I'm passing the pointer reference. Perhaps should pass a deep fresh copy
            _targetContactsElmPtr->children.push_back( _srcContacts[i] );
        }
    }

    // @CHECK: Maybe sensor should be created abstractly, and the backend should ...
    // update its internals on each prestep requested to the concrete kintreeagent
    void TMjcKinTreeAgentWrapper::_injectMjcSensors( mjcf::GenericElement* root )
    {
        // Generate for each joint an appropiate sensor, and ...
        // add it to the model element if it has one, or create it if not
        auto _srcSensorsElmPtr = mjcf::findFirstChildByType( m_modelElmPtr, "sensor" );
        if ( !_srcSensorsElmPtr )
        {
            _srcSensorsElmPtr = new mjcf::GenericElement( "sensor" );
            m_modelElmPtr->children.push_back( _srcSensorsElmPtr );
        }

        // create a brand new sensors-tag for this model sensor resources
        auto _targetSensorsElmPtr = new mjcf::GenericElement( "sensor" );
        // and add it to te root target model
        root->children.push_back( _targetSensorsElmPtr );
        // now, create the sensors and add them to both target and src
        // @CHECK: Same as above, perhaps should pass a fresh new copy
        auto _kinJoints = m_kinTreeAgentPtr->getKinTreeJoints();
        for ( size_t i = 0; i < _kinJoints.size(); i++ )
        {
            auto _jointMjcSensorResource = new mjcf::GenericElement( "jointpos" );
            // set the necessary properties
            _jointMjcSensorResource->setAttributeString( "name", std::string( "mjc_sensor_" ) + 
                                                                 m_kinTreeAgentPtr->getName() + 
                                                                 std::string( "_j_" ) +
                                                                 std::to_string( i ) );
            _jointMjcSensorResource->setAttributeString( "joint", _kinJoints[i]->name );
            // add this to the sensor element
            _srcSensorsElmPtr->children.push_back( _jointMjcSensorResource );
        }

        // finally, copy all sensor elements from src to target element
        for ( size_t i = 0; i < _srcSensorsElmPtr->children.size(); i++ )
        {
            _targetSensorsElmPtr->children.push_back( _srcSensorsElmPtr->children[i] );
        }
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

        // also, inject the assets used
        _injectMjcAssets( root );
        // as well as the contacts
        _injectMjcContacts( root );
        // and the sensors
        _injectMjcSensors( root );  
    }

    void TMjcKinTreeAgentWrapper::preStep()
    {
        auto _kinActuators = m_kinTreeAgentPtr->getKinTreeActuators();

        for ( size_t i = 0; i < _kinActuators.size(); i++ )
        {
            mjcint::setActuatorCtrl( m_mjcModelPtr,
                                     m_mjcDataPtr,
                                     _kinActuators[i]->name,
                                     _kinActuators[i]->ctrlValue );
        }
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

        // @TODO: and get the colors
        auto _kinVisuals = m_kinTreeAgentPtr->getKinTreeVisuals();
        for ( size_t i = 0; i < _kinVisuals.size(); i++ )
        {
            float _color[3];
            mjcint::getGeometryColor( m_mjcModelPtr,
                                      m_mjcScenePtr,
                                      _kinVisuals[i]->name,
                                      _color );

            _kinVisuals[i]->material.diffuse.x = _color[0];
            _kinVisuals[i]->material.diffuse.y = _color[1];
            _kinVisuals[i]->material.diffuse.z = _color[2];

            _kinVisuals[i]->material.specular.x = _color[0];
            _kinVisuals[i]->material.specular.y = _color[1];
            _kinVisuals[i]->material.specular.z = _color[2];
        }
    }

}}