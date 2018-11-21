
#include <tysocMjcAgent.h>




namespace tysocMjc
{


    TMjcAgentWrapper::TMjcAgentWrapper( const std::string& name,
                                        mjcf::GenericElement* modelElmPtr,
                                        float posX, float posY, float posZ )
    {
        m_name = name;
        m_modelElmPtr = modelElmPtr;
        m_startX = posX;
        m_startY = posY;
        m_startZ = posZ;

        // create wrapped agent object
        _createWrappedAgentObj();
    }

    TMjcAgentWrapper::~TMjcAgentWrapper()
    {
        if ( m_agentPtr )
        {
            // deletion of the base reosurces is in charge of the scenario
            m_agentPtr = NULL;
        }

        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
    }

    void TMjcAgentWrapper::setMjcModel( mjModel* mjcModelPtr )
    {
        m_mjcModelPtr   = mjcModelPtr;
    }

    void TMjcAgentWrapper::setMjcData( mjData* mjcDataPtr )
    {
        m_mjcDataPtr    = mjcDataPtr;
    }

    void TMjcAgentWrapper::setMjcScene( mjvScene* mjcScenePtr )
    {
        m_mjcScenePtr   = mjcScenePtr;
    }

    std::string TMjcAgentWrapper::name()
    {
        return m_name;
    }

    void TMjcAgentWrapper::_createWrappedAgentObj()
    {
        m_agentPtr = new tysocagent::TAgent( m_name );

        auto _worldBodyElm = mjcf::findFirstChildByType( m_modelElmPtr, "worldbody" );
        auto _actuatorsElm = mjcf::findFirstChildByType( m_modelElmPtr, "actuator" );
        // @TODO: Adde sensors as well

        // collect bodies, geometries and joints from the worldbody
        _collectBodyGeometryOrJoint( _worldBodyElm );
        _collectActuator( _actuatorsElm );
    }

    void TMjcAgentWrapper::_collectBodyGeometryOrJoint( mjcf::GenericElement* elm )
    {
        if ( elm->etype == "body" )
        {
            // collect body
            auto _bname = elm->getAttributeString( "name" );

            m_agentPtr->addBody( _bname );

            // @HACK: for floor and root bodies, change the
            // starting position to the one given by the user
            // @TODO: This should be removed by the initialization ...
            // using the mjcint helper functions
            if ( _bname.find( "root" ) != std::string::npos )
            {
                elm->setAttributeVec3( "pos", { m_startX, m_startY, m_startZ } );
            }
        }
        else if ( elm->etype == "geom" )
        {
            // collect geometry
            auto _gname = elm->getAttributeString( "name" );
            auto _gtype = elm->getAttributeString( "type" );
            auto _gsize = elm->getAttributeArrayFloat( "size" );

            // @CHECK
            float _size[3] = { _gsize.buff[0], _gsize.buff[1], _gsize.buff[2] };
            m_agentPtr->addGeom( _gname, _gtype, _size );

            // @HACK: for floor and root bodies, change the
            // starting position to the one given by the user
            // @TODO: This should be removed by the initialization ...
            // using the mjcint helper functions
            if ( _gname.find( "floor" ) != std::string::npos )
            {
                elm->setAttributeVec3( "pos", { m_startX, m_startY, 0.0f } );
            }
        }
        else if ( elm->etype == "joint" )
        {
            // collect joint
            auto _jname = elm->getAttributeString( "name" );
            auto _jtype = elm->getAttributeString( "type" );

            m_agentPtr->addJoint( _jname, _jtype );
        }

        for ( size_t i = 0; i < elm->children.size(); i++ )
        {
            _collectBodyGeometryOrJoint( elm->children[i] );
        }
    }

    void TMjcAgentWrapper::_collectActuator( mjcf::GenericElement* elm )
    {
        if ( elm->etype == "motor" )
        {
            // collect body
            auto _aname             = elm->getAttributeString( "name" );
            auto _atype             = "motor";
            auto _ajointLinkedName  = elm->getAttributeString( "joint" );

            m_agentPtr->addActuator( _aname, 
                                     _atype, 
                                     _ajointLinkedName );
        }

        for ( size_t i = 0; i < elm->children.size(); i++ )
        {
            _collectActuator( elm->children[i] );
        }
    }

    void TMjcAgentWrapper::injectMjcResources( mjcf::GenericElement* root )
    {
        auto _worldBodyElm = mjcf::findFirstChildByType( m_modelElmPtr, "worldbody" );
        auto _actuatorsElm = mjcf::findFirstChildByType( m_modelElmPtr, "actuator" );

        // actuators also link to the joint name, so we should replace that too
        mjcf::replaceNameRecursive( _actuatorsElm, m_name, "joint" );

        root->children.push_back( _worldBodyElm );
        root->children.push_back( _actuatorsElm );
    }

    void TMjcAgentWrapper::preStep()
    {
        // set all ctrls from the agent object actuator values ( from user )
        auto _actuators = m_agentPtr->actuators();

        for ( auto it = _actuators.begin(); it != _actuators.end(); it++ )
        {
            mjcint::setActuatorCtrl( m_mjcModelPtr,
                                     m_mjcDataPtr,
                                     it->second->name,
                                     it->second->ctrlValue );
        }
    }

    void TMjcAgentWrapper::postStep()
    {
        auto _geometries = m_agentPtr->geometries();

        for ( auto it = _geometries.begin(); it != _geometries.end(); it++ )
        {
            float _pos[3];
            float _rotmat[9];
            mjcint::getGeometryTransform( m_mjcModelPtr, 
                                          m_mjcScenePtr,
                                          it->second->name,
                                          _pos,
                                          _rotmat );

            it->second->pos = { _pos[0], _pos[1], _pos[2] };

            it->second->rotmat[0] = _rotmat[0];
            it->second->rotmat[1] = _rotmat[1];
            it->second->rotmat[2] = _rotmat[2];
            it->second->rotmat[3] = _rotmat[3];
            it->second->rotmat[4] = _rotmat[4];
            it->second->rotmat[5] = _rotmat[5];
            it->second->rotmat[6] = _rotmat[6];
            it->second->rotmat[7] = _rotmat[7];
            it->second->rotmat[8] = _rotmat[8];
        }

        // do nothing with the joints, seems we need sensors here instead
        // auto _joints = m_agentPtr->joints();

        auto _bodies = m_agentPtr->bodies();

        for ( auto it = _bodies.begin(); it != _bodies.end(); it++ )
        {
            auto _pos = mjcint::getBodyPosition( m_mjcModelPtr, it->second->name );

            it->second->pos = { _pos.x, _pos.y, _pos.z };
        }
    }

}