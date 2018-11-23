
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
            if ( _bname.find( "tmjcroot" ) != std::string::npos )
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
            // needed for the glengine, but it should be ...
            // easily used for other engines as well
            float _size[3] = { 0.0f, 0.0f, 0.0f };
            _extractStandardSize( elm, _size );
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

    void TMjcAgentWrapper::_extractStandardSize( mjcf::GenericElement* geomElm,
                                                 float* targetSize )
    {
        auto _gname     = geomElm->getAttributeString( "name" );
        auto _gtype     = geomElm->getAttributeString( "type" );
        auto _gsize     = geomElm->getAttributeArrayFloat( "size" );
        auto _gfromto   = geomElm->getAttributeArrayFloat( "fromto" );

        if ( _gtype == "plane" )
        {
            // should normalize sizes accordingly
            float _width, _depth;
            if ( _gsize.ndim == 0 )
            {
                // This is weird, but just in case make some default dimenions
                std::cout << "WARNING> the plane " <<  _gname << " has no size :/" << std::endl;
                _width = 3.0f;
                _depth = 3.0f;
            }
            else if ( _gsize.ndim == 1 )
            {
                // the dimensions should be repeated
                _width = _depth = _gsize.buff[0];
            }
            else if ( _gsize.ndim >= 2 )
            {
                // each dimensiones has a separate field
                _width = _gsize.buff[0];
                _depth = _gsize.buff[1];
                // third is spacing
            }

            targetSize[0] = _width;
            targetSize[1] = _depth;
        }
        else if ( _gtype == "sphere" )
        {
            float _radius;

            if ( _gsize.ndim == 0 )
            {
                // This is weird, but just in case make some default dimenions
                std::cout << "WARNING> the sphere " << _gname  << " has no size :/" << std::endl;
                _radius = 0.1f;
            }
            else if ( _gsize.ndim == 1 )
            {
                _radius = _gsize.buff[0];
            }
            else
            {
                // just in case, if someone passed more parameters than needed
                // it's like ... "thanks, but no thanks xD", so just use the first two
                std::cout << "WARNING> the sphere "<< _gname << " has more parameters than needed" << std::endl;
                _radius = _gsize.buff[0];
            }

            targetSize[0] = _radius;
        }
        else if ( _gtype == "capsule" ||
                  _gtype == "cylinder" )
        {
            float _radius, _length;

            // This one is a bit trick, because it might have a fromto
            if ( _gfromto.ndim == 6 ) // use fromto
            {
                /*   _____________________
                *   |                     |
                *  |  s                 e  |
                *   |_____________________|
                */
                // first 3 are start point (s)
                float _x1 = _gfromto.buff[0];
                float _y1 = _gfromto.buff[1];
                float _z1 = _gfromto.buff[2];
                // second 3 are the end point (e)
                float _x2 = _gfromto.buff[3];
                float _y2 = _gfromto.buff[4];
                float _z2 = _gfromto.buff[5];
                // get the length of the capsule (the space orientation is ...
                // computed from the scene, so we just use that one)
                float _dx = ( _x2 - _x1 );
                float _dy = ( _y2 - _y1 );
                float _dz = ( _z2 - _z1 );
                _length = sqrtf( _dx * _dx + _dy * _dy + _dz * _dz );

                // get the radius from the size param
                if ( _gsize.ndim >= 1 )
                {
                    _radius = _gsize.buff[0];
                }
                else
                {
                    std::cout << "WARNING> the capsule/cylinder " << _gname << " has wrong size dim for fromto" << std::endl;
                    _radius = 0.25f * _length;
                }
            }
            else
            {
                if ( _gsize.ndim == 2 )
                {
                    _radius = _gsize.buff[0];
                    _length = 2.0f * _gsize.buff[1];
                }
                else
                {
                    // default, just in case passed less than (radius,length)
                    std::cout << "WARNING> the capsule/cylinder " << _gname << " has wrong size dim" << std::endl;
                    _radius = 0.05f;
                    _length = 0.1f;
                }
            }

            targetSize[0] = _radius;
            targetSize[1] = _length;
        }
        else if ( _gtype == "box" )
        {
            float _hwidth, _hdepth, _hheight;

            if ( _gsize.ndim == 3 )
            {
                _hwidth  = _gsize.buff[0];
                _hdepth  = _gsize.buff[1];
                _hheight = _gsize.buff[2];
            }
            else
            {
                std::cout << "WARNING> the box " << _gname << " has wrong dims for creation" << std::endl;
                _hwidth = _hdepth = _hheight = 0.1f;
            }

            targetSize[0] = 2 * _hwidth;
            targetSize[1] = 2 * _hdepth;
            targetSize[2] = 2 * _hheight;
        }
    }

    void TMjcAgentWrapper::injectMjcResources( mjcf::GenericElement* root )
    {
        auto _worldBodyElm = mjcf::findFirstChildByType( m_modelElmPtr, "worldbody" );
        auto _actuatorsElm = mjcf::findFirstChildByType( m_modelElmPtr, "actuator" );

        // actuators also link to the joint name, so we should replace that too
        mjcf::replaceNameRecursive( _actuatorsElm, m_name, "joint" );
        mjcf::replaceNameRecursive( _worldBodyElm, m_name, "target" );

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

            float _color[3];
            mjcint::getGeometryColor( m_mjcModelPtr,
                                      m_mjcScenePtr,
                                      it->second->name,
                                      _color );

            it->second->color.r = _color[0];
            it->second->color.g = _color[1];
            it->second->color.b = _color[2];
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