
#include <mujoco_terrain_wrapper.h>

namespace tysoc {
namespace mujoco {


    TMjcTerrainGenWrapper::TMjcTerrainGenWrapper( TITerrainGenerator* terrainGenPtr )
        : TTerrainGenWrapper( terrainGenPtr )
    {
        m_mjcModelPtr   = nullptr;
        m_mjcDataPtr    = nullptr;
        m_mjcScenePtr   = nullptr;
        m_mjcfTargetResourcesPtr = nullptr;

        // Create resources that will be fixed|static (no pool needed)
        _collectStaticFromGenerator();

        // @TODO: Commented due to some compatibility we wanted to try out with ...
        // the statis terrain generators and the mujoco-py environments. The ...
        // problem was that those environments extract vectorized information ...
        // using directly the mjData structure, and if we enabled pooling, some ...
        // information would go into this vectorized information grabbed from ...
        // mjData (like mjData->qpos, etc.). We will fix this issue later by ...
        // filtering out the non-working primitives (which remain in the pool) ...
        // and pass the information required only from the active primities.

//        // Create resources that will be reused *********************************
//        for ( size_t i = 0; i < MJC_TERRAIN_POOL_SIZE; i++ )
//        {
//            auto _mjcPrimitive = new TMjcTerrainPrimitive();
//            auto _name = std::string( "tGen_" ) + name() + 
//                         std::string( "_" ) + std::to_string( i + m_mjcTerrainPrimitives.size() );
//
//            _mjcPrimitive->mjcBodyId        = -1;
//            _mjcPrimitive->mjcGeomName      = _name;
//            _mjcPrimitive->mjcGeomType      = "box";
//            _mjcPrimitive->mjcGeomSize      = { 0.5f * MJC_TERRAIN_PATH_DEFAULT_WIDTH, 
//                                                0.5f * MJC_TERRAIN_PATH_DEFAULT_DEPTH, 
//                                                0.5f * MJC_TERRAIN_PATH_DEFAULT_TICKNESS };
//
//            _mjcPrimitive->tysocPrimitiveObj = nullptr;
//
//            m_mjcTerrainPrimitives.push_back( _mjcPrimitive );
//            m_mjcAvailablePrimitives.push( _mjcPrimitive );
//        }
//
//        // collect starting info from generator
//        _collectReusableFromGenerator();
//        // **********************************************************************
    }

    TMjcTerrainGenWrapper::~TMjcTerrainGenWrapper()
    {
        m_mjcModelPtr   = nullptr;
        m_mjcDataPtr    = nullptr;
        m_mjcScenePtr   = nullptr;
        m_mjcfTargetResourcesPtr = nullptr;

        while ( !m_mjcAvailablePrimitives.empty() )
            m_mjcAvailablePrimitives.pop();

        while ( !m_mjcWorkingPrimitives.empty() )
            m_mjcWorkingPrimitives.pop();

        for ( auto _terrainWrapper : m_mjcTerrainPrimitives )
            delete _terrainWrapper;

        m_mjcTerrainPrimitives.clear();
    }

    void TMjcTerrainGenWrapper::setMjcModel( mjModel* mjcModelPtr )
    {
        m_mjcModelPtr = mjcModelPtr;
    }

    void TMjcTerrainGenWrapper::setMjcData( mjData* mjcDataPtr )
    {
        m_mjcDataPtr = mjcDataPtr;
    }

    void TMjcTerrainGenWrapper::setMjcScene( mjvScene* mjcScenePtr )
    {
        m_mjcScenePtr = mjcScenePtr;
    }

    void TMjcTerrainGenWrapper::setMjcfTargetElm( mjcf::GenericElement* targetResourcesPtr )
    {
        m_mjcfTargetResourcesPtr = targetResourcesPtr;
    }

    void TMjcTerrainGenWrapper::_initializeInternal()
    {
        // Check if the caller (TMjcSimulation) set the target reference
        if ( !m_mjcfTargetResourcesPtr )
        {
            std::cout << "ERROR> mjc-sim object must pass a reference of the"
                      << " target resources to this terrain generator" << std::endl;
            return;
        }

        // create a worldbody element to inject into the targetResources element
        auto _worldbody = new mjcf::GenericElement( "worldbody" );

        // add all geometry resources into this element
        for ( size_t i = 0; i < m_mjcTerrainPrimitives.size(); i++ )
        {
            auto _mjcPrimitivePtr = m_mjcTerrainPrimitives[i];
            auto _position = TVec3();
            auto _orientation = TVec4();
            bool _isStatic = ( ( _mjcPrimitivePtr->tysocPrimitiveObj != nullptr ) && 
                               ( _mjcPrimitivePtr->tysocPrimitiveObj->type == TERRAIN_TYPE_STATIC ) );

            if ( _isStatic )
            {
                _position = _mjcPrimitivePtr->tysocPrimitiveObj->pos;
                _orientation = TMat3::toQuaternion( _mjcPrimitivePtr->tysocPrimitiveObj->rotmat );
            }
            else
            {
                _position = { 0.0f, 0.0f, 100.0f + i * ( MJC_TERRAIN_PATH_DEFAULT_TICKNESS + 1.0f ) };
                _orientation = { 1.0f, 0.0f, 0.0f, 0.0f };
            }

            //auto _bodyElm = mjcf::createBody( _mjcPrimitivePtr->mjcGeomName,
            //                                  _position,
            //                                  _orientation );

            auto _geomElm = mjcf::createGeometry( _mjcPrimitivePtr->mjcGeomName,
                                                  _mjcPrimitivePtr->mjcGeomType,
                                                  vec3ToSizef( _mjcPrimitivePtr->mjcGeomSize ),
                                                  0.0f,
                                                  _position,
                                                  _orientation );

            _geomElm->setAttributeInt( "contype", 0 );
            _geomElm->setAttributeInt( "conaffinity", 1 );
            _geomElm->setAttributeVec3( "friction", { 0.7f, 0.1f, 0.1f } );

            //_bodyElm->children.push_back( _geomElm );
            //_worldbody->children.push_back( _bodyElm );
            _worldbody->children.push_back( _geomElm );
        }

        m_mjcfTargetResourcesPtr->children.push_back( _worldbody );
    }

    void TMjcTerrainGenWrapper::_resetInternal()
    {
        // @TODO: add reset functionality
    }

    void TMjcTerrainGenWrapper::_preStepInternal()
    {
        _collectReusableFromGenerator();

        // update the properties of all objects (if they have a primitive as reference)
        for ( size_t i = 0; i < m_mjcTerrainPrimitives.size(); i++ )
        {
            if ( !m_mjcTerrainPrimitives[i]->tysocPrimitiveObj )
                return;

            if ( m_mjcTerrainPrimitives[i]->tysocPrimitiveObj->type == TERRAIN_TYPE_STATIC )
                return;

            _updateProperties( m_mjcTerrainPrimitives[i] );
        }
    }

    void TMjcTerrainGenWrapper::_postStepInternal()
    {
        // @TODO: should contact checking be done here? (store contacts)
    }

    void TMjcTerrainGenWrapper::_collectReusableFromGenerator()
    {
        auto _newPrimitivesQueue = m_terrainGenPtr->getJustCreated();

        while ( !_newPrimitivesQueue.empty() )
        {
            auto _newPrimitive = _newPrimitivesQueue.front();
            _newPrimitivesQueue.pop();

            _wrapReusablePrimitive( _newPrimitive );
        }

        // flush the creation queue to avoid double references everywhere
        m_terrainGenPtr->flushJustCreatedQueue();
    }

    void TMjcTerrainGenWrapper::_collectStaticFromGenerator()
    {
        auto _fixedPrimitivesQueue = m_terrainGenPtr->getFixed();

        while ( !_fixedPrimitivesQueue.empty() )
        {
            auto _fixedPrimitive = _fixedPrimitivesQueue.front();
            _fixedPrimitivesQueue.pop();

            _wrapStaticPrimitive( _fixedPrimitive );
        }

        // flush the fixed queue to avoid double references (the wrapper now holds the ref.)
        m_terrainGenPtr->flushFixedQueue();
    }

    void TMjcTerrainGenWrapper::_updateProperties( TMjcTerrainPrimitive* mjcTerrainPritimivePtr )
    {
        auto _primitiveObj = mjcTerrainPritimivePtr->tysocPrimitiveObj;

        utils::setTerrainBodyPosition( m_mjcModelPtr, 
                                        m_mjcDataPtr,
                                        mjcTerrainPritimivePtr->mjcGeomName,
                                        { _primitiveObj->pos.x, _primitiveObj->pos.y, _primitiveObj->pos.z } );

        utils::setTerrainBodyOrientation( m_mjcModelPtr,
                                           m_mjcDataPtr,
                                           mjcTerrainPritimivePtr->mjcGeomName,
                                           _primitiveObj->rotmat.buff );

        auto _mjcSize = _extractMjcSizeFromStandardSize( _primitiveObj->geomType,
                                                         { _primitiveObj->size.x,
                                                           _primitiveObj->size.y,
                                                           _primitiveObj->size.z } );

        // @TODO: Change to use TVec3s instead of TSizefs T_T
        utils::changeSize( m_mjcModelPtr,
                           mjcTerrainPritimivePtr->mjcGeomName,
                           _mjcSize );

        utils::setRbound( m_mjcModelPtr,
                           mjcTerrainPritimivePtr->mjcGeomName,
                           _primitiveObj->rbound );

        if ( !_primitiveObj->useCustomColor )
        {
            float _color[3];
            utils::getGeometryColor( m_mjcModelPtr,
                                      m_mjcScenePtr,
                                      mjcTerrainPritimivePtr->mjcGeomName,
                                      _color );

            _primitiveObj->color.x = _color[0];
            _primitiveObj->color.y = _color[1];
            _primitiveObj->color.z = _color[2];
        }
    }

    void TMjcTerrainGenWrapper::_wrapStaticPrimitive( TTerrainPrimitive* primitivePtr )
    {
        // Create a new primitive for each static primitive (these do not change)
        auto _mjcPrimitive = new TMjcTerrainPrimitive();
        auto _mjcName = std::string( "tGen_" ) + 
                        name() + 
                        std::string( "_" ) + 
                        std::to_string( m_mjcTerrainPrimitives.size() );

        auto _mjcSize = _extractMjcSizeFromStandardSize( primitivePtr->geomType,
                                                         { primitivePtr->size.x,
                                                           primitivePtr->size.y,
                                                           primitivePtr->size.z } );

        _mjcPrimitive->mjcBodyId            = -1;
        _mjcPrimitive->mjcGeomName          = _mjcName;
        _mjcPrimitive->mjcGeomType          = primitivePtr->geomType;
        _mjcPrimitive->mjcGeomSize          = _mjcSize;
        _mjcPrimitive->mjcGeomFilename      = primitivePtr->filename;
        _mjcPrimitive->tysocPrimitiveObj    = primitivePtr;

        m_mjcTerrainPrimitives.push_back( _mjcPrimitive );
    }

    void TMjcTerrainGenWrapper::_wrapReusablePrimitive( TTerrainPrimitive* primitivePtr )
    {
        // if the pool is empty, force to recycle the last object
        if ( m_mjcAvailablePrimitives.empty() )
        {
            auto _oldest = m_mjcWorkingPrimitives.front();
            if ( _oldest->tysocPrimitiveObj )
            {
                m_terrainGenPtr->recycle( _oldest->tysocPrimitiveObj );
                _oldest->tysocPrimitiveObj = nullptr;
                m_mjcWorkingPrimitives.pop();
            }
            m_mjcAvailablePrimitives.push( _oldest );
        }

        // grab the oldest available object
        auto _mjcPrimitive = m_mjcAvailablePrimitives.front();
        m_mjcAvailablePrimitives.pop();

        // link an object from the working queue with the requested new primitive
        _mjcPrimitive->tysocPrimitiveObj = primitivePtr;

        // put it into the working queue
        m_mjcWorkingPrimitives.push( _mjcPrimitive );
    }

    TVec3 TMjcTerrainGenWrapper::_extractMjcSizeFromStandardSize( const std::string& shapeType,
                                                                  const TVec3& shapeSize )
    {
        TVec3 _res;

        if ( shapeType == "plane" )
        {
            _res = { 0.5f * shapeSize.x, 
                     0.5f * shapeSize.y,
                     0.5f * shapeSize.z };
        }
        else if ( shapeType == "sphere" )
        {
            _res = { shapeSize.x,
                     shapeSize.y,
                     shapeSize.z };
        }
        else if ( shapeType == "capsule" ||
                  shapeType == "cylinder" )
        {
            _res = { shapeSize.x, 
                     0.5f * shapeSize.y,
                     shapeSize.z };
        }
        else if ( shapeType == "box" )
        {
            _res = { 0.5f * shapeSize.x, 
                     0.5f * shapeSize.y, 
                     0.5f * shapeSize.z };
        }

        return _res;
    }

    extern "C" TTerrainGenWrapper* terrain_createFromAbstract( TITerrainGenerator* terrainGenPtr )
    {
        return new TMjcTerrainGenWrapper( terrainGenPtr );
    }

    extern "C" TTerrainGenWrapper* terrain_createFromParams( const std::string& name,
                                                             const TGenericParams& params )
    {
        return nullptr;
    }


}}