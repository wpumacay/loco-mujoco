
#include <mujoco_terrain_wrapper.h>


namespace tysoc {
namespace mujoco {


    TMjcTerrainGenWrapper::TMjcTerrainGenWrapper( terrain::TITerrainGenerator* terrainGenPtr,
                                                  const std::string& workingDir )
        : TTerrainGenWrapper( terrainGenPtr, workingDir )
    {
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
        m_mjcfTargetResourcesPtr = NULL;

        // Create resources that will be fixed|static (no pool needed)
        _collectStaticFromGenerator();

        // Create resources that will be reused *********************************
        for ( size_t i = 0; i < MJC_TERRAIN_POOL_SIZE; i++ )
        {
            auto _mjcPrimitive = new TMjcTerrainPrimitive();
            auto _name = std::string( "tGen_" ) + name() + 
                         std::string( "_" ) + std::to_string( i + m_mjcTerrainPrimitives.size() );

            _mjcPrimitive->mjcBodyId        = -1;
            _mjcPrimitive->mjcGeomName      = _name;
            _mjcPrimitive->mjcGeomType      = "box";
            _mjcPrimitive->mjcGeomSize      = { 3, { 0.5f * MJC_TERRAIN_PATH_DEFAULT_WIDTH, 
                                                     0.5f * MJC_TERRAIN_PATH_DEFAULT_DEPTH, 
                                                     0.5f * MJC_TERRAIN_PATH_DEFAULT_TICKNESS } };

            _mjcPrimitive->tysocPrimitiveObj = NULL;

            m_mjcTerrainPrimitives.push_back( _mjcPrimitive );
            m_mjcAvailablePrimitives.push( _mjcPrimitive );
        }

        // collect starting info from generator
        _collectReusableFromGenerator();
        // **********************************************************************
    }

    TMjcTerrainGenWrapper::~TMjcTerrainGenWrapper()
    {
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
        m_mjcfTargetResourcesPtr = NULL;

        while ( !m_mjcAvailablePrimitives.empty() )
        {
            m_mjcAvailablePrimitives.pop();
        }

        while ( !m_mjcWorkingPrimitives.empty() )
        {
            m_mjcWorkingPrimitives.pop();
        }

        for ( size_t i = 0; i < m_mjcTerrainPrimitives.size(); i++ )
        {
            delete m_mjcTerrainPrimitives[i];
            m_mjcTerrainPrimitives[i] = NULL;
        }
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
            bool _isStatic = ( ( _mjcPrimitivePtr->tysocPrimitiveObj != NULL ) && 
                               ( _mjcPrimitivePtr->tysocPrimitiveObj->type == terrain::TERRAIN_TYPE_STATIC ) );

            if ( _isStatic )
            {
                // Grab the primitive that is being wrapped
                auto _tysocPrimitiveObj = _mjcPrimitivePtr->tysocPrimitiveObj;
                // extract the rotation matrix @TODO: Change float[9] byt TMat3 (T_T)
                auto _rotmat = TMat3();
                for ( size_t i = 0; i < 9; i++ )
                    _rotmat.buff[i] = _tysocPrimitiveObj->rotmat[i];

                _position = { _tysocPrimitiveObj->pos.x,
                              _tysocPrimitiveObj->pos.y,
                              _tysocPrimitiveObj->pos.z };
                _orientation = TMat3::toQuaternion( _rotmat );
            }
            else
            {
                _position = { 0.0f, 0.0f, 100.0f + i * ( MJC_TERRAIN_PATH_DEFAULT_TICKNESS + 1.0f ) };
                _orientation = { 1.0f, 0.0f, 0.0f, 0.0f };
            }

            auto _bodyElm = mjcf::createBody( _mjcPrimitivePtr->mjcGeomName,
                                              _position,
                                              _orientation );

            auto _geomElm = mjcf::createGeometry( _mjcPrimitivePtr->mjcGeomName,
                                                  _mjcPrimitivePtr->mjcGeomType,
                                                  _mjcPrimitivePtr->mjcGeomSize );

            _geomElm->setAttributeInt( "contype", 0 );
            _geomElm->setAttributeInt( "conaffinity", 1 );
            //_geomElm->setAttributeVec3( "friction", { 0.7f, 0.1f, 0.1f } );

            _bodyElm->children.push_back( _geomElm );
            _worldbody->children.push_back( _bodyElm );
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

            if ( m_mjcTerrainPrimitives[i]->tysocPrimitiveObj->type == terrain::TERRAIN_TYPE_STATIC )
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
                                           _primitiveObj->rotmat );

        utils::changeSize( m_mjcModelPtr,
                            mjcTerrainPritimivePtr->mjcGeomName,
                            { 0.5f * _primitiveObj->size.x, 0.5f * _primitiveObj->size.y, 0.5f * _primitiveObj->size.z } );

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

            _primitiveObj->color.r = _color[0];
            _primitiveObj->color.g = _color[1];
            _primitiveObj->color.b = _color[2];
        }
    }

    void TMjcTerrainGenWrapper::_wrapStaticPrimitive( terrain::TTerrainPrimitive* primitivePtr )
    {
        // Create a new primitive for each static primitive (these do not change)
        auto _mjcPrimitive = new TMjcTerrainPrimitive();
        auto _mjcName = std::string( "tGen_" ) + 
                        name() + 
                        std::string( "_" ) + 
                        std::to_string( m_mjcTerrainPrimitives.size() );

        _mjcPrimitive->mjcBodyId            = -1;
        _mjcPrimitive->mjcGeomName          = _mjcName;
        _mjcPrimitive->mjcGeomType          = primitivePtr->geomType;
        _mjcPrimitive->mjcGeomSize          = { 3, { primitivePtr->size.x,
                                                     primitivePtr->size.y,
                                                     primitivePtr->size.z } };
        _mjcPrimitive->mjcGeomFilename      = primitivePtr->filename;
        _mjcPrimitive->tysocPrimitiveObj    = primitivePtr;

        m_mjcTerrainPrimitives.push_back( _mjcPrimitive );
    }

    void TMjcTerrainGenWrapper::_wrapReusablePrimitive( terrain::TTerrainPrimitive* primitivePtr )
    {
        // if the pool is empty, force to recycle the last object
        if ( m_mjcAvailablePrimitives.empty() )
        {
            auto _oldest = m_mjcWorkingPrimitives.front();
            if ( _oldest->tysocPrimitiveObj )
            {
                m_terrainGenPtr->recycle( _oldest->tysocPrimitiveObj );
                _oldest->tysocPrimitiveObj = NULL;
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


    extern "C" TTerrainGenWrapper* terrain_createFromAbstract( terrain::TITerrainGenerator* terrainGenPtr,
                                                               const std::string& workingDir )
    {
        return new TMjcTerrainGenWrapper( terrainGenPtr, workingDir );
    }

    extern "C" TTerrainGenWrapper* terrain_createFromParams( const std::string& name,
                                                             const TGenericParams& params,
                                                             const std::string& workingDir )
    {
        return NULL;
    }


}}