
#include <tysocMjcTerrain.h>


namespace tysoc {
namespace mujoco {


    TMjcTerrainGenWrapper::TMjcTerrainGenWrapper( const std::string& name,
                                                  tysoc::terrain::TTerrainGenerator* terrainGenPtr )
    {
        m_name          = name;
        m_terrainGenPtr = terrainGenPtr;
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;

        // initialize mujoco resources
        for ( size_t i = 0; i < MJC_TERRAIN_POOL_SIZE; i++ )
        {
            auto _mjcPrimitive = new TMjcTerrainPrimitive();

            _mjcPrimitive->mjcBodyId        = -1;
            _mjcPrimitive->mjcBodyName      = std::string( "terrainGen_" ) + name + std::string( "_" ) + std::to_string( i );
            _mjcPrimitive->mjcGeomType      = "box";
            _mjcPrimitive->mjcGeomSize      = { 3, { 0.5f * MJC_TERRAIN_PATH_DEFAULT_WIDTH, 
                                                     0.5f * MJC_TERRAIN_PATH_DEFAULT_DEPTH, 
                                                     0.5f * MJC_TERRAIN_PATH_DEFAULT_TICKNESS } };
            _mjcPrimitive->isAvailable      = true;

            _mjcPrimitive->tysocPrimitiveObj = NULL;

            m_mjcTerrainPrimitives.push_back( _mjcPrimitive );
            m_mjcAvailablePrimitives.push( _mjcPrimitive );
        }
    }

    TMjcTerrainGenWrapper::~TMjcTerrainGenWrapper()
    {
        if ( m_terrainGenPtr )
        {
            // deletion of the base reosurces is in charge of the scenario
            m_terrainGenPtr = NULL;
        }

        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;

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

    void TMjcTerrainGenWrapper::initialize()
    {
        // collect starting info from generator
        _collectFromGenerator();
        _collectFixedFromGenerator();
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

    void TMjcTerrainGenWrapper::injectMjcResources( mjcf::GenericElement* root )
    {
        // create a worldbody element to inject into the root element
        auto _worldbody = new mjcf::GenericElement( "worldbody" );

        // add all geometry resources into this element
        for ( size_t i = 0; i < m_mjcTerrainPrimitives.size(); i++ )
        {
            auto _bodyElm = mjcf::_createBody( m_mjcTerrainPrimitives[i]->mjcBodyName,
                                               { 0.0f,
                                                 0.0f,
                                                 100.0f + i * ( MJC_TERRAIN_PATH_DEFAULT_TICKNESS + 1.0f ) } );

            auto _geomElm = mjcf::_createGeometry( m_mjcTerrainPrimitives[i]->mjcBodyName,
                                                   m_mjcTerrainPrimitives[i]->mjcGeomType,
                                                   m_mjcTerrainPrimitives[i]->mjcGeomSize,
                                                   1.0f );
            _geomElm->setAttributeInt( "contype", 0 );
            _geomElm->setAttributeInt( "conaffinity", 1 );

            _bodyElm->children.push_back( _geomElm );
            _worldbody->children.push_back( _bodyElm );
        }


        root->children.push_back( _worldbody );
    }

    void TMjcTerrainGenWrapper::preStep()
    {
        _collectFromGenerator();

        // update the properties of all objects (if they have a primitive as reference)
        for ( size_t i = 0; i < m_mjcTerrainPrimitives.size(); i++ )
        {
            if ( m_mjcTerrainPrimitives[i]->tysocPrimitiveObj )
            {
                _updateProperties( m_mjcTerrainPrimitives[i] );
            }
        }
    }

    void TMjcTerrainGenWrapper::_collectFromGenerator()
    {
        auto _newPrimitivesQueue = m_terrainGenPtr->getJustCreated();

        while ( !_newPrimitivesQueue.empty() )
        {
            auto _newPrimitive = _newPrimitivesQueue.front();
            _newPrimitivesQueue.pop();

            _wrapNewPrimitive( _newPrimitive, true );
        }

        // flush the creation queue to avoid double references everywhere
        m_terrainGenPtr->flushJustCreatedQueue();
    }

    void TMjcTerrainGenWrapper::_collectFixedFromGenerator()
    {
        auto _fixedPrimitivesQueue = m_terrainGenPtr->getFixed();

        while ( !_fixedPrimitivesQueue.empty() )
        {
            auto _fixedPrimitive = _fixedPrimitivesQueue.front();
            _fixedPrimitivesQueue.pop();

            _wrapNewPrimitive( _fixedPrimitive, false );
        }

        // flush the fixed queue to avoid double references (the wrapper now holds the ref.)
        m_terrainGenPtr->flushFixedQueue();
    }

    void TMjcTerrainGenWrapper::_updateProperties( TMjcTerrainPrimitive* mjcTerrainPritimivePtr )
    {
        auto _primitiveObj = mjcTerrainPritimivePtr->tysocPrimitiveObj;


        mjcint::setTerrainBodyPosition( m_mjcModelPtr, 
                                        m_mjcDataPtr,
                                        mjcTerrainPritimivePtr->mjcBodyName,
                                        { _primitiveObj->pos.x, _primitiveObj->pos.y, _primitiveObj->pos.z } );

        mjcint::setTerrainBodyOrientation( m_mjcModelPtr,
                                           m_mjcDataPtr,
                                           mjcTerrainPritimivePtr->mjcBodyName,
                                           _primitiveObj->rotmat );

        mjcint::changeSize( m_mjcModelPtr,
                            mjcTerrainPritimivePtr->mjcBodyName,
                            { 0.5f * _primitiveObj->size.x, 0.5f * _primitiveObj->size.y, 0.5f * _primitiveObj->size.z } );

        mjcint::setRbound( m_mjcModelPtr,
                           mjcTerrainPritimivePtr->mjcBodyName,
                           _primitiveObj->rbound );

        if ( !_primitiveObj->useCustomColor )
        {
            float _color[3];
            mjcint::getGeometryColor( m_mjcModelPtr,
                                      m_mjcScenePtr,
                                      mjcTerrainPritimivePtr->mjcBodyName,
                                      _color );

            _primitiveObj->color.r = _color[0];
            _primitiveObj->color.g = _color[1];
            _primitiveObj->color.b = _color[2];
        }
    }

    void TMjcTerrainGenWrapper::_wrapNewPrimitive( tysoc::terrain::TTerrainPrimitive* primitivePtr, bool isReusable )
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

        if ( isReusable )
        {
            // put it into the working queue
            m_mjcWorkingPrimitives.push( _mjcPrimitive );
        }
        else
        {
            // put it into the fixed queue
            m_mjcFixedPrimitives.push( _mjcPrimitive );
        }
    }


}}