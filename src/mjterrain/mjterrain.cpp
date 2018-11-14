
#include <mjterrain/mjterrain.h>

namespace mjterrain
{

    TerrainHandler::TerrainHandler()
    {
        m_scenarioObj   = NULL;
        m_mjModel       = NULL;
        m_mjData        = NULL;

        _createResources();
    }

    TerrainHandler::TerrainHandler( tysocterrain::TScenario* scenarioObj )
    {
        m_scenarioObj   = scenarioObj;
        m_mjModel       = NULL;
        m_mjData        = NULL;

        _createResources();
    }

    void TerrainHandler::_createResources()
    {
        // create some resources
        for ( size_t i = 0; i < POOL_SIZE; i++ )
        {
            auto _obj = new TerrainObj();
            _obj->objName   = std::string( "tobj_" ) + std::to_string( i );
            _obj->objId     = -1;
            _obj->objType   = "box";
            _obj->objSize   = { 3, { 0.5f * SECTION_DEFAULT_WIDTH, 
                                     0.5f * SECTION_DEFAULT_DEPTH, 
                                     0.5f * SECTION_DEFAULT_TICKNESS } };

            _obj->sectionObj = NULL;

            m_terrainObjs.push_back( _obj );
            m_availableObjs.push( _obj );
        }
    }

    TerrainHandler::~TerrainHandler()
    {
        if ( m_scenarioObj )
        {
            delete m_scenarioObj;
            m_scenarioObj = NULL;
        }

        m_mjData = NULL;
        m_mjModel = NULL;
        
        while ( !m_availableObjs.empty() )
        {
            m_availableObjs.pop();
        }

        while ( !m_workingObjs.empty() )
        {
            m_workingObjs.pop();
        }

        for ( size_t i = 0; i < m_terrainObjs.size(); i++ )
        {
            delete m_terrainObjs[i];
            m_terrainObjs[i] = NULL;
        }
        m_terrainObjs.clear();
    }

    void TerrainHandler::initialize( tysocterrain::TScenarioContext* scenarioContext,
                                     mjModel* model, 
                                     mjData* data )
    {
        m_mjModel   = model;
        m_mjData    = data;
        
        if ( m_scenarioObj )
        {
            m_scenarioObj->initialize( scenarioContext );
            update( scenarioContext );
        }
    }

    void TerrainHandler::saveResourcesIntoMjcf( mjcf::GenericElement* root )
    {
        // create a root body for the elements in the pool
        auto _worldbody = mjcf::_createWorldBody();
        root->children.push_back( _worldbody );

        // add the bodies to the root element
        for ( size_t i = 0; i < m_terrainObjs.size(); i++ )
        {
            auto _obj = m_terrainObjs[i];
            // create the elements to write to the mjcf
            auto _bodyObj = mjcf::_createBody( _obj->objName, 
                                               { 0.0f,
                                                 0.0f,
                                                 100.0f + i * ( SECTION_DEFAULT_TICKNESS + 1.0f ) } );
            auto _geomObj = mjcf::_createGeometry( _obj->objName,
                                                   _obj->objType,
                                                   _obj->objSize, 0.1f );
            _geomObj->setAttributeInt( "contype", 0 );
            _geomObj->setAttributeInt( "conaffinity", 1 );
            
            _bodyObj->children.push_back( _geomObj );
            _worldbody->children.push_back( _bodyObj );
        }
    }



    TerrainConnectedPathHandler::TerrainConnectedPathHandler( tysocterrain::TScenarioConnectedPath* scenarioObj )
        : TerrainHandler( scenarioObj )
    {
        // Make some extra stuff if needed
    }

    void TerrainConnectedPathHandler::update( tysocterrain::TScenarioContext* scenarioContext )
    {
        m_scenarioObj->update( scenarioContext );

        // Check for new created sections
        auto _sections = reinterpret_cast< tysocterrain::TScenarioConnectedPath* >( m_scenarioObj )->getSections();
        
        for ( size_t i = 0; i < _sections.size(); i++ )
        {
            if ( _sections[i]->awaitingInitialization )
            {
                _createSection( _sections[i] );
                _sections[i]->awaitingInitialization = false;
            }
        }

        // Update the objects according to the linked section
        for ( size_t i = 0; i < m_terrainObjs.size(); i++ )
        {
            auto _terrainObj = m_terrainObjs[i];
            if ( _terrainObj->sectionObj )
            {
                _updateProperties( _terrainObj );
            }
        }
    }

    void TerrainConnectedPathHandler::_updateProperties( TerrainObj* terrainObj )
    {
        auto _sectionObj = terrainObj->sectionObj;

        mjcint::setBodyPosition( m_mjModel, 
                                 terrainObj->objName,
                                 { _sectionObj->pos.x, _sectionObj->pos.y, _sectionObj->pos.z } );

        mjcint::setBodyOrientation( m_mjModel,
                                    terrainObj->objName,
                                    _sectionObj->rotmat );

        mjcint::changeSize( m_mjModel,
                            terrainObj->objName,
                            { 0.5f * _sectionObj->size.x, 0.5f * _sectionObj->size.y, 0.5f * _sectionObj->size.z } );

        mjcint::setRbound( m_mjModel,
                           terrainObj->objName,
                           _sectionObj->rbound );
    }

    void TerrainConnectedPathHandler::_createSection( tysocterrain::TConnectedPathSection* section )
    {
        // if the pool is empty, recycle the last object
        if ( m_availableObjs.empty() )
        {
            auto _oldest = m_workingObjs.front();
            if ( _oldest->sectionObj )
            {
                _oldest->sectionObj->awaitingDestruction = true;
                _oldest->sectionObj = NULL;
                m_workingObjs.pop();
            }
            m_availableObjs.push( _oldest );
        }

        // link an object from the working queue with the requested new section
        auto _terrainObj = m_availableObjs.front();
        _terrainObj->sectionObj = section;

        // put it into the working queue
        m_availableObjs.pop();
        m_workingObjs.push( _terrainObj );
    }

}