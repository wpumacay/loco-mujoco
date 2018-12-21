
#include <tysocCustomVizTerrainGenerator.h>

#ifndef TYSOCMJC_RESOURCES_PATH
    #define TYSOCMJC_RESOURCES_PATH "../../res/xml"
#endif

namespace tysoc {
namespace viz {

    TCustomVizTerrainGenerator::TCustomVizTerrainGenerator( terrain::TITerrainGenerator* terrainGeneratorPtr,
                                                engine::LScene* scenePtr )
    {
        m_scenePtr              = scenePtr;
        m_terrainGeneratorPtr   = terrainGeneratorPtr;

        _collectTerrainPrimitives();
    }

    TCustomVizTerrainGenerator::~TCustomVizTerrainGenerator()
    {
        m_scenePtr              = NULL;
        m_terrainGeneratorPtr   = NULL;

        for ( size_t i = 0; i < m_vizTerrainPrimitives.size(); i++ )
        {
            m_vizTerrainPrimitives[i].axesPtr               = NULL;
            m_vizTerrainPrimitives[i].meshPtr               = NULL;
            m_vizTerrainPrimitives[i].terrainPrimitivePtr   = NULL;
        }
        m_vizTerrainPrimitives.clear();
    }

    void TCustomVizTerrainGenerator::update()
    {
        for ( size_t i = 0; i < m_vizTerrainPrimitives.size(); i++ )
        {
            _updateVizTerrainPrimitive( m_vizTerrainPrimitives[i] );
        }
    }

    terrain::TITerrainGenerator* TCustomVizTerrainGenerator::getTerrainGeneratorPtr()
    {
        return m_terrainGeneratorPtr;
    }

    void TCustomVizTerrainGenerator::_collectTerrainPrimitives()
    {
        auto _terrainPrimitives = m_terrainGeneratorPtr->getPrimitives();
        for ( size_t i = 0; i < _terrainPrimitives.size(); i++ )
        {
            TCustomVizTerrainPrimitive _vizTerrainPrimitive;

            // get the underlying primitive
            terrain::TTerrainPrimitive* _terrainPrimitivePtr = _terrainPrimitives[i];
            // and assign it to our wrapper
            _vizTerrainPrimitive.terrainPrimitivePtr = _terrainPrimitivePtr;
            // add the axes as well
            _vizTerrainPrimitive.axesPtr = engine::LMeshBuilder::createAxes( VIZTERRAIN_AXES_DEFAULT_SIZE );
            // and add it to the scene
            m_scenePtr->addRenderable( _vizTerrainPrimitive.axesPtr );

            // and create the appropiate mesh
            if ( _terrainPrimitivePtr->geomType != "data" )
            {
                _vizTerrainPrimitive.meshPtr = _createMeshPrimitive( _terrainPrimitivePtr );
            }
            else
            {
                _vizTerrainPrimitive.meshPtr = _createMeshFromData( _terrainPrimitivePtr );
            }

            m_vizTerrainPrimitives.push_back( _vizTerrainPrimitive );
        }
    }

    engine::LIRenderable* TCustomVizTerrainGenerator::_createMeshPrimitive( terrain::TTerrainPrimitive* terrainPrimitivePtr )
    {
        engine::LIRenderable* _renderablePtr = NULL;

        // create the mesh with some default sizes (for rescaling later)
        if ( terrainPrimitivePtr->geomType == "plane" )
        {
            _renderablePtr = engine::LMeshBuilder::createPlane( 2.0f, 2.0f );
        }
        else if ( terrainPrimitivePtr->geomType == "box" )
        {
            _renderablePtr = engine::LMeshBuilder::createBox( 2.0f, 2.0f, 2.0f );
        }
        else if ( terrainPrimitivePtr->geomType == "sphere" )
        {
            _renderablePtr = engine::LMeshBuilder::createSphere( 1.0f );
        }
        else if ( terrainPrimitivePtr->geomType == "capsule" )
        {
            _renderablePtr = engine::LMeshBuilder::createCapsule( 1.0f, 2.0f );
        }
        else if ( terrainPrimitivePtr->geomType == "cylinder" )
        {
            _renderablePtr = engine::LMeshBuilder::createCylinder( 1.0f, 2.0f );
        }
        else if ( terrainPrimitivePtr->geomType == "mesh" )
        {
            auto _meshFilePath = std::string( TYSOCMJC_RESOURCES_PATH ) + 
                                 std::string( "xml/" ) + 
                                 terrainPrimitivePtr->filename;
            _renderablePtr = engine::LMeshBuilder::createModelFromFile( _meshFilePath, "" );
        }

        if ( _renderablePtr )
        {
            // resize the mesh to its actual size
            _resizeMesh( _renderablePtr, terrainPrimitivePtr );

            // apply material settings
            TVec3 _color = { terrainPrimitivePtr->color.r,
                             terrainPrimitivePtr->color.g,
                             terrainPrimitivePtr->color.b };

            _renderablePtr->getMaterial()->ambient     = { _color.x, _color.y, _color.z };
            _renderablePtr->getMaterial()->diffuse     = { _color.x, _color.y, _color.z };
            _renderablePtr->getMaterial()->specular    = { _color.x, _color.y, _color.z };

            // and add it to the renderer scene
            m_scenePtr->addRenderable( _renderablePtr );
        }
        else
        {
            std::cout << "WARNING> could not create mesh of type: " << terrainPrimitivePtr->geomType << std::endl;
            if ( terrainPrimitivePtr->geomType == "mesh" )
            {
                std::cout << "WARNING> filename of mesh: " << terrainPrimitivePtr->filename << std::endl;
            }
        }

        return _renderablePtr;
    }

    engine::LIRenderable* TCustomVizTerrainGenerator::_createMeshFromData( terrain::TTerrainPrimitive* terrainPrimitivePtr )
    {
        // @WIP
        std::cout << "WARNING> WIP-terrainFromData. Working to add support for this feature" << std::endl;
        return NULL;
    }

    void TCustomVizTerrainGenerator::_resizeMesh( engine::LIRenderable* renderablePtr, terrain::TTerrainPrimitive* terrainPrimitivePtr )
    {
        if ( terrainPrimitivePtr->geomType == "plane" )
        {
            renderablePtr->scale.x = 0.5f * terrainPrimitivePtr->size.x;
            renderablePtr->scale.y = 0.5f * terrainPrimitivePtr->size.y;
        }
        else if ( terrainPrimitivePtr->geomType == "sphere" )
        {
            renderablePtr->scale.x = terrainPrimitivePtr->size.x;
        }
        else if ( terrainPrimitivePtr->geomType == "capsule" )
        {
            renderablePtr->scale.x = terrainPrimitivePtr->size.x;
            renderablePtr->scale.y = terrainPrimitivePtr->size.x;
            renderablePtr->scale.z = 0.5f * terrainPrimitivePtr->size.y;
        }
        else if ( terrainPrimitivePtr->geomType == "cylinder" )
        {
            renderablePtr->scale.x = terrainPrimitivePtr->size.x;
            renderablePtr->scale.y = terrainPrimitivePtr->size.x;
            renderablePtr->scale.z = 0.5f * terrainPrimitivePtr->size.y;
        }
        else if ( terrainPrimitivePtr->geomType == "box" )
        {
            renderablePtr->scale.x = 0.5f * terrainPrimitivePtr->size.x;
            renderablePtr->scale.y = 0.5f * terrainPrimitivePtr->size.y;
            renderablePtr->scale.z = 0.5f * terrainPrimitivePtr->size.z;
        }
    }

    void TCustomVizTerrainGenerator::_updateVizTerrainPrimitive( TCustomVizTerrainPrimitive& vizTerrainPrimitive )
    {
        auto _axesPtr               = vizTerrainPrimitive.axesPtr;
        auto _renderablePtr         = vizTerrainPrimitive.meshPtr;
        auto _terrainPrimitivePtr   = vizTerrainPrimitive.terrainPrimitivePtr;

        // Set some rendering options
        _renderablePtr->setVisibility( drawState.showPrimitives ||
                                       _terrainPrimitivePtr->inUse );
        _renderablePtr->setWireframeMode( drawState.drawAsWireframe );
        _axesPtr->setVisibility( drawState.showFrameAxes );

        // also, resize the mesh appropiately (@CHECK: this could be done once, but will check later)
        _resizeMesh( _renderablePtr, _terrainPrimitivePtr );

        // @TODO: Check the case where there are models as well as meshes ...
        // as there could be some issues with their children ahving different colors

        // and update the colors (@CHECK: again, this could be done once, but will check later if breaks something)
        _renderablePtr->getMaterial()->ambient.x = _terrainPrimitivePtr->color.r;
        _renderablePtr->getMaterial()->ambient.y = _terrainPrimitivePtr->color.g;
        _renderablePtr->getMaterial()->ambient.z = _terrainPrimitivePtr->color.b;

        _renderablePtr->getMaterial()->diffuse.x = _terrainPrimitivePtr->color.r;
        _renderablePtr->getMaterial()->diffuse.y = _terrainPrimitivePtr->color.g;
        _renderablePtr->getMaterial()->diffuse.z = _terrainPrimitivePtr->color.b;

        _renderablePtr->getMaterial()->specular.x = _terrainPrimitivePtr->color.r;
        _renderablePtr->getMaterial()->specular.y = _terrainPrimitivePtr->color.g;
        _renderablePtr->getMaterial()->specular.z = _terrainPrimitivePtr->color.b;

        // finally update the position
        _renderablePtr->pos.x = _terrainPrimitivePtr->pos.x;
        _renderablePtr->pos.y = _terrainPrimitivePtr->pos.y;
        _renderablePtr->pos.z = _terrainPrimitivePtr->pos.z;

        // and the orientation
        _renderablePtr->rotation.set( 0, 0, _terrainPrimitivePtr->rotmat[0] );
        _renderablePtr->rotation.set( 0, 1, _terrainPrimitivePtr->rotmat[3] );
        _renderablePtr->rotation.set( 0, 2, _terrainPrimitivePtr->rotmat[6] );
        _renderablePtr->rotation.set( 1, 0, _terrainPrimitivePtr->rotmat[1] );
        _renderablePtr->rotation.set( 1, 1, _terrainPrimitivePtr->rotmat[4] );
        _renderablePtr->rotation.set( 1, 2, _terrainPrimitivePtr->rotmat[7] );
        _renderablePtr->rotation.set( 2, 0, _terrainPrimitivePtr->rotmat[2] );
        _renderablePtr->rotation.set( 2, 1, _terrainPrimitivePtr->rotmat[5] );
        _renderablePtr->rotation.set( 2, 2, _terrainPrimitivePtr->rotmat[8] );
    }

}}