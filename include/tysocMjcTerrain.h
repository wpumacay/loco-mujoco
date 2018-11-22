
#pragma once

#include <tysocMjcCommon.h>

// Pool size for the number of mjc bodies to use
#define MJC_TERRAIN_POOL_SIZE 30
// Default parameters for the connected-paths section
#define MJC_TERRAIN_PATH_DEFAULT_WIDTH 0.5f
#define MJC_TERRAIN_PATH_DEFAULT_DEPTH 2.0f
#define MJC_TERRAIN_PATH_DEFAULT_TICKNESS 0.025f

#include <terrain/terrain.h>

namespace tysocMjc
{

    /**
    * This is a wrapper on top of the primitives...
    * used by the terrain generators
    */
    struct TMjcTerrainPrimitive
    {
        int                         mjcBodyId;
        std::string                 mjcBodyName;
        std::string                 mjcGeomType;
        mjcf::Sizef                 mjcGeomSize;
        bool                        isAvailable;
        tysocterrain::TTerrainPrimitive*   tysocPrimitiveObj;
    };


    class TMjcTerrainGenWrapper
    {
        private :

        // Main storage for the primitives
        std::vector< TMjcTerrainPrimitive* > m_mjcTerrainPrimitives;
        // working queues for the logic
        std::queue< TMjcTerrainPrimitive* > m_mjcAvailablePrimitives;
        std::queue< TMjcTerrainPrimitive* > m_mjcWorkingPrimitives;

        // terrain generator to wrap
        tysocterrain::TTerrainGenerator* m_terrainGenPtr;

        // mujoco resources to inject into workspace
        mjcf::GenericElement* m_modelElmPtr;

        // a reference to the mujoco model
        mjModel* m_mjcModelPtr;

        // name for this agentwrapper (and underlying agent as well)
        std::string m_name;

        void _collectFromGenerator();
        void _wrapNewPrimitive( tysocterrain::TTerrainPrimitive* primitivePtr );
        void _updateProperties( TMjcTerrainPrimitive* mjcTerrainPritimivePtr );

        public :

        TMjcTerrainGenWrapper( const std::string& name,
                               tysocterrain::TTerrainGenerator* terrainGenPtr );
        ~TMjcTerrainGenWrapper();

        void initialize();
        void setMjcModel( mjModel* mjcModelPtr );

        std::string name() { return m_name; }
        tysocterrain::TTerrainGenerator* terrainGenerator() { return m_terrainGenPtr; }

        void injectMjcResources( mjcf::GenericElement* root );

        // update the wrapper by collecting all ...
        // information needed for stages on top of ...
        // the wrapper, like its concrete TTysocMjcApi parent
        void preStep();
    };

}