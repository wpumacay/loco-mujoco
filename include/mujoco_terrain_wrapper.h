
#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

// Pool size for the number of mjc bodies to use
#define MJC_TERRAIN_POOL_SIZE PROCEDURAL_TERRAIN_POOL_SIZE - 1
// Default parameters for the connected-paths section
#define MJC_TERRAIN_PATH_DEFAULT_WIDTH 0.5f
#define MJC_TERRAIN_PATH_DEFAULT_DEPTH 2.0f
#define MJC_TERRAIN_PATH_DEFAULT_TICKNESS 0.025f

#include <terrain_wrapper.h>

namespace tysoc {
namespace mujoco {

    /**
    * This is a wrapper on top of the primitives...
    * used by the terrain generators
    */
    struct TMjcTerrainPrimitive
    {
        int                             mjcBodyId;
        std::string                     mjcGeomName;
        std::string                     mjcGeomType;
        TSizef                          mjcGeomSize;
        std::string                     mjcGeomFilename;
        terrain::TTerrainPrimitive*     tysocPrimitiveObj;
    };


    class TMjcTerrainGenWrapper : public TTerrainGenWrapper
    {
        private :

        // Main storage for the primitives
        std::vector< TMjcTerrainPrimitive* > m_mjcTerrainPrimitives;
        // working queues for the logic
        std::queue< TMjcTerrainPrimitive* > m_mjcAvailablePrimitives;
        std::queue< TMjcTerrainPrimitive* > m_mjcWorkingPrimitives;

        // mujoco resources to inject into workspace
        mjcf::GenericElement* m_modelElmPtr;
        // mjcf data where to send the data from above
        mjcf::GenericElement* m_mjcfTargetResourcesPtr;

        // a reference to the mujoco model
        mjModel* m_mjcModelPtr;
        mjData* m_mjcDataPtr;
        mjvScene* m_mjcScenePtr;

        void _collectReusableFromGenerator();// collects primitives that can be reused and rewrapped in the lifetime of the generator
        void _collectStaticFromGenerator();// collects primitives that are single in the lifetime of the generator
        void _wrapReusablePrimitive( terrain::TTerrainPrimitive* primitivePtr );
        void _wrapStaticPrimitive( terrain::TTerrainPrimitive* primitivePtr );
        void _updateProperties( TMjcTerrainPrimitive* mjcTerrainPritimivePtr );

        protected :

        void _initializeInternal() override;
        void _resetInternal() override;
        void _preStepInternal() override;
        void _postStepInternal() override;

        public :

        TMjcTerrainGenWrapper( terrain::TITerrainGenerator* terrainGenPtr,
                               const std::string& workingDir );
        ~TMjcTerrainGenWrapper();

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );
        void setMjcfTargetElm( mjcf::GenericElement* targetResourcesPtr );
    };


    extern "C" TTerrainGenWrapper* terrain_createFromAbstract( terrain::TITerrainGenerator* terrainGenPtr,
                                                               const std::string& workingDir );

    extern "C" TTerrainGenWrapper* terrain_createFromParams( const std::string& name,
                                                             const TGenericParams& params,
                                                             const std::string& workingDir );
}}