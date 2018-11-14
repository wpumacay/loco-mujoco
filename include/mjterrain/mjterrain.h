
#pragma once

#include <mjcf/mjcf_api.h>
#include <mjcint/mjcint_api.h>
#include <terrains/terrain_sections.h>

#include <map>
#include <vector>
#include <queue>
#include <cmath>

#define POOL_SIZE 20
// Default parameters for the connected-paths section
#define SECTION_DEFAULT_WIDTH 0.5f
#define SECTION_DEFAULT_DEPTH 2.0f
#define SECTION_DEFAULT_TICKNESS 0.025f

namespace mjterrain
{

    struct TerrainObj
    {
        int                                     objId;
        std::string                             objName;
        std::string                             objType;
        mjcf::Sizef                             objSize;
        bool                                    available;
        tysocterrain::TConnectedPathSection*    sectionObj;
    };

    // @TODO: Check and refactor if needed. Is it ok bridge pattern here?



    class TerrainHandler
    {
        protected :

        std::vector< TerrainObj* >  m_terrainObjs; // pool of objects to reuse
        std::queue< TerrainObj* >   m_availableObjs;
        std::queue< TerrainObj* >   m_workingObjs;

        tysocterrain::TScenario*    m_scenarioObj;

        mjModel*    m_mjModel;
        mjData*     m_mjData;

        void _resetObj( TerrainObj* pObj );// @TODO WIP
        void _recycleObj( TerrainObj* pObj );// @TODO WIP

        void _createResources();

        public :

        TerrainHandler();
        TerrainHandler( tysocterrain::TScenario* scenarioObj );
        virtual ~TerrainHandler();
        
        void initialize( tysocterrain::TScenarioContext* scenarioContext,
                         mjModel* model, 
                         mjData* data );
        void saveResourcesIntoMjcf( mjcf::GenericElement* root );

        virtual void update( tysocterrain::TScenarioContext* scenarioContext ) = 0;
    };

    class TerrainConnectedPathHandler : public TerrainHandler
    {
        private :
        
        void _updateProperties( TerrainObj* terrainObj );
                                
        void _createSection( tysocterrain::TConnectedPathSection* section );

        public :

        TerrainConnectedPathHandler( tysocterrain::TScenarioConnectedPath* scenarioObj );
        
        void update( tysocterrain::TScenarioContext* scenarioContext ) override;

    }; 

}


