
#pragma once

#include <tysocMjcTerrain.h>
#include <tysocMjcAgent.h>

#include <dirent.h>

// @TODO: Add comments-docs for doxygen

namespace tysocMjc
{




    class TMjcFactory
    {
        private :

        mjcf::Schema* m_mjcfSchema;
        std::vector< std::string > m_templateModelFiles;
        std::map< std::string, mjcf::GenericElement* > m_cachedModels;

        void _precacheSingleModel( const std::string& templateFile );
        void _precacheModels();

        public :

        TMjcFactory();
        ~TMjcFactory();

        // @TODO: Should add support for loading from non-cached file?
        TMjcAgentWrapper* createAgent( const std::string& name,
                                       const std::string& modelname );

        // @TODO: Make a generic build object which has all params?
        TMjcTerrainGenWrapper* createTerrainGen( const std::string& name,
                                                 const std::string& type );

    };



}