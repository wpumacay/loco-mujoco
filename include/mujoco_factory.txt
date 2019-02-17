
#pragma once

#include <tysocMjcTerrain.h>
#include <tysocMjcKinTreeAgent.h>

#include <dirent.h>

// @TODO: Add comments-docs for doxygen

namespace tysoc {
namespace mujoco {


    class TMjcFactory
    {
        private :

        std::vector< std::string > m_templateMjcfModelFiles;
        std::vector< std::string > m_templateUrdfModelFiles;
        std::vector< std::string > m_templateRlsimModelFiles;

        std::map< std::string, mjcf::GenericElement* >  m_cachedMjcfModels;
        std::map< std::string, urdf::UrdfModel* >       m_cachedUrdfModels;
        std::map< std::string, rlsim::RlsimModel* >     m_cachedRlsimModels;

        void _precacheModels();
        void _precacheMjcfModels();
        void _precacheSingleMjcfModel( const std::string& templateFile );
        void _precacheUrdfModels();
        void _precacheSingleUrdfModel( const std::string& templateFile );
        void _precacheRlsimModels();
        void _precacheSingleRlsimModel( const std::string& templateFile );

        public :

        TMjcFactory();
        ~TMjcFactory();

        // @TODO|@CHECK: should add functionality for general ...
        // agent creation not just kintree based agents. 

        TMjcKinTreeAgentWrapper* createKinTreeAgentFromMjcf( const std::string& name,
                                                             const std::string& modelname,
                                                             float startX, float startY, float startZ );

        TMjcKinTreeAgentWrapper* createKinTreeAgentFromUrdf( const std::string& name,
                                                             const std::string& modelname,
                                                             float startX, float startY, float startZ );

        TMjcKinTreeAgentWrapper* createKinTreeAgentFromRlsim( const std::string& name,
                                                              const std::string& modelname,
                                                              float startX, float startY, float startZ );

        TMjcTerrainGenWrapper* createTerrainGen( const std::string& name,
                                                 const std::string& type,
                                                 const TGenericParams& params );

    };


}}