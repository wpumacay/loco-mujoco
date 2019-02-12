
#pragma once

#include <tysocMjcTerrain.h>
#include <tysocMjcKinTreeAgent.h>

#include <dirent.h>

// @TODO: Add comments-docs for doxygen

namespace tysoc {
namespace mujoco {

    class TGenericParams
    {
        private :

        std::map< std::string, int > m_ints;
        std::map< std::string, float > m_floats;
        std::map< std::string, TVec3 > m_vec3s;
        std::map< std::string, TVec4 > m_vec4s;
        std::map< std::string, TSizei > m_sizeis;
        std::map< std::string, TSizef > m_sizefs;
        std::map< std::string, std::string > m_strings;

        public :

        void set( const std::string& name, int val )
        {
            m_ints[ name ] = val;
        }

        void set( const std::string& name, float val )
        {
            m_floats[ name ] = val;
        }

        void set( const std::string& name, const TVec3& vec )
        {
            m_vec3s[ name ] = vec;
        }

        void set( const std::string& name, const TVec4& vec )
        {
            m_vec4s[ name ] = vec;
        }

        void set( const std::string& name, const TSizei& sizei )
        {
            m_sizeis[ name ] = sizei;
        }

        void set( const std::string& name, const TSizef& sizef )
        {
            m_sizefs[ name ] = sizef;
        }

        void set( const std::string& name, const std::string& str )
        {
            m_strings[ name ] = str;
        }

        int getInt( const std::string& name, int def = 0 ) const
        {
            if ( m_ints.find( name ) != m_ints.end() )
            {
                return m_ints.at( name );
            }
            return def;
        }

        float getFloat( const std::string& name, float def = 0.0f ) const
        {
            if ( m_floats.find( name ) != m_floats.end() )
            {
                return m_floats.at( name );
            }
            return def;
        }

        TVec3 getVec3( const std::string& name, const TVec3& def = { 0.0f, 0.0f, 0.0f } ) const
        {
            if ( m_vec3s.find( name ) != m_vec3s.end() )
            {
                return m_vec3s.at( name );
            }
            return def;
        }

        TVec4 getVec4( const std::string& name, const TVec4& def = { 0.0f, 0.0f, 0.0f, 1.0f } ) const
        {
            if ( m_vec4s.find( name ) != m_vec4s.end() )
            {
                return m_vec4s.at( name );
            }
            return def;
        }

        TSizei getSizei( const std::string& name, const TSizei& def = { 0, { 0 } } ) const
        {
            if ( m_sizeis.find( name ) != m_sizeis.end() )
            {
                return m_sizeis.at( name );
            }
            return def;
        }

        TSizef getSizef( const std::string& name, const TSizef& def = { 0, { 0.0f } } ) const
        {
            if ( m_sizefs.find( name ) != m_sizefs.end() )
            {
                return m_sizefs.at( name );
            }
            return def;
        }

        std::string getString( const std::string& name, const std::string& def = "undefined" ) const
        {
            if ( m_strings.find( name ) != m_strings.end() )
            {
                return m_strings.at( name );
            }
            return def;
        }

    };


    class TMjcFactory
    {
        private :

        mjcf::Schema* m_mjcfSchema;

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