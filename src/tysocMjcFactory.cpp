
#include <tysocMjcFactory.h>

namespace tysocMjc
{


    TMjcFactory::TMjcFactory()
    {
        // load the schema to be used with the mjcf helper
        m_mjcfSchema = new mjcf::Schema();
        {
            std::string _schemaPath( TYSOCMJC_RESOURCES_PATH );
            _schemaPath += "xml/schema.xml";

            m_mjcfSchema->load( _schemaPath );
        }

        // precache all resources?
        _precacheModels();
    }

    TMjcFactory::~TMjcFactory()
    {
        if ( m_mjcfSchema )
        {
            delete m_mjcfSchema;
            m_mjcfSchema = NULL;
        }

        m_templateModelFiles.clear();

        for ( auto it = m_cachedModels.begin();
              it != m_cachedModels.end();
              it++ )
        {
            delete it->second;
        }
        m_cachedModels.clear();
    }

    void TMjcFactory::_precacheModels()
    {
        std::vector< std::string > _filesToCache;

        // Get the template files from the templates folder *****************
        DIR* _directoryPtr;
        struct dirent* _direntPtr;

        std::string _templatesFolderPath;
        _templatesFolderPath += TYSOCMJC_RESOURCES_PATH;
        _templatesFolderPath += "templates/";

        _directoryPtr = opendir( _templatesFolderPath.c_str() );
        if ( _directoryPtr )
        {
            while ( _direntPtr = readdir( _directoryPtr ) )
            {
                std::string _fname = _direntPtr->d_name;
                if ( _fname.find( ".xml" ) != std::string::npos )
                {
                    m_templateModelFiles.push_back( _fname );
                }
            }
        }
        closedir( _directoryPtr );

        // ******************************************************************

        for ( size_t i = 0; i < m_templateModelFiles.size(); i++ )
        {
            _precacheSingleModel( m_templateModelFiles[ i ] );
        }
    }

    void TMjcFactory::_precacheSingleModel( const std::string& templateFile )
    {
        std::cout << "INFO> trying to load template: " << templateFile << std::endl;
        // Gran the model into a mjcf::GenericElement node
        auto _root = mjcf::loadGenericModel( m_mjcfSchema, std::string( TYSOCMJC_RESOURCES_PATH ) + "templates/" + templateFile );

        // Extract the name to use as key in the cache dictionary
        size_t _xmlTagPos = templateFile.find( ".xml" );
        std::string _modelName = templateFile.substr( 0, _xmlTagPos );

        // cache the model
        m_cachedModels[ _modelName ] = _root;

        std::cout << "INFO> Precached template model: " << _modelName << std::endl;
    }

    TMjcAgentWrapper* TMjcFactory::createAgent( const std::string& name,
                                                const std::string& modelname,
                                                float startX, float startY, float startZ )
    {
        if ( m_cachedModels.find( modelname ) == m_cachedModels.end() )
        {
            std::cout << "WARNING> agent " << name << " "
                      << "cannot be created because modelname " << modelname << " "
                      << "is not a templated model or wasn't cached" << std::endl;

            return NULL;
        }

        // grab the template model and its components
        auto _modelTemplateElm = m_cachedModels[ modelname ];
        // create a new genericelement with the names changed
        auto _modelElm = new mjcf::GenericElement();
        mjcf::deepCopy( _modelElm, _modelTemplateElm );
        mjcf::replaceNameRecursive( _modelElm, name );

        auto _agentWrapper = new TMjcAgentWrapper( name, _modelElm,
                                                   startX, startY, startZ );

        return _agentWrapper;
    }

    TMjcTerrainGenWrapper* TMjcFactory::createTerrainGen( const std::string& name,
                                                          const std::string& type,
                                                          const TGenericParams& params )
    {
        tysocterrain::TTerrainGenerator* _terrainGenerator = NULL;

        if ( type == "procedural" )
        {
            tysocterrain::TProfileGenerator* _profileGenerator;
            if ( params.getString( "profiler" ) == "sine" )
            {
                float _ampl     = params.getFloat( "sineProfileAmplitude", 2.0f );
                float _period   = params.getFloat( "sineProfilePeriod", 10.0f );
                float _phase    = params.getFloat( "sineProfilePhase", 1.57f );
                _profileGenerator = new tysocterrain::TSineProfileGenerator( _ampl, 
                                                                             _period, 
                                                                             _phase );
            }
            else if ( params.getString( "profiler" ) == "perlin" )
            {
                int _octaves        = params.getInt( "perlinProfileOctaves", 4 );
                float _persistance  = params.getFloat( "perlinProfilePersistance", 0.5f );
                float _lacunarity   = params.getFloat( "perlinProfileLacunarity", 2.0f );
                float _noiseScale   = params.getFloat( "perlinProfileNoiseScale", 10.0f );
                _profileGenerator = new tysocterrain::TPerlin1DProfileGenerator( _octaves,
                                                                                 _persistance,
                                                                                 _lacunarity,
                                                                                 _noiseScale );
            }
            else
            {
                _profileGenerator = new tysocterrain::TSineProfileGenerator( 2.0f, 10.0f, 1.57f );
            }

            float _sectionDepth         = params.getFloat( "sectionDepth", 1.0f );
            float _componentsSpacingX   = params.getFloat( "componentsSpacingX", 0.5f );
            float _componentsThickness  = params.getFloat( "componentsThickness", 0.01f );

            mjcf::Vec3 _startPosition = params.getVec3( "startPosition" );

            _terrainGenerator = new tysocterrain::TPathTerrainGenerator( name, 
                                                                         _startPosition.x,
                                                                         _startPosition.y,
                                                                         _startPosition.z,
                                                                         _sectionDepth,
                                                                         _componentsSpacingX,
                                                                         _componentsThickness,
                                                                         _profileGenerator );
        }

        auto _terrainGeneratorWrapper = new TMjcTerrainGenWrapper( name, _terrainGenerator );

        return _terrainGeneratorWrapper;
    }
}