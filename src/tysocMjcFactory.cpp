
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
                                                const std::string& modelname )
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

        auto _agentWrapper = new TMjcAgentWrapper( name, _modelElm );

        return _agentWrapper;
    }

    TMjcTerrainGenWrapper* TMjcFactory::createTerrainGen( const std::string& name,
                                                          const std::string& type )
    {
        tysocterrain::TTerrainGenerator* _terrainGenerator = NULL;

        if ( type == "procedural" )
        {
            auto _sineProfileGenerator = new tysocterrain::TSineProfileGenerator( 2.0f, 10.0f, 1.57f );
            _terrainGenerator = new tysocterrain::TPathTerrainGenerator( name, 
                                                                         _sineProfileGenerator,
                                                                         0.5f, 3.0f, 0.01f );
        }

        auto _terrainGeneratorWrapper = new TMjcTerrainGenWrapper( name, _terrainGenerator );

        return _terrainGeneratorWrapper;
    }
}