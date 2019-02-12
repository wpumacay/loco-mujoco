
#include <tysocMjcFactory.h>

namespace tysoc {
namespace mujoco {


    TMjcFactory::TMjcFactory()
    {
        // precache all resources?
        _precacheModels();
    }

    TMjcFactory::~TMjcFactory()
    {
        m_templateMjcfModelFiles.clear();
        m_templateUrdfModelFiles.clear();
        m_templateRlsimModelFiles.clear();

        for ( auto it = m_cachedMjcfModels.begin();
              it != m_cachedMjcfModels.end();
              it++ )
        {
            delete it->second;
        }
        m_cachedMjcfModels.clear();

        for ( auto it = m_cachedUrdfModels.begin();
              it != m_cachedUrdfModels.end();
              it++ )
        {
            delete it->second;
        }
        m_cachedUrdfModels.clear();

        for ( auto it = m_cachedRlsimModels.begin();
              it != m_cachedRlsimModels.end();
              it++ )
        {
            delete it->second;
        }
        m_cachedRlsimModels.clear();
    }

    void TMjcFactory::_precacheModels()
    {
        _precacheMjcfModels();
        _precacheUrdfModels();
        _precacheRlsimModels();
    }

    void TMjcFactory::_precacheMjcfModels()
    {
        std::vector< std::string > _filesToCache;

        // Get the template files from the templates folder *****************
        DIR* _directoryPtr;
        struct dirent* _direntPtr;

        std::string _templatesFolderPath;
        _templatesFolderPath += TYSOCMJC_RESOURCES_PATH;
        _templatesFolderPath += "templates/mjcf/";

        _directoryPtr = opendir( _templatesFolderPath.c_str() );
        if ( _directoryPtr )
        {
            while ( _direntPtr = readdir( _directoryPtr ) )
            {
                std::string _fname = _direntPtr->d_name;
                if ( _fname.find( ".xml" ) != std::string::npos )
                {
                    m_templateMjcfModelFiles.push_back( _fname );
                }
            }
        }
        closedir( _directoryPtr );

        // ******************************************************************

        for ( size_t i = 0; i < m_templateMjcfModelFiles.size(); i++ )
        {
            _precacheSingleMjcfModel( m_templateMjcfModelFiles[ i ] );
        }
    }

    void TMjcFactory::_precacheUrdfModels()
    {
        std::vector< std::string > _filesToCache;

        // Get the template files from the templates folder *****************
        DIR* _directoryPtr;
        struct dirent* _direntPtr;

        std::string _templatesFolderPath;
        _templatesFolderPath += TYSOCMJC_RESOURCES_PATH;
        _templatesFolderPath += "templates/urdf/";

        _directoryPtr = opendir( _templatesFolderPath.c_str() );
        if ( _directoryPtr )
        {
            while ( _direntPtr = readdir( _directoryPtr ) )
            {
                std::string _fname = _direntPtr->d_name;
                if ( _fname.find( ".urdf" ) != std::string::npos )
                {
                    m_templateUrdfModelFiles.push_back( _fname );
                }
            }
        }
        closedir( _directoryPtr );

        // ******************************************************************

        for ( size_t i = 0; i < m_templateUrdfModelFiles.size(); i++ )
        {
            _precacheSingleUrdfModel( m_templateUrdfModelFiles[ i ] );
        }
    }

    // @TODO|@CHECK: refactor this part in order to have a single precache starting point
    void TMjcFactory::_precacheRlsimModels()
    {
        std::vector< std::string > _filesToCache;

        // Get the template files from the templates folder *****************
        DIR* _directoryPtr;
        struct dirent* _direntPtr;

        std::string _templatesFolderPath;
        _templatesFolderPath += TYSOCMJC_RESOURCES_PATH;
        _templatesFolderPath += "templates/rlsim/";

        _directoryPtr = opendir( _templatesFolderPath.c_str() );
        if ( _directoryPtr )
        {
            while ( _direntPtr = readdir( _directoryPtr ) )
            {
                std::string _fname = _direntPtr->d_name;
                if ( _fname.find( ".json" ) != std::string::npos )
                {
                    m_templateRlsimModelFiles.push_back( _fname );
                }
            }
        }
        closedir( _directoryPtr );

        // ******************************************************************

        for ( size_t i = 0; i < m_templateRlsimModelFiles.size(); i++ )
        {
            _precacheSingleRlsimModel( m_templateRlsimModelFiles[ i ] );
        }
    }

    void TMjcFactory::_precacheSingleMjcfModel( const std::string& templateFile )
    {
        std::cout << "INFO> trying to load template: " << templateFile << std::endl;
        // Gran the model into a mjcf::GenericElement node
        auto _root = mjcf::loadGenericModel( std::string( TYSOCMJC_RESOURCES_PATH ) + "templates/mjcf/" + templateFile );

        // Extract the name to use as key in the cache dictionary
        size_t _xmlTagPos = templateFile.find( ".xml" );
        std::string _modelName = templateFile.substr( 0, _xmlTagPos );

        // cache the model
        m_cachedMjcfModels[ _modelName ] = _root;

        std::cout << "INFO> Precached mjcf template model: " << _modelName << std::endl;
    }

    void TMjcFactory::_precacheSingleUrdfModel( const std::string& templateFile )
    {
        std::cout << "INFO> trying to load template: " << templateFile << std::endl;
        // Gran the model into a mjcf::GenericElement node
        auto _urdfModel = urdf::loadGenericModel( std::string( TYSOCMJC_RESOURCES_PATH ) + "templates/urdf/" + templateFile );

        // Extract the name to use as key in the cache dictionary
        size_t _xmlTagPos = templateFile.find( ".urdf" );
        std::string _modelName = templateFile.substr( 0, _xmlTagPos );

        // cache the model
        m_cachedUrdfModels[ _modelName ] = _urdfModel;

        std::cout << "INFO> Precached urdf template model: " << _modelName << std::endl;
    }

    // @TODO|@CHECK: refactor this part in order to have a single precachesingle function
    void TMjcFactory::_precacheSingleRlsimModel( const std::string& templateFile )
    {
        std::cout << "INFO> trying to load template: " << templateFile << std::endl;
        // Gran the model into a mjcf::GenericElement node
        auto _rlsimModel = rlsim::loadGenericModel( std::string( TYSOCMJC_RESOURCES_PATH ) + "templates/rlsim/" + templateFile );

        // Extract the name to use as key in the cache dictionary
        size_t _jsonTagPos = templateFile.find( ".json" );
        std::string _modelName = templateFile.substr( 0, _jsonTagPos );

        // cache the model
        m_cachedRlsimModels[ _modelName ] = _rlsimModel;

        std::cout << "INFO> Precached rlsim template model: " << _modelName << std::endl;
    }

    TMjcKinTreeAgentWrapper* TMjcFactory::createKinTreeAgentFromMjcf( const std::string& name,
                                                                      const std::string& modelname,
                                                                      float startX, float startY, float startZ )
    {
        if ( m_cachedMjcfModels.find( modelname ) == m_cachedMjcfModels.end() )
        {
            std::cout << "WARNING> agent " << name << " "
                      << "cannot be created because modelname " << modelname << " "
                      << "is not a mjcf templated model or wasn't cached" << std::endl;

            return NULL;
        }

        // grab the template model and its components
        auto _modelTemplateElm = m_cachedMjcfModels[ modelname ];
        // and create the wrapper (it handles everything inside)

        auto _kinTreeAgentWrapper = new TMjcKinTreeAgentWrapper( name,
                                                                 _modelTemplateElm,
                                                                 { startX, startY, startZ } );

        return _kinTreeAgentWrapper;
    }

    TMjcKinTreeAgentWrapper* TMjcFactory::createKinTreeAgentFromUrdf( const std::string& name,
                                                                      const std::string& modelname,
                                                                      float startX, float startY, float startZ )
    {
        if ( m_cachedUrdfModels.find( modelname ) == m_cachedUrdfModels.end() )
        {
            std::cout << "WARNING> agent " << name << " "
                      << "cannot be created because modelname " << modelname << " "
                      << "is not a urdf templated model or wasn't cached" << std::endl;

            return NULL;
        }

        // grab the template model and its components
        auto _modelTemplateElm = m_cachedUrdfModels[ modelname ];
        // and create the wrapper (it handles everything inside)

        auto _kinTreeAgentWrapper = new TMjcKinTreeAgentWrapper( name,
                                                                 _modelTemplateElm,
                                                                 { startX, startY, startZ } );

        return _kinTreeAgentWrapper;
    }

    TMjcKinTreeAgentWrapper* TMjcFactory::createKinTreeAgentFromRlsim( const std::string& name,
                                                                       const std::string& modelname,
                                                                       float startX, float startY, float startZ )
    {
        if ( m_cachedRlsimModels.find( modelname ) == m_cachedRlsimModels.end() )
        {
            std::cout << "WARNING> agent " << name << " "
                      << "cannot be created because modelname " << modelname << " "
                      << "is not an rlsim templated model or wasn't cached" << std::endl;

            return NULL;
        }

        // grab the template model and its components
        auto _modelTemplateElm = m_cachedRlsimModels[ modelname ];
        // and create the wrapper (it handles everything inside)

        auto _kinTreeAgentWrapper = new TMjcKinTreeAgentWrapper( name,
                                                                 _modelTemplateElm,
                                                                 { startX, startY, startZ } );

        return _kinTreeAgentWrapper;
    }

    TMjcTerrainGenWrapper* TMjcFactory::createTerrainGen( const std::string& name,
                                                          const std::string& type,
                                                          const TGenericParams& params )
    {
        terrain::TITerrainGenerator* _terrainGenerator = NULL;

        if ( type == "procedural" )
        {
            auto _sectionsType = params.getString( "sectionType", "blocky" );

            if ( _sectionsType == "path" )
            {
                terrain::TProfileGenerator* _profileGenerator;
                if ( params.getString( "pathProfile" ) == "sine" )
                {
                    float _ampl     = params.getFloat( "sineProfileAmplitude", 2.0f );
                    float _period   = params.getFloat( "sineProfilePeriod", 10.0f );
                    float _phase    = params.getFloat( "sineProfilePhase", 1.57f );
                    _profileGenerator = new terrain::TSineProfileGenerator( _ampl, 
                                                                            _period, 
                                                                            _phase );
                }
                else if ( params.getString( "pathProfile" ) == "perlin" )
                {
                    int _octaves        = params.getInt( "perlinProfileOctaves", 4 );
                    float _persistance  = params.getFloat( "perlinProfilePersistance", 0.5f );
                    float _lacunarity   = params.getFloat( "perlinProfileLacunarity", 2.0f );
                    float _noiseScale   = params.getFloat( "perlinProfileNoiseScale", 10.0f );
                    _profileGenerator = new terrain::TPerlin1DProfileGenerator( _octaves,
                                                                                _persistance,
                                                                                _lacunarity,
                                                                                _noiseScale );
                }
                else
                {
                    _profileGenerator = new terrain::TSineProfileGenerator( 2.0f, 10.0f, 1.57f );
                }

                float _sectionDepth         = params.getFloat( "sectionDepth", 1.0f );
                float _componentsSpacingX   = params.getFloat( "componentsSpacingX", 0.5f );
                float _componentsThickness  = params.getFloat( "componentsThickness", 0.01f );

                TVec3 _startPosition = params.getVec3( "startPosition" );

                _terrainGenerator = new terrain::TPathTerrainGenerator( name, 
                                                                        _startPosition.x,
                                                                        _startPosition.y,
                                                                        _startPosition.z,
                                                                        _sectionDepth,
                                                                        _componentsSpacingX,
                                                                        _componentsThickness,
                                                                        _profileGenerator );
            }
            else
            {
                terrain::TBlockyParams _bparams;

                _bparams.usesBase               = params.getInt( "sectionUsesBase" ) == 1;
                _bparams.usesSides              = params.getInt( "sectionUsesSides" ) == 1;
                _bparams.sectionLength          = params.getFloat( "sectionLength" );
                _bparams.baseDepth              = params.getFloat( "sectionDepth" );
                _bparams.baseHeight             = params.getFloat( "sectionBlockyBaseHeight" );
                _bparams.baseWidth              = params.getFloat( "sectionBlockyBaseWidth" );
                _bparams.baseSpacingX           = params.getFloat( "sectionBlockyBaseSpacingX" );
                _bparams.baseOffsetZ            = params.getFloat( "sectionBlockyBaseOffsetZ" );
                _bparams.percentDepth.min       = params.getFloat( "sectionBlockyPercentDepthMin" );
                _bparams.percentDepth.max       = params.getFloat( "sectionBlockyPercentDepthMax" );
                _bparams.percentHeight.min      = params.getFloat( "sectionBlockyPercentHeightMin" );
                _bparams.percentHeight.max      = params.getFloat( "sectionBlockyPercentHeightMax" );
                _bparams.percentWidth.min       = params.getFloat( "sectionBlockyPercentWidthMin" );
                _bparams.percentWidth.max       = params.getFloat( "sectionBlockyPercentWidthMax" );
                _bparams.percentSpacingX.min    = params.getFloat( "sectionBlockyPercentSpacingXMin" );
                _bparams.percentSpacingX.max    = params.getFloat( "sectionBlockyPercentSpacingXMax" );
                _bparams.percentOffsetZ.min     = params.getFloat( "sectionBlockyPercentOffsetZMin" );
                _bparams.percentOffsetZ.max     = params.getFloat( "sectionBlockyPercentOffsetZMax" );

                TVec3 _startPosition = params.getVec3( "startPosition" );

                _terrainGenerator = new terrain::TBlockyTerrainGenerator( name,
                                                                          _startPosition.x,
                                                                          _startPosition.y,
                                                                          _startPosition.z,
                                                                          _bparams );
            }
        }

        auto _terrainGeneratorWrapper = new TMjcTerrainGenWrapper( name, _terrainGenerator );

        return _terrainGeneratorWrapper;
    }


}}