
#include <json.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>

#ifndef JSON_TEST_RESOURCES
    #define JSON_TEST_RESOURCES "../../../res/templates/rlsim/"
#endif

static std::string MODEL_NAME = "dog3d.json";

#define JSON_KEY_SKELETON       "Skeleton"
#define JSON_KEY_JOINTS         "Joints"
#define JSON_KEY_BODIES         "BodyDefs" // defines collisions along body
#define JSON_KEY_VISUALS        "DrawShapeDefs"
#define JSON_KEY_PD_CONTROLLERS "PDControllers"

int main( int argc, const char** argv )
{
    if ( argc > 1 )
    {
        try
        {
            MODEL_NAME = std::string( argv[1] );
        }
        catch ( const std::exception& e )
        {
            std::cout << "ERROR> Should pass an string for the model template" << std::endl;
            std::cerr << e.what() << '\n';
            return 1;
        }
    }

    // set full path to the .json resource
    std::string _modelName;
    _modelName += JSON_TEST_RESOURCES;
    _modelName += MODEL_NAME;

    // open the resource
    std::cout << "LOG> trying to open resource: " << _modelName << std::endl;
    std::ifstream _modelFileHandle;
    _modelFileHandle.open( _modelName.c_str(), std::ifstream::in );
    std::cout << "LOG> finished opening resource" << std::endl;

    // decode json reosurce
    std::cout << "LOG> starting json decoding" << std::endl;
    nlohmann::json _modelJson;
    _modelFileHandle >> _modelJson;
    std::cout << "LOG> finished json decoding" << std::endl;

    // release file handle
    _modelFileHandle.close();

    // // just dump for a sanity check
    // std::cout << "json ****************************" << std::endl;
    // std::cout <<_modelJson.dump( 4 ) << std::endl;
    // std::cout << "*********************************" << std::endl;

    // children of root model
    for ( auto& el : _modelJson.items() )
    {
        std::cout << "key: " << el.key() << std::endl;
    }

    std::vector< std::string > _jointsInfo;
    std::vector< std::string > _bodiesInfo;
    std::vector< std::string > _visualsInfo;
    std::vector< std::string > _pdControllersInfo;

    size_t _maxLength = 0;

    if ( _modelJson.find( JSON_KEY_SKELETON ) != _modelJson.end() )
    {
        auto _skeletonJson = _modelJson[ JSON_KEY_SKELETON ];

        // std::cout << "Skeleton *********" << std::endl;
        // std::cout << _skeletonJson.dump( 4 ) << std::endl;
        // std::cout << "******************" << std::endl;

        if ( _skeletonJson.find( JSON_KEY_JOINTS ) != _skeletonJson.end() )
        {
            auto _jointsJson = _skeletonJson[ JSON_KEY_JOINTS ];

            // std::cout << "Joints ***********" << std::endl;
            // std::cout << _jointsJson.dump( 4 ) << std::endl;
            // std::cout << "******************" << std::endl;

            std::cout << "numJoints: " << _jointsJson.size() << std::endl;
            _jointsInfo.push_back( std::string( "joints: " ) + std::to_string( _jointsJson.size() ) );
            _maxLength = std::max( _maxLength, _jointsJson.size() );

            for ( size_t i = 0; i < _jointsJson.size(); i++ )
            {
                _jointsInfo.push_back( std::string( _jointsJson[i]["Name"] ) );
                // std::cout<< _jointsJson[i]["Name"] << std::endl;
            }
        }
    }

    if ( _modelJson.find( JSON_KEY_BODIES ) != _modelJson.end() )
    {
        auto _bodiesJson = _modelJson[ JSON_KEY_BODIES ];

        // std::cout << "Bodies ***********" << std::endl;
        // std::cout << _bodiesJson.dump( 4 ) << std::endl;
        // std::cout << "******************" << std::endl;

        std::cout << "numBodies: " << _bodiesJson.size() << std::endl;
        _bodiesInfo.push_back( std::string( "bodies: " ) + std::to_string( _bodiesJson.size() ) );
        _maxLength = std::max( _maxLength, _bodiesJson.size() );

        for( size_t i = 0; i < _bodiesJson.size(); i++ )
        {
            _bodiesInfo.push_back( std::string( _bodiesJson[i]["Name"] ) );
            // std::cout << _bodiesJson[i]["Name"] << std::endl;
        }
    }

    if ( _modelJson.find( JSON_KEY_VISUALS ) != _modelJson.end() )
    {
        auto _visualsJson = _modelJson[ JSON_KEY_VISUALS ];

        // std::cout << "Visuals **********" << std::endl;
        // std::cout << _visualsJson.dump( 4 ) << std::endl;
        // std::cout << "******************" << std::endl;

        std::cout << "numVisuals: " << _visualsJson.size() << std::endl;
        _visualsInfo.push_back( std::string( "visuals: " ) + std::to_string( _visualsJson.size() ) );
        _maxLength = std::max( _maxLength, _visualsJson.size() );

        for ( size_t i = 0; i < _visualsJson.size(); i++ )
        {
            _visualsInfo.push_back( std::string( _visualsJson[i]["Name"] ) );
            // std::cout << _visualsJson[i]["Name"] << std::endl;
        }
    }

    if ( _modelJson.find( JSON_KEY_PD_CONTROLLERS ) != _modelJson.end() )
    {
        auto _pdControllersJson = _modelJson[ JSON_KEY_PD_CONTROLLERS ];

        // std::cout << "PDControllers ****" << std::endl;
        // std::cout << _pdControllersJson.dump( 4 ) << std::endl;
        // std::cout << "******************" << std::endl;

        std::cout << "numPDControllers: " << _pdControllersJson.size() << std::endl;
        _pdControllersInfo.push_back( std::string( "pdcontrollers: " ) + std::to_string( _pdControllersJson.size() ) );
        _maxLength = std::max( _maxLength, _pdControllersJson.size() );

        for ( size_t i = 0; i < _pdControllersJson.size(); i++ )
        {
            _pdControllersInfo.push_back( std::string( _pdControllersJson[i]["Name"] ) );
            // std::cout << _pdControllersJson[i]["Name"] << std::endl;
        }
    }

    std::cout << "maxlen: " << _maxLength << std::endl;

    for ( size_t q = 0; q < _maxLength; q++ )
    {
        if ( q < _jointsInfo.size() )
            std::cout << std::setw( 20 ) << _jointsInfo[q] << "\t\t";
        else
            std::cout << std::setw( 20 ) << "--------" << "\t\t";

        if ( q < _bodiesInfo.size() )
            std::cout << std::setw( 20 ) << _bodiesInfo[q] << "\t\t";
        else
            std::cout << std::setw( 20 ) << "--------" << "\t\t";

        if ( q < _visualsInfo.size() )
            std::cout << std::setw( 20 ) << _visualsInfo[q] << "\t\t";
        else
            std::cout << std::setw( 20 ) << "--------" << "\t\t";

        if ( q < _pdControllersInfo.size() )
            std::cout << std::setw( 20 ) << _pdControllersInfo[q] << "\t\t";
        else
            std::cout << std::setw( 20 ) << "--------" << "\t\t";

        std::cout << std::endl;
    }

    return 0;
}