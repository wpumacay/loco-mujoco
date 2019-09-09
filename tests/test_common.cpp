
#include <tysoc_common.h>


int main()
{

    // string - helpers functionality
    std::string _strSample1 = "/home/gregor/data/foo.txt";
    std::string _strSample2 = "./documents/bar.json";
    std::string _strSample3 = "~/downloads/path-funky/even_more::funky/fun-var|fun@jojojo.stl";

    std::vector< std::string > _strSamples = { _strSample1, _strSample2, _strSample3 };

    for ( size_t i = 0; i < _strSamples.size(); i++ )
    {
        std::cout << "split-parts of: " << _strSamples[i] << std::endl;

        auto _parts = tysoc::split( _strSamples[i], '/' );
        for ( size_t j = 0; j < _parts.size(); j++ )
            std::cout << _parts[j] << std::endl;

        std::cout << "filename: " << tysoc::getFilenameFromFilePath( _strSamples[i] ) << std::endl;
        std::cout << "foldername: " << tysoc::getFoldernameFromFilePath( _strSamples[i] ) << std::endl;
        std::cout << "folderpath: " << tysoc::getFolderpathFromFilePath( _strSamples[i] ) << std::endl;
        std::cout << "filename-no-extension: " << tysoc::getFilenameNoExtensionFromFilePath( _strSamples[i] ) << std::endl;

        std::cout << "-----------------------------------" << std::endl;
    }

    return 0;
}