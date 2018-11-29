
#include <utils/urdf/UrdfParser.h>


#ifndef TYSOCBULLET_RESOURCES_PATH
    #define TYSOCBULLET_RESOURCES_PATH "../../res/"
#endif


int main()
{

    auto _parser = new urdf::UrdfParser();

    std::cout << "INFO> started parsing" << std::endl;

    std::string _filepath;
    _filepath += TYSOCBULLET_RESOURCES_PATH;
    _filepath += "urdf/laikago.urdf";

    std::cout << "loading file: " << _filepath << std::endl;

    _parser->loadUrdf( _filepath.c_str(), false );

    std::cout << "INFO> finished parsing" << std::endl;

    delete _parser;

    return 0;
}