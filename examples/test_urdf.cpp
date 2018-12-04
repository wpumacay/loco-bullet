
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
    _filepath += "urdf/laikago/laikago.urdf";
    // _filepath += "urdf/atlas/atlas.urdf";
    // _filepath += "urdf/baxter/baxter.urdf";
    // _filepath += "urdf/cassie/cassie.urdf";

    std::cout << "loading file: " << _filepath << std::endl;

    _parser->loadUrdf( _filepath.c_str(), false );

    std::cout << "INFO> finished parsing" << std::endl;

    std::cout << "TESTING SOME INFO FROM URDF" << std::endl;

    auto& _urdfModel = _parser->getModel();

    std::cout << "modelName: "      << _urdfModel.m_name << std::endl;
    std::cout << "sourceFile: "     << _urdfModel.m_sourceFile << std::endl;
    std::cout << "numRootLinks: "   << _urdfModel.m_rootLinks.size() << std::endl;
    std::cout << "numLinks: "       << _urdfModel.m_links.size() << std::endl;
    std::cout << "numJoints: "      << _urdfModel.m_joints.size() << std::endl;
    std::cout << "numMaterials: "   << _urdfModel.m_materials.size() << std::endl;


    delete _parser;

    return 0;
}