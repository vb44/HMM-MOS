#include "ConfigParser.hpp"

ConfigParser::ConfigParser(int argc, char** yamlFilePath)
{
    if (argc != 2)
    {
        std::cerr << "Usage: ./hmmMOS config_file.yaml" << std::endl;
        exit(EXIT_FAILURE);
    }
    yamlFilePath_ = yamlFilePath[1];
}

ConfigParser::~ConfigParser()
{
}

int ConfigParser::parseConfig()
{
    try
    {
        YAML::Node configFromYaml = YAML::LoadFile(yamlFilePath_);
        
        startScan = configFromYaml["startScan"].as<unsigned int>();
        endScan = configFromYaml["endScan"].as<unsigned int>();
        convSize = configFromYaml["convSize"].as<unsigned int>();
        localWindowSize = configFromYaml["localWindowSize"].as<unsigned int>();
        globalWindowSize = configFromYaml["globalWindowSize"].as<unsigned int>();
        occupancySigma = configFromYaml["occupancySigma"].as<double>();
        freeSigma = configFromYaml["freeSigma"].as<double>();
        beliefThreshold = configFromYaml["beliefThreshold"].as<double>();
        voxelSize = configFromYaml["voxelSize"].as<double>();
        minRange = configFromYaml["minRange"].as<double>();
        maxRange = configFromYaml["maxRange"].as<double>();
        minOtsu = configFromYaml["minOtsu"].as<double>();
        outputFile = configFromYaml["outputFile"].as<bool>();
        outputLabels = configFromYaml["outputLabels"].as<bool>();
        scanPath = configFromYaml["scanPath"].as<std::string>();
        posePath = configFromYaml["posePath"].as<std::string>();
        outputFileName = configFromYaml["outputFileName"].as<std::string>();
        outputLabelFolder = configFromYaml["outputLabelFolder"].as<std::string>();
    
        for (auto x : configFromYaml["scanNumsToPrint"])
        {
            scanNumsToPrint[x.as<int>()] = 1;
        }

    } catch(const YAML::BadFile& e)
    {
        std::cerr << e.msg << std::endl;
        return 1;
    } catch(const YAML::ParserException& e) 
    {
        std::cerr << e.msg << std::endl;
        return 1;
    }

    return 0;
}