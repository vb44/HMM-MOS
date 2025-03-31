#pragma once

#ifndef HMMMOS_CONFIGPARSER_H_
#define HMMMOS_CONFIGPARSER_H_

#include <iostream>
#include <stdlib.h>
#include <string>
#include "yaml-cpp/yaml.h"

struct ConfigParser
{
    // The 3D spatial convolution size (m by m by m).
    unsigned int convSize;

    // The local window size to extend the 3D convolution to a
    // 4D convolution.
    unsigned int localWindowSize;

    // The global window size used to maintain the map size and
    //  account for unertainties in the pose estimates.
    unsigned int globalWindowSize;

    // Mapping voxel size.
    double voxelSize;

    // Voxel occupancy likelihood standard deviation.
    double occupancySigma;

    // Change detection threshold.
    double beliefThreshold;

    // Minimum sensor range to filter point cloud measurements.
    double minRange;

    // Maximum sensor range to filter point cloud measurements.
    double maxRange;

    // Minimum Otsu threshold for identifying dyanmic voxels.
    double minOtsu;

    // Option to output .label files in the Semantic KITTI format.
    bool outputLabels;

    // Option to output indicies file in the Dynablox format.
    bool outputFile;

    // Scan path. All scans must be in the .bin KITTI format. 
    std::string scanPath;

    // Path to the pose estimates file in the KITTI format.
    std::string posePath;

    // The name of the output file.
    std::string outputFileName;

    // The name of the output folder for the .label files.
    std::string outputLabelFolder;

    // The set of scan numbers to print.
    std::unordered_map<int, int> scanNumsToPrint;

    int parseConfig(std::string yamlFilePath)
    {
        try
        {
            YAML::Node configFromYaml = YAML::LoadFile(yamlFilePath);
            
            convSize = configFromYaml["convSize"].as<unsigned int>();
            localWindowSize = configFromYaml["localWindowSize"].as<unsigned int>();
            globalWindowSize = configFromYaml["globalWindowSize"].as<unsigned int>();
            occupancySigma = configFromYaml["occupancySigma"].as<double>();
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
};

#endif // HMMOS_CONFIGPARSER_H_