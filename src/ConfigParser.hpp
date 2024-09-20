#pragma once

#ifndef HMMMOS_CONFIGPARSER_H_
#define HMMMOS_CONFIGPARSER_H_

#include <iostream>
#include <stdlib.h>
#include <string>
#include "yaml-cpp/yaml.h"

/**
 * @brief A class to handle the algorithm configuration parsing.
 * 
 */
class ConfigParser
{
    public:
        /**
         * @brief Construct a new Config Parser object.
         * 
         * @param argc          Number of commandline arguments.
         * @param yamlFilePath  Commandline arguments.
         */
        ConfigParser(int argc, char** yamlFilePath);

        /**
         * @brief Destroy the Config Parser object.
         * 
         */
        ~ConfigParser();

        /**
         * @brief Parse the algrithm configuration path.
         * 
         * @return int Returns 0 if algorithm parsing was successful and 1
         *             otherwise.
         */
        int parseConfig();

    public:
        // The 3D spatial convolution size (m by m by m).
        unsigned int convSize;

        // The local window size to extend the 3D convolution to a
        // 4D convolution.
        unsigned int localWindowSize;

        // The global window size used to maintain the map size and
        //  account for unertainties in the pose estimates.
        unsigned int globalWindowSize;
        
        // The window size to retain high-confidence dyanmic detections.
        unsigned int dynamicRegionWindowSize;

        // Number of scans to delay the dyanmic detection.
        unsigned int numScansDelay;
        
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

    private:
        // Path to the algorithm configuration file.
        std::string yamlFilePath_;
};

#endif // HMMOS_CONFIGPARSER_H_