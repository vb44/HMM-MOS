#pragma once

#ifndef CONFIGPARSER_H
#define CONFIGPARSER_H

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

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
        // The scan to start the algorithm. Zero-based indexing.
        unsigned int startScan;

        // The scan to end the algorithm.
        unsigned int endScan;

        // The 3D spatial convolution size (m by m by m).
        unsigned int convSize;

        // The local window size to extend the 3D convolution to a
        // 4D convolution.
        unsigned int localWindowSize;

        // The global window size used to maintain the map size and
        //  account for unertainties in the pose estimates.
        unsigned int globalWindowSize;
        
        // An offset option to use future scans. This introduced a
        // delay in the scan interpretation.
        unsigned int offset;

        // Mapping voxel size.
        double voxelSize;

        // Voxel occupancy likelihood standard deviation.
        double occupancySigma;

        // Voxel free likelihood standard deviation.
        double freeSigma;

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

#endif