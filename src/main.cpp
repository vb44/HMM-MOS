#include "ConfigParser.hpp"
#include "Scan.hpp"
#include "Map.hpp"
#include "utils.hpp"

#include <boost/circular_buffer.hpp>
#include <chrono>

int main(int argc, char** argv)
{
    // ------------------------------------------------------------------------
    // ARGUMENT PARSING
    // ------------------------------------------------------------------------
    ConfigParser configParser(argc, argv);
    int configStatus = configParser.parseConfig();
    if (configStatus) exit(1);

    std::ofstream outFile;
    if (configParser.outputFile)
    {
        outFile.open(configParser.outputFileName, std::ios::out);
    }

    // ------------------------------------------------------------------------
    // LOAD THE SCAN PATHS AND THE POSE ESTIMATES
    // ------------------------------------------------------------------------
    std::vector<std::string> scanFiles;
    for (auto const& dir_entry : std::filesystem::directory_iterator(configParser.scanPath))
    { 
        scanFiles.push_back(dir_entry.path());
    }

    // Sort the scans in order of the file name.
    std::sort(scanFiles.begin(), scanFiles.end(), compareStrings);

    // Number of scans.
    unsigned int numScans = scanFiles.size();
    
    // Read the pose estimates in KITTI format.
    std::vector<std::vector<double> > poseEstimates = readPoseEstimates(configParser.posePath); 
    if (scanFiles.size() != poseEstimates.size())
    {
        std::cerr << "The number of scans (" << scanFiles.size() 
                  <<  ") do not match the number of poses (" 
                  << poseEstimates.size() << ")!" << std::endl;
        exit(1);
    }

    // ------------------------------------------------------------------------
    // INSTANTIATE CONTAINERS
    // ------------------------------------------------------------------------
    Scan scan(configParser);
    Map map(configParser);
    boost::circular_buffer<Scan> scanHistory(configParser.localWindowSize);

    // ------------------------------------------------------------------------
    // HARDCODED PARAMETERS
    // ------------------------------------------------------------------------
    // These hmmConfig parameters stem from the design of the algorithm and are not
    // changed for the MOS task.
    map.hmmConfig.numStates = 3;
    map.hmmConfig.stateTransitionMatrix.resize(map.hmmConfig.numStates,
                                               map.hmmConfig.numStates);
    map.hmmConfig.stateTransitionMatrix << 0.99,  0.00,  0.00,
                                           0.005, 0.995, 0.005,
                                           0.005, 0.005, 0.995;

    // ------------------------------------------------------------------------
    // Main loop.
    // ------------------------------------------------------------------------
    bool printTimes = false;
    for (unsigned int scanNum = configParser.startScan-1;
                      scanNum < configParser.endScan; scanNum++)
    {
        auto startScanTimer = std::chrono::high_resolution_clock::now();

        // Read the new scan.
        scan.readScan(scanFiles[scanNum], poseEstimates[scanNum]);
        auto finishReadScan = std::chrono::high_resolution_clock::now();

        // Voxelize the scan to construct the occupancy map and find all the
        // observed voxels.
        scan.voxelizeScan();
        auto finishVoxelizeScan = std::chrono::high_resolution_clock::now();

        // Update the map.
        map.update(scan, scanNum);
        auto finishMapUpdate = std::chrono::high_resolution_clock::now();
        
        if (scanNum >= (configParser.startScan + configParser.localWindowSize))
        {
            // Determine dynamic voxels.
            map.findDynamicVoxels(scan, scanHistory);
            auto finishFindDynamicVoxel = std::chrono::high_resolution_clock::now();
            
            if (printTimes)
            {
                std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(finishReadScan - startScanTimer).count() << " "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(finishVoxelizeScan - finishReadScan).count() << " "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(finishMapUpdate - finishVoxelizeScan).count() << " "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(finishFindDynamicVoxel - finishMapUpdate).count() << std::endl;
            }
        } else
        {
            scanHistory.push_back(scan);
        }

        if (configParser.outputFile)
        {
            if (configParser.scanNumsToPrint.contains(scanNum+1-configParser.offset))
            {
                scan.writeFile(outFile, scanNum);
            }
        }

        // --------------------------------------------------------------------
        // Write labels.
        // --------------------------------------------------------------------
        if (configParser.outputLabels)
        {
            scan.writeLabel(scanNum);
        }
    }
}