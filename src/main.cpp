#include <boost/circular_buffer.hpp>
#include <chrono>

#include "ConfigParser.hpp"
#include "Map.hpp"
#include "Scan.hpp"
#include "utils.hpp"

int main(int argc, char** argv)
{
    // Argument parsing
    ConfigParser configParser(argc, argv);
    int configStatus = configParser.parseConfig();
    if (configStatus) exit(1);
    bool printTimes = false; // TODO: Shift to config?

    std::ofstream outFile;
    if (configParser.outputFile)
    {
        outFile.open(configParser.outputFileName, std::ios::out);
    }

    // Load the scan paths and the pose estimates
    std::vector<std::string> scanFiles;
    for (auto const& dir_entry : std::filesystem::directory_iterator(configParser.scanPath))
    { 
        scanFiles.push_back(dir_entry.path());
    }
    std::sort(scanFiles.begin(), scanFiles.end(), compareStrings);
    unsigned int numScans = scanFiles.size();
    
    // Read the pose estimates in KITTI format
    std::vector<std::vector<double> > poseEstimates = readPoseEstimates(configParser.posePath); 
    if (scanFiles.size() != poseEstimates.size())
    {
        std::cerr << "The number of scans (" << scanFiles.size() 
                  <<  ") do not match the number of poses (" 
                  << poseEstimates.size() << ")!" << std::endl;
        exit(1);
    }

    Scan scan(configParser);
    Map map(configParser);
    boost::circular_buffer<Scan> scanHistory(configParser.localWindowSize);

    // Hardcoded parameters
    // These hmmConfig parameters stem from the design of the algorithm and are
    // not changed for the MOS task.
    map.hmmConfig.numStates = 3;
    double epsilon = 0.005;
    map.hmmConfig.stateTransitionMatrix.resize(map.hmmConfig.numStates,
                                               map.hmmConfig.numStates);
    map.hmmConfig.stateTransitionMatrix << 1.0-2*epsilon,  0.00,  0.00,
                                           epsilon, 1.0-epsilon, epsilon,
                                           epsilon, epsilon, 1.0-epsilon;

    // Estimate the per-scan dynamic detections.
    for (unsigned int scanNum = 0; scanNum < numScans; scanNum++)
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
        
        if (scanNum > configParser.localWindowSize)
        {
            // Estimate dynamic voxels.
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

        // Write results.
        if (configParser.outputFile)
        {
            if (configParser.scanNumsToPrint.contains(scanNum+1))
            {
                scan.writeFile(outFile, scanNum);
            }
        }

        if (configParser.outputLabels)
        {
            scan.writeLabel(scanNum);
        }

        // Print the current time stamp to terminal.
        printf("\rScan: %d", scanNum);
        fflush(stdout);
    }

    std::cout << std::endl;
}