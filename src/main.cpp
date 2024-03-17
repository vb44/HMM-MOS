#include "ConfigParser.hpp"
#include "Scan.hpp"
#include "Map.hpp"
#include "utils.hpp"

#include <boost/circular_buffer.hpp>

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
    for (auto const& dir_entry : std::filesystem::directory_iterator(
                                configParser.scanPath))
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

    // These QCD parameters stem from the design of the algorithm and are not
    // changed for the MOS task.
    map.qcd.numStates = 3;
    map.qcd.stateTransitionMatrix.resize(map.qcd.numStates,
                                         map.qcd.numStates);
    map.qcd.stateTransitionMatrix << 0.99, 0.005, 0.005,
                                     0.00, 0.995, 0.005,
                                     0.00, 0.005, 0.995;

    // ------------------------------------------------------------------------
    // Main loop.
    // ------------------------------------------------------------------------
    for (unsigned int scanNum = configParser.startScan-1;
                      scanNum < configParser.endScan; scanNum++)
    {
        // std::cout << scanNum << std::endl;

        // Read the new scan.
        scan.readScan(scanFiles[scanNum], poseEstimates[scanNum]);

        // Voxelize the scan to construct the occupancy map and find all the
        // observed voxels.
        scan.voxelizeScan();
        
        // Update the map.
        map.update(scan, scanNum);

        // for (auto &x : scan.observedVoxels)
        //     std::cout << x.transpose() << std::endl;
        // exit(1);
        
        if (scanNum >= (configParser.startScan + configParser.localWindowSize))
        {
            // std::cout << "here " << scanNum << std::endl;
            // Determine dynamic voxels.
            map.findDynamicVoxels(scan, scanHistory);
        } else
        {
            scanHistory.push_back(scan);
        }

        if (configParser.outputFile)
        {
            if (configParser.scanNumsToPrint.contains(scanNum+1-configParser.offset))
            {
                std::cout << scanFiles[scanNum] << std::endl;
                scan.writeFile(outFile, scanNum);
                // writeFile(scanNum, scan, outFile);
            }
        }

        // std::cout << scanNum << " "  << scan.occupiedVoxels.size() << " " << scan.ptsOccupied.size() << " " <<  scan.observedVoxels.size() <<  " " << map.getMapSize() << std::endl;

    }

}