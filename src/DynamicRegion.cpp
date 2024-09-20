#include "DynamicRegion.hpp"

DynamicRegion::DynamicRegion(const ConfigParser &config)
    : dynamicRegionWinLen_(config.dynamicRegionWindowSize)
    , maxRange_(config.maxRange)
    , minOtsu_(config.minOtsu)
    , voxelSize_(config.voxelSize)
    , nBins_(100) // Hardcoded - used for histogram thresholding.
{
}

DynamicRegion::~DynamicRegion()
{
}

void DynamicRegion::removeVoxelsOutsideWindowAndMaxRange()
{
    std::vector<Eigen::Vector3i> ps;
    double ptNorm;
    double ptDiffX, ptDiffY, ptDiffZ;
    for (const auto &[pt, state] : map_)
    {
        ptDiffX = pt.coeffRef(0)*voxelSize_-sensorPose_.coeffRef(0,3);
        ptDiffY = pt.coeffRef(1)*voxelSize_-sensorPose_.coeffRef(1,3);
        ptDiffZ = pt.coeffRef(2)*voxelSize_-sensorPose_.coeffRef(2,3);
        ptNorm = (ptDiffX*ptDiffX) + (ptDiffY*ptDiffY) + (ptDiffZ*ptDiffZ); 
        if (ptNorm  > ((maxRange_*2)*(maxRange_*2)) || 
           (scanNum_ > dynamicRegionWinLen_ && 
            state.scanLastSeen < scanNum_- dynamicRegionWinLen_))
        {
            ps.push_back(pt);
        }
    }
    
    for (const auto &x : ps)
    {
        map_.erase(x);
    }
}

void DynamicRegion::update(Scan &scan, unsigned int scanNum)
{
    scanNum_ = scanNum;
    sensorPose_ = scan.sensorPose;

    for (auto [voxel, voxelState] : map_)
    {
        map_[voxel].isDynamicHighConfidence = false;
    }

    // Update the scan-wise dynamic detections in to the temporal static map. 
    for (auto [voxel, voxelState] : scan.scan_)
    {
        if (!map_.contains(voxel))
        {
            DynamicVoxelState voxState;
            map_.insert({voxel,voxState});
        }
        if (map_[voxel].scanLastSeen != scanNum)
        {
            // Only label the voxel as dynamic if the dynamic threshold is greater than the minimum Otsu value.
            // Don't revert back to static once the voxel is set to dynamic.
            if (scan.dynThreshold > minOtsu_ && !map_[voxel].isDynamic)
            {
                map_[voxel].isDynamic = voxelState.isDynamic && !voxelState.isDynamicFromBackendConv;
                map_[voxel].scanLastSeen = scanNum; // The last time the voxel was dynamic.
            }
            map_[voxel].hasUnobservedNeighbour = voxelState.hasUnobservedNeighbour;
        }
    }

    // Remove incorrect dynamic detections in the static map
    // (false positives) by using Otsu thresholding.
    std::vector<double> scores;
    int edge = 1; 
    for (auto [voxel,voxelState] : map_)
    {
        if (voxelState.isDynamic)
        {
            // Extract the point indicies.
            int x = voxel.coeffRef(0);
            int y = voxel.coeffRef(1);
            int z = voxel.coeffRef(2);
            
            // Find the neighbours in the convolution size.
            double score = 0;
            for (int i = x - edge; i < x + edge + 1; ++i)
            {
                for (int j = y - edge; j < y + edge + 1; ++j)
                {
                    for (int k = z - edge; k < z + edge + 1; ++k)
                    {
                        Voxel neighbouringVoxel = {i,j,k};
                        if (map_.contains(neighbouringVoxel) &&
                            map_[neighbouringVoxel].isDynamic)
                        {
                            score++;
                        }
                    }
                }
            }
            map_[voxel].staticConvScore = score;
            scores.push_back(score);
        }
    }

    if (scores.size() > 0)
    {
        Eigen::VectorXd binCounts = Eigen::VectorXd::Zero(nBins_);
        Eigen::VectorXd edges;
        findHistogramCounts(nBins_, scores, binCounts, edges);
        int level = otsu(binCounts);
        if (level > 0)
        {
            staticConvThreshold = edges(level-1);
        }
    }

    // If the dynamic voxel has a dynamic neighbour that is greater than the dynamicity threshold, add it back to the cluster.
    int edge2 = 1; 
    for (auto [voxel,voxelState] : map_)
    {
        if (!voxelState.hasUnobservedNeighbour &&
              voxelState.isDynamic && (voxelState.staticConvScore > staticConvThreshold))
        {
            map_[voxel].isDynamicHighConfidence = true;

            // Extract the point indicies.
            int x = voxel.coeffRef(0);
            int y = voxel.coeffRef(1);
            int z = voxel.coeffRef(2);
            
            // Find the neighbours in the convolution size.
            double score = 0;
            for (int i = x - edge2; i < x + edge2 + 1; ++i)
            {
                for (int j = y - edge2; j < y + edge2 + 1; ++j)
                {
                    for (int k = z - edge2; k < z + edge2 + 1; ++k)
                    {
                        Voxel neighbouringVoxel = {i,j,k};
                        if (map_.contains(neighbouringVoxel) && map_[neighbouringVoxel].isDynamic) // Want to recover the boundary voxels removed due to Otsu thresholding.
                        {
                            map_[neighbouringVoxel].isDynamic = true; // Safer option.
                        }
                    }
                }
            }
        }
    }

    // // Identify missed voxels in the scan.
    // for (auto [voxel, voxelState] : scan.scan_)
    // {
    //     if (!voxelState.hasUnobservedNeighbour && ((map_[voxel].isDynamic && map_[voxel].staticConvScore > staticConvThreshold))) // TODO: Equivalent to high confidence detection?
    //     {
    //         scan.scan_[voxel].isDynamicInDynamicOccupancyRegion = true; // TODO: Uncomment this.
    //     }
    // }
    
    // Maintain the map size.
    removeVoxelsOutsideWindowAndMaxRange();
}

void DynamicRegion::writeMapToFile(long sampleTime, std::string &saveFolderPath)
{
    std::stringstream fNameMaker;
    fNameMaker << saveFolderPath << std::setw(8) << std::setfill('0') << std::to_string(sampleTime) << ".bin";  
    std::string fName;
    fNameMaker >> fName;
    std::ofstream fout(fName, std::ios::binary);
    std::vector<float> p(7);

    for (auto &[voxel, voxelState] : map_)
    {
        p[0] = static_cast<float>(voxel(0)*voxelSize_);
        p[1] = static_cast<float>(voxel(1)*voxelSize_);
        p[2] = static_cast<float>(voxel(2)*voxelSize_);
        p[3] = static_cast<float>(voxelState.isDynamic);
        p[4] = static_cast<float>(voxelState.scanLastSeen);
        p[5] = static_cast<float>(voxelState.hasUnobservedNeighbour);
        p[6] = static_cast<float>(voxelState.staticConvScore > staticConvThreshold);
        fout.write((char*)&p[0], p.size() * sizeof(float));
    }
    fout.close();
}