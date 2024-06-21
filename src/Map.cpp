#include "Map.hpp"

Map::Map(const ConfigParser &config)
    : Scan(config)
{
    globalWinLen_ = config.globalWindowSize;
    qcd.beliefThreshold = config.beliefThreshold;
    qcd.sigFree = config.freeSigma;
    qcd.sigOcc = config.occupancySigma;
    maxRange_ = config.maxRange;
    minRange_ = config.minRange;
    voxelSize_ = config.voxelSize;
    convSize_ = config.convSize;
    edge_ = (convSize_-1)/2;

    normDistOccDen_ = 1/(2*pow(qcd.sigOcc, 2));
    normDistFreeDen_ = 1/(2*pow(qcd.sigFree, 2));

    // Hardcoded parameters.
    nBins_ = 100; // Used for histogram thresholding.
}

Map::~Map()
{

}

void Map::addPoints(const std::vector<Eigen::Vector3i> &points)
{
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        if (map_.contains(point))
        {
            map_[point].currentStateScan = scanNum_;
        } else
        {
            MapVoxelState v;
            v.currentStateScan = scanNum_;
            map_.insert({point, v});
        }
    });
}

void Map::update(Scan &scan, unsigned int scanNum)
{
    scanNum_ = scanNum;
    sensorPose = scan.sensorPose;

    // Construct a KD-Tree of the measurements.
    convertToPointCloudKdTree(scan.ptsOccupiedOverWindow);
    my_kd_tree_t *ptCloudKdTree = new my_kd_tree_t(3, pcForKdTree_, {10});

    // Update the observed voxels in the map.
    addPoints(scan.observedVoxels);

    // std::vector<Eigen::Vector3i> pointsObservedUnique;
    // for (const auto &x : map_)
    // {
    //     if (x.second.currentStateScan == scanNum)
    //         pointsObservedUnique.push_back(x.first);
    // }
    // tbb::parallel_for(
    // tbb::blocked_range<int>(0,scan.observedVoxels.size()),
    // [&](tbb::blocked_range<int> r)
    tbb::parallel_for(
    tbb::blocked_range<pointsIterator>(scan.observedVoxels.cbegin(),scan.observedVoxels.cend()),
    [&](tbb::blocked_range<pointsIterator> r)
    {
        for (const auto &voxel : r)
        {
            
            size_t numResults = 1;
            uint32_t retIndex;
            double pt[] = {voxel.coeffRef(0)*voxelSize_,
                           voxel.coeffRef(1)*voxelSize_,
                           voxel.coeffRef(2)*voxelSize_};
            ptCloudKdTree->knnSearch(&pt[0], numResults, &retIndex,
                                     &map_[voxel].closestDistance);
            // map_[voxel].closestDistance = outDistSqr;
            
            // Update voxel likelihoods.
            Eigen::Matrix3d B;
            B << 0, 0, 0,
                 0, exp(-map_[voxel].closestDistance*normDistOccDen_), 0,
                 0, 0, (1 - exp(-map_[voxel].closestDistance*normDistFreeDen_));

            Eigen::Vector3d alpha = B * qcd.stateTransitionMatrix 
                                      * map_[voxel].xHat;
            map_[voxel].xHat = alpha/alpha.sum();
            map_[voxel].scanLastSeen = scanNum;
            
            // Update the voxel state if the probability of being in that state is greater than the predefined threshold.
            // std::vector<double> vec(voxelMap.map_[pointsObservedUnique[i]].xHat.data(), voxelMap.map_[pointsObservedUnique[i]].xHat.data() + qcdParams.numStates);                
            // double maxElement = *std::max_element(&vec[0], &vec[0]+qcdParams.numStates);
            // int maxElementIndex = std::find(&vec[0], &vec[0]+qcdParams.numStates, maxElement) - &vec[0];

            int maxElementIndex = map_[voxel].xHat(0) >
                                  map_[voxel].xHat(1) ? 0 : 1;
            maxElementIndex = map_[voxel].xHat(2) > 
                              map_[voxel].xHat(maxElementIndex) ? 2 : maxElementIndex;
            double maxElement = map_[voxel].xHat(maxElementIndex);

            if (maxElement > qcd.beliefThreshold)// && maxElement != voxelStates[voxelNum].currentState[0]
            { 
                // Check for a change in state.
                // Looking for a change in state from free to occupied and vice versa.
                if ((maxElementIndex != map_[voxel].currentState) && 
                    (maxElementIndex != 0 && map_[voxel].currentState != 0))
                {
                    map_[voxel].lastStateChangeScan[0] = map_[voxel].lastStateChangeScan[1]; 
                    map_[voxel].lastStateChangeScan[1] = scanNum;
                }

                // Save the current state and the time it was observed.
                map_[voxel].currentState = maxElementIndex;
                // voxelMap.map_[pointsObservedUnique[i]].currentStateScan = scanNum;
            }
            // Update the scan the last time the voxel was observed.
            map_[voxel].currentStateScan = scanNum; 
        }
    });
    // Remove map points outside the maximum range and the frame sliding window.
    removeVoxelsOutsideWindowAndMaxRange();

    delete ptCloudKdTree;
}

unsigned int Map::getMapSize()
{
    return map_.size();
}

void Map::removeVoxelsOutsideWindowAndMaxRange()
{
    std::vector<Eigen::Vector3i> ps;
    double ptNorm;
    double ptDiffX, ptDiffY, ptDiffZ;
    for (const auto &[pt, state] : map_)
    {
        ptDiffX = pt.coeffRef(0)*voxelSize_-sensorPose.coeffRef(0,3);
        ptDiffY = pt.coeffRef(1)*voxelSize_-sensorPose.coeffRef(1,3);
        ptDiffZ = pt.coeffRef(2)*voxelSize_-sensorPose.coeffRef(2,3);
        ptNorm = (ptDiffX*ptDiffX) + (ptDiffY*ptDiffY) + (ptDiffZ*ptDiffZ); 
        if (ptNorm  > ((maxRange_*2)*(maxRange_*2)) || 
           (state.scanLastSeen > globalWinLen_ && 
            state.scanLastSeen < scanNum_- globalWinLen_))
        {
            ps.push_back(pt);
        }
    }
    
    for (const auto &x : ps)
        map_.erase(x);
}

void Map::findDynamicVoxels(Scan &scan, boost::circular_buffer<Scan> &scanHistory)
{
    // std::cout << scan.occupiedVoxels.size() << std::endl; exit(1);
    // std::vector<double> tempScores(scan.occupiedVoxels.size(),0.0);
    tbb::parallel_for(
    tbb::blocked_range<pointsIterator>(scan.occupiedVoxels.cbegin(), scan.occupiedVoxels.cend()),
    [&](tbb::blocked_range<pointsIterator> r)
    {
        for (const auto &voxel : r)
        {
            // Extract the point indicies.
            int x = voxel.coeffRef(0);
            int y = voxel.coeffRef(1);
            int z = voxel.coeffRef(2);
            
            // Find the neighbours in the convolution size.
            std::vector<Eigen::Vector3i> nPtsValid;
            for (int i = x - edge_; i < x + edge_ + 1; ++i)
            {
                for (int j = y - edge_; j < y + edge_ + 1; ++j)
                {
                    for (int k = z - edge_; k < z + edge_ + 1; ++k)
                    {
                        nPtsValid.push_back({i, j, k});
                    }
                }
            }
            bool voxHasUnobservedNeighbor = false;
            double totalScore = 0;
            for (unsigned int i = 0; i < nPtsValid.size(); i++)
            {
                // double score = 0;
                // if (nPtsValid[i](0) != x && nPtsValid[i](1) != y && nPtsValid[i](2) != z)
                {
                    // Check if the voxel exists in the map.
                    if (map_.contains(nPtsValid[i]) &&
                       (map_[nPtsValid[i]].currentState != 0) && 
                       (scanNum_ - map_[nPtsValid[i]].currentStateScan < globalWinLen_))
                    {
                        if (map_[nPtsValid[i]].lastStateChangeScan[1] == scanNum_ &&
                            scan.checkIfEntryExists(nPtsValid[i]))
                        {
                            totalScore = totalScore + 1;
                            // totalScore = totalScore + map_[nPtsValid[i]].xHat(1);
                        }
                    } else
                    {
                        // voxHasUnobservedNeighbor = true;
                        // break;
                        totalScore = totalScore - 1;
                    }
                }
            }
            totalScore = std::max(totalScore,0.0);
            if (!voxHasUnobservedNeighbor)
                scan.setConvScore(voxel, totalScore);
        }
    });
    
    // Clean the convolution scores.
    findMedianValue(scan);

    // Save the scan in the scan history.
    scanHistory.push_back(scan);

    // Extend the spatial convolution score to a spatio-temporal convolution.
    std::vector<double> scoresOverWindow;
    for (auto x : scanHistory)
    {
        for (auto y : x.occupiedVoxels)
        {
            if (scan.checkIfEntryExists(y))
            {
                scan.setConvScoreOverWindow(y, scan.getConvScoreOverWindow(y)+x.getConvScore(y)); 
            }
        }
    }

    for (auto x : scan.occupiedVoxels)
    {
        scoresOverWindow.push_back(scan.getConvScoreOverWindow(x));
    }

    // Find the dynamic voxels using the convolution scores.
    // These are the high confidence dynamic voxel estimates.
    Eigen::VectorXd binCounts = Eigen::VectorXd::Zero(nBins_);
    Eigen::VectorXd edges;
    findHistogramCounts(nBins_, scoresOverWindow, binCounts, edges);
    // findHistogramCounts(nBins, scores, binCounts, edges);
    int level = otsu(binCounts);    
    double thresholdLimit = 0;
    scan.dynThreshold = 0;
    if (level > 0)
    {
        thresholdLimit = edges(level-1);
        scan.dynThreshold = thresholdLimit;
        // thresholdLimit = 15;
        for (unsigned int j = 0; j < scan.occupiedVoxels.size(); j++)
        {
            if (scan.getConvScoreOverWindow(scan.occupiedVoxels[j]) > thresholdLimit)
            {
                scan.setDynamicHighConfidence(scan.occupiedVoxels[j]);
                // scanDynamicVoxels_.push_back(scan.occupiedVoxels[j]);
            }
        }
    }

    // for (auto x : prevScanDynamicVoxels_)
    // {
    //     scan.setDynamic(x);
    // }

    // for (auto x : scanHcDynamicVoxels_)
    // {
    //     scan.setDynamic(x);
    // }
    // scanHcDynamicVoxels_.clear();
    // for (auto x : scan.occupiedVoxels)
    // {
    //     if (scan.getDynamic(x))
    //     {
    //         scanHcDynamicVoxels_.push_back(x);
    //         // prevScanHcDynamicVoxels_.push_back(x);
    //     }
    // }

       // scanDynamicVoxels_.clear();
    // Save the high confidence dynamic detections to be used for the next scan.

    for (auto x: prevScanDynamicVoxels_)
    {
        // if (scan.checkIfEntryExists(x) && scan.getConvScoreOverWindow(x) > 0)
        // if (prevScanOccupancy.CheckIfEntryExists(x) && prevScanOccupancy.map_[x].convScore > 3)
        if (scan.checkIfEntryExists(x) && scan.getConvScoreOverWindow(x) > 3)
        {
            scan.setDynamicHighConfidence(x);
        }
    }

    // prevScanDynamicVoxels_.clear();
    // for (auto x : scan.occupiedVoxels)
    // {
    //     if (scan.getDynamic(x))
    //     {
    //         prevScanDynamicVoxels_.push_back(x);
    //         // voxelMap.map_[x.first].isDynamic = true;
    //     } 
    //     // else
    //     // {
    //     //     voxelMap.map_[x.first].isDynamic = false;

    //     // }
    // }

    // Region growing.
    int dilSize = convSize_;
    Eigen::Vector3i p;

    // Perform a dilation.
    scanDynamicVoxels_.clear();
    dilSize = 3;
    for (auto &vox : scan.occupiedVoxels)
    {
        if (scan.getDynamicHighConfidence(vox))
        {
            int x = vox(0);
            int y = vox(1);
            int z = vox(2);
            int edge = (dilSize-1)/2;
            for (int i = x - edge; i < x + edge + 1; ++i)
            {
                for (int j = y - edge; j < y + edge + 1; ++j)
                {
                    for (int k = z - edge; k < z + edge + 1; ++k)
                    {
                        scanDynamicVoxels_.push_back({i, j, k});
                    }
                }
            }
        }
    }

    for (auto x : scanDynamicVoxels_)
    {
        if (scan.checkIfEntryExists(x))
        {
            scan.setDynamicHighConfidence(x);
        }
    }


    // for (auto x : prevScanHcDynamicVoxels_)
    // {
    //     if (scan.checkIfEntryExists(x))
    //     {
    //         scan.setDynamic(x);
    //     }
    // }

    // prevScanHcDynamicVoxels_.clear();
    // for (auto x : scanHcDynamicVoxels_)
    //     prevScanHcDynamicVoxels_.push_back(x);

    // prevScanHcDynamicVoxels_ = scanHcDynamicVoxels_;

    

    // scanDynamicVoxels_.clear();
    // Save the high confidence dynamic detections to be used for the next scan.
    prevScanDynamicVoxels_.clear();
    for (auto x : scan.occupiedVoxels)
    {
        if (scan.getDynamicHighConfidence(x))
        {
            prevScanDynamicVoxels_.push_back(x);
            // voxelMap.map_[x.first].isDynamic = true;
        } 
        // else
        // {
        //     voxelMap.map_[x.first].isDynamic = false;

        // }
    }
    
    // std::cout << scanNum_ << " " << thresholdLimit << std::endl;
    
}

void Map::findMedianValue(Scan &scan)
{
    // Clean the convolution scores.
    // Average the convolution scores around the neighbour.
    Scan scanOriginal = scan;
    // VoxelHashMap currentScanOccupancyCopy = prevScanOccupancy;
    // for (auto &[vox,state] : prevScanOccupancy.map_)
    for (auto &vox : scan.occupiedVoxels)
    {
        Eigen::Vector3i p;
        std::vector<double> voxScores;
        int x = vox(0);
        int y = vox(1);
        int z = vox(2);
        int edge = (3-1)/2; // nearest neighbour
        for (int i = x - edge; i < x + edge + 1; ++i)
        {
            for (int j = y - edge; j < y + edge + 1; ++j)
            {
                for (int k = z - edge; k < z + edge + 1; ++k)
                {
                    p << i, j, k;
                    if (scan.checkIfEntryExists(p))
                    {
                        voxScores.push_back(scan.getConvScore(p));
                    }
                }
            }
        }
        int val = findMedian(voxScores); 
        scanOriginal.setConvScore(vox, val);
    }

    for (auto &vox : scan.occupiedVoxels)
        scan.setConvScore(vox, scanOriginal.getConvScore(vox));
}