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
        if (map_.contains(point)) {
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
    tbb::parallel_for(
    tbb::blocked_range<int>(0,scan.observedVoxels.size()),
    [&](tbb::blocked_range<int> r)
    {
        for (unsigned int i = r.begin(); i < r.end(); i++)
        // for (unsigned int i = 0; i < scan.observedVoxels.size(); i++)
        {
            
            size_t numResults = 1;
            uint32_t retIndex;
            double outDistSqr;
            double pt[3];
            pt[0] = scan.observedVoxels[i](0)*voxelSize_;
            pt[1] = scan.observedVoxels[i](1)*voxelSize_;
            pt[2] = scan.observedVoxels[i](2)*voxelSize_;

            ptCloudKdTree->knnSearch(&pt[0], numResults, &retIndex, &outDistSqr);
            map_[scan.observedVoxels[i]].closestDistance = outDistSqr;
            
            // Update voxel likelihoods.
            double b1, b2, b3;
            Eigen::Matrix3d B;
            Eigen::Vector3d alpha;

            b1 = 0;
            b2 = exp(-map_[scan.observedVoxels[i]].closestDistance*normDistOccDen_);
            b3 = 1 - exp(-map_[scan.observedVoxels[i]].closestDistance*normDistFreeDen_);
            B << b1, 0, 0,
                0, b2, 0,
                0, 0, b3;

            alpha = B * qcd.stateTransitionMatrix.transpose() 
                      * map_[scan.observedVoxels[i]].xHat;
            map_[scan.observedVoxels[i]].xHat = alpha/alpha.sum();
            map_[scan.observedVoxels[i]].scanLastSeen = scanNum;
            
            // Update the voxel state if the probability of being in that state is greater than the predefined threshold.
            // std::vector<double> vec(voxelMap.map_[pointsObservedUnique[i]].xHat.data(), voxelMap.map_[pointsObservedUnique[i]].xHat.data() + qcdParams.numStates);                
            // double maxElement = *std::max_element(&vec[0], &vec[0]+qcdParams.numStates);
            // int maxElementIndex = std::find(&vec[0], &vec[0]+qcdParams.numStates, maxElement) - &vec[0];

            int maxElementIndex = map_[scan.observedVoxels[i]].xHat(0) >
                                  map_[scan.observedVoxels[i]].xHat(1) ? 0 : 1;
            maxElementIndex = map_[scan.observedVoxels[i]].xHat(2) > 
                              map_[scan.observedVoxels[i]].xHat(maxElementIndex) ? 2 : maxElementIndex;
            double maxElement = map_[scan.observedVoxels[i]].xHat(maxElementIndex);

            if (maxElement > qcd.beliefThreshold)// && maxElement != voxelStates[voxelNum].currentState[0]
            { 
                // Check for a change in state.
                // Looking for a change in state from free to occupied and vice versa.
                if ((maxElementIndex != map_[scan.observedVoxels[i]].currentState) && 
                    (maxElementIndex != 0 && map_[scan.observedVoxels[i]].currentState != 0))
                {
                    map_[scan.observedVoxels[i]].lastStateChangeScan[0] = map_[scan.observedVoxels[i]].lastStateChangeScan[1]; 
                    map_[scan.observedVoxels[i]].lastStateChangeScan[1] = scanNum;
                }

                // Save the current state and the time it was observed.
                map_[scan.observedVoxels[i]].currentState = maxElementIndex;
                // voxelMap.map_[pointsObservedUnique[i]].currentStateScan = scanNum;
            }
            // Update the scan the last time the voxel was observed.
            map_[scan.observedVoxels[i]].currentStateScan = scanNum; 
        }
    });
    // Remove map points outside the maximum range and the frame sliding window.
    removeVoxelsOutsideWindowAndMaxRange();

    // if (scanNum_ == 4)
    // {
    //     for (auto &[vox,state] : map_)
    //         std::cout << vox(0) << " " << vox(1) << " "  << vox(2) << " " << state.scanLastSeen << std::endl;
    //     exit(1);
    // }

    delete ptCloudKdTree;
}

unsigned int Map::getMapSize()
{
    return map_.size();
}

void Map::removeVoxelsOutsideWindowAndMaxRange()
{
    std::vector<Eigen::Vector3i> ps;
    for (const auto &[pt, state] : map_)
    {
        double norm = (pow(pt(0)*voxelSize_-sensorPose(0,3),2) + 
                       pow(pt(1)*voxelSize_-sensorPose(1,3),2) + 
                       pow(pt(2)*voxelSize_-sensorPose(2,3),2));
        if (norm  > pow(maxRange_*2,2) || 
           (state.scanLastSeen > globalWinLen_ && 
            state.scanLastSeen < scanNum_- globalWinLen_))
        {
            ps.push_back(pt);
        }
    }
    
    for (auto x : ps)
        map_.erase(x);
}

void Map::findDynamicVoxels(Scan &scan, boost::circular_buffer<Scan> &scanHistory)
{
    // std::cout << scan.occupiedVoxels.size() << std::endl; exit(1);
    std::vector<double> tempScores(scan.occupiedVoxels.size(),0.0);
    tbb::parallel_for(
    tbb::blocked_range<int>(0, scan.occupiedVoxels.size()),
    [&](tbb::blocked_range<int> r)
    {
        for (unsigned int voxIndex = r.begin(); voxIndex < r.end(); voxIndex++)
        {
            // Extract the point indicies.
            int x = scan.occupiedVoxels[voxIndex](0);
            int y = scan.occupiedVoxels[voxIndex](1);
            int z = scan.occupiedVoxels[voxIndex](2);
            
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
            // if (scan.occupiedVoxels[voxIndex](0) == 5 && 
            //     scan.occupiedVoxels[voxIndex](1) == -9 &&
            //     scan.occupiedVoxels[voxIndex](2) == -1)
            // {
            //     // totalScore = 100;
            //     std::cout << x << " " << y << " "  << z << " " << edge_ << std::endl;
            //     std::cout << nPtsValid.size() << std::endl;
            //     // std::cout << totalScore << std::endl;
            //     exit(1);
            // }
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
            tempScores[voxIndex] = totalScore;
            // if (!voxHasUnobservedNeighbor)
                scan.setConvScore(scan.occupiedVoxels[voxIndex], totalScore);
                // prevScanOccupancy.map_[scan.occupiedVoxels[voxIndex]].convScore = totalScore;
        }
    });

    // for (auto x : scan.occupiedVoxels)
    // {
    //     std::cout << x(0) << " " << x(1) << " " << x(2) << std::endl;
    // }
    // exit(1);

    // for (auto x : tempScores)
    //     std::cout << x << std::endl;
    // exit(1);

    // scan.printVoxels(); exit(1);
    // for (auto &[vox,state] : map_)
    // {
    //     std::cout << vox(0) << " " << vox(1) << " " << vox(2) << " " << state. << std::endl;
    // }
    // exit(1);

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
    Eigen::VectorXd binCounts = Eigen::VectorXd::Zero(nBins_);
    Eigen::VectorXd edges;
    findHistogramCounts(nBins_, scoresOverWindow, binCounts, edges);
    // findHistogramCounts(nBins, scores, binCounts, edges);
    int level = otsu(binCounts);
    
    double thresholdLimit = 0;
    if (level > 0)
    {
        thresholdLimit = edges(level-1);
        // thresholdLimit = 15;
        for (unsigned int j = 0; j < scan.occupiedVoxels.size(); j++)
        {
            if (scan.getConvScoreOverWindow(scan.occupiedVoxels[j]) > thresholdLimit)
            {
                scan.setDynamic(scan.occupiedVoxels[j]);
                // scanDynamicVoxels_.push_back(scan.occupiedVoxels[j]);
            }
        }
    }

    // for (auto x : prevScanDynamicVoxels_)
    // {
    //     scan.setDynamic(x);
    // }

    for (auto x: prevScanDynamicVoxels_)
    {
        // if (prevScanOccupancy.CheckIfEntryExists(x))
        // if (prevScanOccupancy.CheckIfEntryExists(x) && prevScanOccupancy.map_[x].convScore > 3)
        if (scan.checkIfEntryExists(x) && scan.getConvScoreOverWindow(x) > 3)
        {
            scan.setDynamic(x);
        }
    }

    // Region growing.
    int dilSize = convSize_;
    Eigen::Vector3i p;

    // Perform a dilation.
    scanDynamicVoxels_.clear();
    dilSize = 3;
    for (auto &vox : scan.occupiedVoxels)
    {
        if (scan.getDynamic(vox))
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
            scan.setDynamic(x);
        }
    }

    // scanDynamicVoxels_.clear();
    // Save the high confidence dynamic detections to be used for the next scan.
    prevScanDynamicVoxels_.clear();
    for (auto x : scan.occupiedVoxels)
    {
        if (scan.getDynamic(x))
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