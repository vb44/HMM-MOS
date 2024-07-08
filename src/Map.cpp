#include "Map.hpp"

Map::Map(const ConfigParser &config)
    : globalWinLen_(config.globalWindowSize)
    , maxRange_(config.maxRange)
    , minRange_(config.minRange)
    , voxelSize_(config.voxelSize)
    , convSize_(config.convSize)
    , minOtsu_(config.minOtsu)
    , nBins_(100) // Hardcoded - used for histogram thresholding.
{
    hmmConfig.beliefThreshold = config.beliefThreshold;
    hmmConfig.sigFree = config.freeSigma;
    hmmConfig.sigOcc = config.occupancySigma;

    // Pre-compute the convolution edge size.
    edge_ = (convSize_-1)/2;

    // Pre-compute the convolution computation constants.
    normDistOccDen_ = 1/(2*pow(hmmConfig.sigOcc, 2));
    normDistFreeDen_ = 1/(2*pow(hmmConfig.sigFree, 2));
}

Map::~Map()
{

}

void Map::addVoxels(const std::vector<Eigen::Vector3i> &voxels)
{
    std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel)
    {
        if (map_.contains(voxel))
        {
            map_[voxel].currentStateScan = scanNum_;
        } else
        {
            MapVoxelState v;
            v.currentStateScan = scanNum_;
            map_.insert({voxel, v});
        }
    });
}

void Map::convertToKdTreeContainer(std::vector<Eigen::Vector3d> &pts)
{
    size_t pcLength = pts.size();
    pcForKdTree_.pts.resize(pcLength);

    tbb::parallel_for(
    tbb::blocked_range<int>(0, pcLength),
    [&](tbb::blocked_range<int> r)
    { 
        for (size_t i = r.begin(); i < r.end(); i++)
        {
            pcForKdTree_.pts[i].x = pts[i](0);
            pcForKdTree_.pts[i].y = pts[i](1);
            pcForKdTree_.pts[i].z = pts[i](2);        
        } 
    });
}

void Map::findDynamicVoxels(Scan &scan, boost::circular_buffer<Scan> &scanHistory)
{
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
                        totalScore = totalScore - 1;
                    }
                }
            }
            totalScore = std::max(totalScore,0.0);
            if (!voxHasUnobservedNeighbor)
                scan.setConvScore(voxel, totalScore);
        }
    });
    
    // Average the convolution scores using median filtering.
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
    int level = otsu(binCounts);    
    double thresholdLimit = 0;
    scan.dynThreshold = 0;
    if (level > 0)
    {
        thresholdLimit = edges(level-1);
        scan.dynThreshold = thresholdLimit;
        for (unsigned int j = 0; j < scan.occupiedVoxels.size(); j++)
        {
            if (scan.getConvScoreOverWindow(scan.occupiedVoxels[j]) > thresholdLimit)
            {
                scan.setDynamicHighConfidence(scan.occupiedVoxels[j]);
            }
        }
    }

    // Set the previously dynamic scans to be dynamic if they exist in the
    // current scan and still have some confidence of being dynamic.
    for (auto x: prevScanDynamicVoxels_)
    {
        if (scan.checkIfEntryExists(x) && scan.getConvScoreOverWindow(x) > minOtsu_)
        {
            scan.setDynamicHighConfidence(x);
        }
    }

    // Region growing.
    // Perform a nearest neighbour dilation.
    scanDynamicVoxels_.clear();
    int edge = 1; // Nearest neighbouring cells.
    for (auto &vox : scan.occupiedVoxels)
    {
        if (scan.getDynamicHighConfidence(vox))
        {
            int x = vox(0);
            int y = vox(1);
            int z = vox(2);
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

    // Save the high confidence dynamic detections to be used for the next scan.
    prevScanDynamicVoxels_.clear();
    for (auto x : scan.occupiedVoxels)
    {
        if (scan.getDynamicHighConfidence(x))
        {
            prevScanDynamicVoxels_.push_back(x);
        } 
    }    
}

void Map::findMedianValue(Scan &scan)
{
    // Clean the convolution scores.
    // Average the convolution scores around the neighbour.
    Scan scanOriginal = scan;
    for (auto &vox : scan.occupiedVoxels)
    {
        Voxel v;
        std::vector<double> voxScores;
        int x = vox(0);
        int y = vox(1);
        int z = vox(2);
        int edge = (3-1)/2; // nearest neighbour, so convolutoin size, m=3
        for (int i = x - edge; i < x + edge + 1; ++i)
        {
            for (int j = y - edge; j < y + edge + 1; ++j)
            {
                for (int k = z - edge; k < z + edge + 1; ++k)
                {
                    v << i, j, k;
                    if (scan.checkIfEntryExists(v))
                    {
                        voxScores.push_back(scan.getConvScore(v));
                    }
                }
            }
        }
        int val = findMedian(voxScores); 
        scanOriginal.setConvScore(vox, val);
    }

    for (auto &vox : scan.occupiedVoxels)
    {
        scan.setConvScore(vox, scanOriginal.getConvScore(vox));
    }
}

void Map::removeVoxelsOutsideWindowAndMaxRange()
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
           (state.scanLastSeen > globalWinLen_ && 
            state.scanLastSeen < scanNum_- globalWinLen_))
        {
            ps.push_back(pt);
        }
    }
    
    for (const auto &x : ps)
    {
        map_.erase(x);
    }
}

void Map::update(Scan &scan, unsigned int scanNum)
{
    scanNum_ = scanNum;
    sensorPose_ = scan.sensorPose;

    // Construct a KD-Tree of the measurements.
    convertToKdTreeContainer(scan.ptsOccupiedOverWindow);
    my_kd_tree_t *ptCloudKdTree = new my_kd_tree_t(3, pcForKdTree_, {10});

    // Update the observed voxels in the map.
    addVoxels(scan.observedVoxels);

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
            ptCloudKdTree->knnSearch(&pt[0], numResults, &retIndex, &map_[voxel].closestDistance);
            
            // Update voxel likelihoods.
            Eigen::Matrix3d B;
            B << 0, 0, 0,
                 0, exp(-map_[voxel].closestDistance*normDistOccDen_), 0,
                 0, 0, (1 - exp(-map_[voxel].closestDistance*normDistFreeDen_));

            Eigen::Vector3d alpha = B * hmmConfig.stateTransitionMatrix * map_[voxel].xHat;
            map_[voxel].xHat = alpha/alpha.sum();
            map_[voxel].scanLastSeen = scanNum;
            
            // Update the voxel state if the probability of being in that state is greater than the predefined threshold.
            int maxElementIndex = map_[voxel].xHat(0) > map_[voxel].xHat(1) ? 0 : 1;
            maxElementIndex = map_[voxel].xHat(2) > map_[voxel].xHat(maxElementIndex) ? 2 : maxElementIndex;
            double maxElement = map_[voxel].xHat(maxElementIndex);

            if (maxElement > hmmConfig.beliefThreshold)
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
            }
            // Update the scan the last time the voxel was observed.
            map_[voxel].currentStateScan = scanNum; 
        }
    });
    // Remove map points outside the maximum range and the frame sliding local window.
    removeVoxelsOutsideWindowAndMaxRange();

    delete ptCloudKdTree;
}