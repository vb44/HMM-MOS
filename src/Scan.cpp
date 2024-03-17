#include "Scan.hpp"

Scan::Scan(const ConfigParser &config)
{
    voxelSize_ = config.voxelSize;
    minRange_ = config.minRange;
    maxRange_ = config.maxRange;
    dim_ = config.maxRange/config.voxelSize * 2 + 1;

    ptsOccupiedHistory_.resize(config.localWindowSize);
}

Scan::~Scan()
{
}

void Scan::readScan(const std::string &fileName, const std::vector<double> &pose)
{
    // scanNum_ = scanNum;
    scanPtsTf_.clear();
    sensorPose << pose[0], pose[1], pose[2] , pose[3],
                  pose[4], pose[5], pose[6] , pose[7],
                  pose[8], pose[9], pose[10], pose[11],
                  0      , 0      , 0       , 1;      
    unsigned int numColumns = 4;
    std::ifstream file(fileName, std::ios::in | std::ios::binary);
    // if (!file) return EXIT_FAILURE;
    float item;
    std::vector<double> ptsFromFile;    // pts read from file
    
    while (file.read((char*)&item, sizeof(item)))
        ptsFromFile.push_back(item);

    unsigned int numPts = ptsFromFile.size() / numColumns;
    scanPts_.resize(numPts, numColumns);
    scanPtsTf_.resize(numPts);

    tbb::parallel_for(
    tbb::blocked_range<int>(0,numPts),
    [&](tbb::blocked_range<int> r)
    {
        for (unsigned int i = r.begin(); i < r.end(); i++)
        {
            int c = i*numColumns;
            scanPts_.row(i) << ptsFromFile[c], ptsFromFile[c+1], ptsFromFile[c+2], 1;
            scanPtsTf_[i] = ((sensorPose*scanPts_.row(i).transpose()).topRows(3));
        }
    });

    file.close();
}

void Scan::voxelizeScan()
{
    ptsOccupied.clear();
    occupiedVoxels.clear();
    observedVoxels.clear();
    clear();

    // Voxelize the scan.
    addPointsWithIndex();

    // Raycast to find all observed voxels.    
    findObservedVoxels();

    // Remove voxelized measurements that are out of range.
    removeVoxelsOutsideMaxRange(); 
}

void Scan::clear()
{
    scan_.clear();
}


void Scan::addPointsWithIndex()
{
    int i = 0;
    std::for_each(scanPtsTf_.cbegin(), scanPtsTf_.cend(), [&](const auto &point) {
        double ptNorm = (pow(point(0)-sensorPose(0,3),2) + 
                         pow(point(1)-sensorPose(1,3),2) + 
                         pow(point(2)-sensorPose(2,3),2));
        if (ptNorm > pow(minRange_,2))
        {
            auto voxel = Voxel((point / voxelSize_).template cast<int>());
            if (scan_.contains(voxel)) {
                scan_[voxel].pointIndicies.push_back(i);
            } else
            {
                ScanVoxelState v;
                v.pointIndicies.push_back(i);
                occupiedVoxels.push_back(voxel);
                scan_.insert({voxel, v});
                // ptsOcc.push_back({point(0),point(1),point(2)});
                ptsOccupied.push_back({voxel(0)*voxelSize_,
                                       voxel(1)*voxelSize_,
                                       voxel(2)*voxelSize_});
            }
        }
        i++;
    });


    ptsOccupiedOverWindow = ptsOccupied;
    unsigned int j = ptsOccupiedHistory_.size()-1;
    {
        // for (auto [vox,state] : currentScanOccupancyHistory[i].map_)
            // currentScanOccupancy.ptsOcc.push_back(ptCloudTfs[i][state.pointIndicies[0]]);
            // currentScanOccupancy.ptsOcc.push_back(ptCloudTf[state.pointIndicies[0]]);
        for (auto pt : ptsOccupiedHistory_[j])
        {
            ptsOccupiedOverWindow.push_back(pt);
        }
    }
    ptsOccupiedHistory_.push_back(ptsOccupied);
}

void Scan::findObservedVoxels()
{    
    std::vector<std::vector<std::vector<bool> > > obs(dim_, std::vector<std::vector<bool> >(dim_, std::vector<bool>(dim_)));

    // Save the occupied voxels. TODO: Make this an inbuilt function of the VoxleHashMap class.    
    // for (auto currentVoxelNum : scan_)
    // {
    //     // if (currentVoxelNum.second.pointIndicies.size() > 1)
    //         occupiedVoxelIndicies.push_back(currentVoxelNum.first);
    // }
    std::vector<std::vector<Eigen::Vector3i> > all(occupiedVoxels.size());
    
    // Find the free voxels traversed for each occupied voxel.
    int sX = std::floor(sensorPose(0,3)/voxelSize_);
    int sY = std::floor(sensorPose(1,3)/voxelSize_);
    int sZ = std::floor(sensorPose(2,3)/voxelSize_);

    tbb::parallel_for(
    tbb::blocked_range<int>(0,occupiedVoxels.size()),
    [&](tbb::blocked_range<int> r)
    {
        for (unsigned int occupiedVoxNum = r.begin(); occupiedVoxNum < r.end(); occupiedVoxNum++)
        { 
            // To support threading.
            std::vector<Eigen::Vector3i> ptsObs;

            // Extract the raycast start and end points.
            int x0 = std::floor(sensorPose(0,3)/voxelSize_);
            int y0 = std::floor(sensorPose(1,3)/voxelSize_);
            int z0 = std::floor(sensorPose(2,3)/voxelSize_);
            int x1 = occupiedVoxels[occupiedVoxNum](0);
            int y1 = occupiedVoxels[occupiedVoxNum](1);
            int z1 = occupiedVoxels[occupiedVoxNum](2);

            int sx = x0<x1 ? 1 : -1;
            int sy = y0<y1 ? 1 : -1;
            int sz = z0<z1 ? 1 : -1;
        
            int dx = abs(x1-x0);
            int dy = abs(y1-y0); 
            int dz = abs(z1-z0);
            int dm = std::max(std::max(dx,dy),dz); // maximum difference
            x1 = y1 = z1 = dm/2; // error offset

            
            Eigen::Vector3i p;
            for(int i = dm; i > -1; i--) 
            {
                double ptNorm = pow((sX-x0)*voxelSize_,2) + 
                                pow((sY-y0)*voxelSize_,2) + 
                                pow((sZ-z0)*voxelSize_,2);
                if (ptNorm <= pow(maxRange_,2))// && n >= pow(3,2))
                {
                    int voxelX = x0;
                    int voxelY = y0;
                    int voxelZ = z0;
                    p << voxelX, voxelY, voxelZ;
                    ptsObs.push_back(p);
                }
                // Update the next voxel to be traversed.
                x1 -= dx; if (x1 < 0) { x1 += dm; x0 += sx; } 
                y1 -= dy; if (y1 < 0) { y1 += dm; y0 += sy; } 
                z1 -= dz; if (z1 < 0) { z1 += dm; z0 += sz; } 
            }
            all[occupiedVoxNum] = ptsObs;
        }
    });

    int i,j,k;
    int offset = maxRange_/voxelSize_;
    // std::vector<Eigen::Vector3i> b;
    for (auto &x : all)
    {
        for (auto &y : x)
        {
            i = y(0)+offset-sX;
            j = y(1)+offset-sY;
            k = y(2)+offset-sZ;
            if (!obs[i][j][k])
            {
                obs[i][j][k] = true;
                observedVoxels.push_back(y);
            }
        }
    }

    // voxelMap.AddPoints(obsVoxUnique, scanNum);

    // for (const auto &x : voxelMap.map_)
    // {
    //     if (x.second.currentStateScan == scanNum)
    //         pointsObservedUnique.push_back(x.first);
    // }
}

void Scan::convertToPointCloudKdTree(std::vector<Eigen::Vector3d> &pts)
{
    size_t pcLength = pts.size();
    pcForKdTree_.pts.clear();
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

void Scan::removeVoxelsOutsideMaxRange()
{
    std::vector<Eigen::Vector3i> ps;
    for (const auto &[pt, state] : scan_)
    {
        double norm = (pow(pt(0)*voxelSize_-sensorPose(0,3),2) + 
                       pow(pt(1)*voxelSize_-sensorPose(1,3),2) + 
                       pow(pt(2)*voxelSize_-sensorPose(2,3),2));
        if (norm  > pow(maxRange_,2))
        {
            ps.push_back(pt);
        }
        
    }
    
    for (auto x : ps)
    {
        scan_.erase(x);
    }

    occupiedVoxels.clear();
    for (auto &[vox, state] : scan_)
    {
        occupiedVoxels.push_back(vox);
    }
}

bool Scan::checkIfEntryExists(const Voxel &voxel)
{
    return scan_.contains(voxel);
}

void Scan::setConvScore(Voxel &voxel, double convScore)
{
    scan_[voxel].convScore = convScore;
}

void Scan::setConvScoreOverWindow(Voxel &voxel, double convScore)
{
    scan_[voxel].convScoreOverWindow = convScore;
}

double Scan::getConvScore(Voxel &voxel)
{
    return scan_[voxel].convScore;
}

double Scan::getConvScoreOverWindow(Voxel &voxel)
{
    return scan_[voxel].convScoreOverWindow;
}

void Scan::setDynamic(Voxel &voxel)
{
    scan_[voxel].isDynamic = true;
}

bool Scan::getDynamic(Voxel &voxel)
{
    return scan_[voxel].isDynamic;
}

 std::vector<int> Scan::getIndicies(Voxel &voxel)
 {
    std::vector<int> inds;
    for (auto x : scan_[voxel].pointIndicies)
    {
        inds.push_back(x);
    }
    return inds;
 }

void Scan::writeFile(std::ofstream &outFile, unsigned int scanNum)
{
    std::stringstream outConstructor;
    std::string out;
    outConstructor << scanNum+1;
    for (auto &[vox, state] : scan_)
    {
        if (state.isDynamic)
        {
            for (auto pt : state.pointIndicies)
            {
                outConstructor << "," << pt;
            }
        }
    }
    outConstructor >> out;
    outFile << out;
    outFile << std::endl;
}

void Scan::printVoxels()
{
    for (auto &[vox,state] : scan_)
    {
        std::cout << vox(0) << " " << vox(1) << " " << vox(2)  << " " << state.convScore << std::endl;
    }
}