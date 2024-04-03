#pragma once

#ifndef SCAN_H
#define SCAN_H

#include <fstream>
#include <string>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <iomanip>

#include "ConfigParser.hpp"
#include "utils.hpp"

#include "ankerl/unordered_dense.h"
#include "eigen3/Eigen/Core"
#include <tbb/tbb.h>
#include <tbb/parallel_for.h>
#include "tsl/robin_map.h"

class Scan
{
    public: 
        using Voxel = Eigen::Vector3i;
        Scan(const ConfigParser &config); 

        ~Scan();

        // void removeVoxelsOutsideWindow(const unsigned int scanNum);        
        void removeVoxelsOutsideMaxRange();
        // void removeVoxelsOutsideWindowAndMaxRange();


        void readScan(const std::string &fileName, const std::vector<double> &pose);
        void voxelizeScan();
        void findObservedVoxels();

        bool checkIfEntryExists(const Voxel &voxel);
        void setDynamic(Voxel &voxel);
        void setDynamicHighConfidence(Voxel &voxel);
        void setConvScore(Voxel voxel, double convScore);
        void setConvScoreOverWindow(Voxel &voxel, double convScore);
        double getConvScore(Voxel &voxel);
        double getConvScoreOverWindow(Voxel &voxel);
        bool getDynamic(Voxel &voxel);
        bool getDynamicHighConfidence(Voxel &voxel);
        // double getTransitionProb(Voxel &voxel);
        std::vector<int> getIndicies(Voxel &voxel);
        void writeFile(std::ofstream &outFile, unsigned int scanNum);
        void writeLabel(unsigned int scanNum);
        void printVoxels();

        Eigen::Matrix4d sensorPose;

        // Unique set of the observed voxels used to upate the global map.        
        std::vector<Eigen::Vector3i> observedVoxels;
        std::vector<Eigen::Vector3i> occupiedVoxels;
        
        // The unquantized point cloud measurements used to construct the
        // scan's KD-Tree and then the Euclidean Distance Field.
        std::vector<Eigen::Vector3d> ptsOccupied;
        std::vector<Eigen::Vector3d> ptsOccupiedOverWindow;


        // int scanNum;
        double dynThreshold;


    protected:
        
        struct VoxelHash
        {
            size_t operator()(const Voxel &voxel) const {
                const uint32_t *vec = reinterpret_cast<const uint32_t *>
                                                            (voxel.data());
                return ((1 << 30) - 1) & (vec[0] * 73856093 ^
                                          vec[1] * 19349669 ^
                                          vec[2] * 83492791);
            }
        };

        double voxelSize_;
        double minRange_, maxRange_;

        void convertToPointCloudKdTree(std::vector<Eigen::Vector3d> &pts);

        // Container for a nanoflann-friendly point cloud.
        PointCloud<double> pcForKdTree_;

    private:
        struct ScanVoxelState
        {
            bool isDynamic = false;
            bool isDynamicHighConfidence = false;

            // Point cloud indicies in the current voxel.
            std::vector<unsigned int> pointIndicies;

            // Spatial 3D convolution score.
            double convScore = 0;

            // Spatio-temporal 4D convolution score.
            double convScoreOverWindow = 0; 
        };
        int dim_;
        std::string outputLabelFolder_;
        Eigen::MatrixXd scanPts_;
        std::vector<Eigen::Vector3d> scanPtsTf_;
        boost::circular_buffer<std::vector<Eigen::Vector3d> > ptsOccupiedHistory_;

        void addPointsWithIndex();
    
        // Voxelized scan.         
        tsl::robin_map<Voxel, ScanVoxelState, VoxelHash> scan_;
        // ankerl::unordered_dense::map<Voxel, ScanVoxelState, VoxelHash> scan_; 
};

#endif