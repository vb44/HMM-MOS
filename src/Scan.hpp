#pragma once

#ifndef SCAN_H
#define SCAN_H

#include <boost/circular_buffer.hpp>
#include <fstream>
#include <iomanip>
#include <string>
#include <tbb/parallel_for.h>
#include <vector>

#include "ankerl/unordered_dense.h"
#include "tsl/robin_map.h"

#include "ConfigParser.hpp"
#include "nanoflannUtils.hpp"
#include "utils.hpp"
#include "VoxelHash.hpp"
class Scan
{
    public: 
        Scan(const ConfigParser &config); 
        ~Scan();
   
        void removeVoxelsOutsideMaxRange();

        void readScan(const std::string &fileName,
                      const std::vector<double> &pose);
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

        // Unique set of the occupied and observed voxels used to upate the global map.        
        std::vector<Eigen::Vector3i> observedVoxels;
        std::vector<Eigen::Vector3i> occupiedVoxels;
        
        // The unquantized point cloud measurements used to construct the
        // scan's KD-Tree and then the Euclidean Distance Field.
        std::vector<Eigen::Vector3d> ptsOccupied;
        std::vector<Eigen::Vector3d> ptsOccupiedOverWindow;

        // int scanNum;
        double dynThreshold;

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


        double voxelSize_;
        double minRange_, maxRange_;

        int dim_;
        double minOtsu_;
        std::string outputLabelFolder_;
        Eigen::MatrixXd scanPts_;
        std::vector<Eigen::Vector3d> scanPtsTf_;
        boost::circular_buffer<std::vector<Eigen::Vector3d> > ptsOccupiedHistory_;

        // Voxelized scan.         
        tsl::robin_map<Voxel, ScanVoxelState, VoxelHash> scan_;
        // ankerl::unordered_dense::map<Voxel, ScanVoxelState, VoxelHash> scan_; 

        void addPointsWithIndex();
};

#endif