#pragma once

#ifndef MAP_H
#define MAP_H

#include "ConfigParser.hpp"
#include "VoxelHash.hpp"
#include "Scan.hpp"
#include "nanoflannUtils.hpp"
#include "utils.hpp"

#include <boost/circular_buffer.hpp>
class Map
{
    public:
        struct HmmConfig
        {
            unsigned int numStates;
            Eigen::MatrixXd stateTransitionMatrix;
            double sigOcc;
            double sigFree;
            double beliefThreshold;
        };

        Map(const ConfigParser &config);
        ~Map();

        void update(Scan &scan, unsigned int scanNum);
        void findDynamicVoxels(Scan &scan, boost::circular_buffer<Scan> &scanHistory);
        void findMedianValue(Scan &scan);
        unsigned int getMapSize();

        HmmConfig hmmConfig;

    private:
        struct MapVoxelState
        {
            unsigned int scanLastSeen = 0;

            // Dynamic state of the voxel.
            // True is dynamic, false if static.
            bool isDynamic = false;

            // The current state of the voxel,
            // (0,1,2) = (unobserved, occupied, free).
            unsigned int currentState = 0;

            // The scan the current state was updated.
            unsigned int currentStateScan = 0;

            // The scan of the last change in state.
            unsigned int lastStateChangeScan[2] = {0,0};

            // Used for Gaussian Distance Field (GDF) construction.
            double closestDistance = 0;

            // The voxel's state vector, initialized in the unobserved state.
            Eigen::Vector3d xHat = {1,0,0};

        };

        using pointsIterator = std::vector<Eigen::Vector3i>::const_iterator;

        int globalWinLen_;
        double normDistOccDen_, normDistFreeDen_;
        double maxRange_, minRange_;
        double voxelSize_;
        double minOtsu_;
        int scanNum_;
        int convSize_;
        int edge_;
        int nBins_;
        Eigen::Matrix4d sensorPose_;
        // Container for a nanoflann-friendly point cloud.
        NanoflannPointsContainer<double> pcForKdTree_;
        tsl::robin_map<Voxel, MapVoxelState, VoxelHash> map_;
        // ankerl::unordered_dense::map<Voxel, MapVoxelState, VoxelHash> map_; 

        std::vector<Voxel> scanDynamicVoxels_;
        std::vector<Voxel> prevScanDynamicVoxels_;
        std::vector<Voxel> scanHcDynamicVoxels_;
        std::vector<Voxel> prevScanHcDynamicVoxels_;

        void addPoints(const std::vector<Voxel> &points);
        void removeVoxelsOutsideWindowAndMaxRange();
        void convertToKdTreeContainer(std::vector<Eigen::Vector3d> &pts);
};

#endif