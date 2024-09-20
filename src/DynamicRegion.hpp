#pragma once

#ifndef HMMMOS_DYNAMICREGION_H_
#define HMMMOS_DYNAMICREGION_H_

#include "ConfigParser.hpp"
#include "Scan.hpp"
#include "utils.hpp"
#include "VoxelHash.hpp"

/**
 * @brief The DynamicRegion saves the high confidence dynamic detections
 *        over a receding window. 
 */
class DynamicRegion
{
    public:
        /**
         * @brief Construct a new DynamicRegion object.
         * 
         * @param config 
         */
        DynamicRegion(const ConfigParser &config);
        
        /**
         * @brief Destroy the Map object.
         * 
         */
        ~DynamicRegion();
        
        
        void update(Scan &scan, unsigned int scanNum);
        void writeMapToFile(long sampleTime, std::string &saveFolderPath);
        double staticConvThreshold = 0;

        /**
         * @brief  States of each voxel in the map.
         *         These are the attributes used to describe the voxel.
         * 
         */
        struct DynamicVoxelState
        { 
            // Dynamic state of the voxel.
            bool isDynamic = false;
            bool isDynamicHighConfidence = false;
            bool hasUnobservedNeighbour = false;
            int staticConvScore = 0;

            // The last time the voxel was seen.
            unsigned int scanLastSeen = 0;
        };
        
        // Voxelized map object.
        ankerl::unordered_dense::map<Voxel, DynamicVoxelState, VoxelHash> map_;

    private:
        int dynamicRegionWinLen_, nBins_, scanNum_;
        double maxRange_, minOtsu_, voxelSize_;
        Eigen::Matrix4d sensorPose_;
        
        /**
         * @brief Remove voxels outside the global moving window and the
         *        maximum range of the sensor.
         * 
         */
        void removeVoxelsOutsideWindowAndMaxRange();        
};

#endif // HMMMOS_DYNAMICREGION_H_