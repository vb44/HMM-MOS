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

        /**
         * @brief Check if voxel exists in the dynamic occupancy region.
         * 
         * @param voxel The voxel to check. 
         * @return true     If the voxel exists in the dynamic occupancy
         *                  region.
         * @return false    If the voxel does not exist in the dyanamic
         *                  occupancy region.
         */
        bool checkIfEntryExists(Voxel &voxel);

        /**
         * @brief Check if the voxel is a high confidence dynamic detection
         *        in the dynamic occupancy region.
         * 
         * @param voxel     The voxel to check the dynamic confidence of.
         * @return true     If the voxel is dynamic with high confidence.
         * @return false    If the voxel is not dynamic with high confidence.
         */
        bool isHighConfidence(Voxel &voxel);
        
        /**
         * @brief Update the dynamic occupancy region with the latest scan. 
         * 
         * @param scan      The scan used to update the dynamic occupancy region. 
         * @param scanNum   The current scan number.
         */
        void update(Scan &scan, unsigned int scanNum);
        
        /**
         * @brief Write the current dynamic occupancy region to file.
         *        This includes the static and dynamic voxels.
         * 
         * @param sampleTime        The timestamp of the map file
         *                          (usually the scan number).
         * @param saveFolderPath    The save location.
         */
        void writeMapToFile(long sampleTime, std::string &saveFolderPath);
        
        /**
         * @brief The automatic Otsu threshold on the convolution scores. 
         * 
         */
        double staticConvThreshold = 0;

    private:
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