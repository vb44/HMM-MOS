#ifndef VoxelHash_h_included_
#define VoxelHash_h_included_

#include <Eigen/Dense>

using Voxel = Eigen::Vector3i;

// Voxel hash map implementation from KISS-ICP.
struct VoxelHash
{
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *> (voxel.data());
        return ((1 << 30) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};

#endif // VoxelHash_h_included