// This file is modeified from the nanoflann library: 
// https://github.com/jlblancoc/nanoflann/tree/master/examples.

/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2022 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "nanoflann.hpp"

template <typename T>
struct PointCloud
{
    struct Point
    {
        T x, y, z;
    };

    using coord_t = T;  //!< The type of each coordinate

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        else if (dim == 1)
            return pts[idx].y;
        else
            return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};

/**
 * @brief Kd tree type used directly from the nanoflann library.
 */
using my_kd_tree_t = const nanoflann::KDTreeSingleIndexAdaptor<
                     nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
                     PointCloud<double>, 3
                     >;

/**
 * @brief Read strings and convert to numbers to correctly order the input
 *        files. No two files should have the same name.
 * 
 * @param a         The first string to compare.
 * @param b         The second string to compare.
 * @return true     If file a's name is smaller than file b's name.
 * @return false    If file b's name is smaller than file a's name.
 */
bool compareStrings(std::string a, std::string b);

/**
 * @brief Construct a homogeneous (4x4) matrix.
 * 
 * @param roll              Roll angle in radians.
 * @param pitch             Pitch angle in radians.
 * @param yaw               Yaw angle in radians.
 * @param x                 X position in metres.
 * @param y                 Y position in metres.
 * @param z                 Z position in metres.
 * @return Eigen::Matrix4d  The homogeneous (4x4) matrix constructed. 
 */
Eigen::Matrix4d homogeneous(double roll, double pitch, double yaw, 
                            double x, double y, double z);

/**
 * @brief Read pose estimates in the sensor frame. The file must be in the
 *        KITTI format. 
 * 
 * @param fileName                             Name of the pose estimates file.
 * @return std::vector<std::vector<double> >   The pose estimates read from the
 *                                             file stored in a (n by 12) 
 *                                             matrix where there an n sensor
 *                                             poses.
 */
std::vector<std::vector<double> > readPoseEstimates(const std::string &fileName);

/**
 * @brief Convert the points to a nanoflann-friendly container.
 * 
 * @param pts The points to convert to a nanoflann-friendly container.
 */
void convertToPointCloud3D(PointCloud<double> *pc, std::vector<Eigen::Vector3d> &scan);

/**
 * @brief Find the median of the elements.
 * 
 * @param a         The set of values to find the median of.
 * @return double   The median of the values in vector "a".
 */
double findMedian(std::vector<double> &a);

/**
 * @brief Find the histogram counts for a set of values.
 * 
 * @param nBins     The number of bins to discrteize the values into.
 * @param vals      The values to bin.
 * @param binCounts The result of the binning process.    
 * @param edges     The bin edges.
 */
void findHistogramCounts(unsigned int nBins, std::vector<double> &vals,
                         Eigen::VectorXd &binCounts, Eigen::VectorXd &edges);

/**
 * @brief Perform an automatic Otsu thresholding thresholding on the histogram counts.
 * 
 * @param histogramCounts   The histogram counts to automatically threshold.
 * @return int              The automatic threshold corresponding to the bin number.
 */
int otsu(Eigen::VectorXd histogramCounts);