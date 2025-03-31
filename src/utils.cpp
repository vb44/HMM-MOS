#include "utils.hpp"

namespace utils
{
    bool compareStrings(std::string a, std::string b)
    {
        std::string delimiterStart = "/";
        std::string delimiterEnd = ".bin";
        
        std::string aNum = a.substr(a.find_last_of(delimiterStart)+delimiterStart.size(), a.size());
        std::string bNum = b.substr(b.find_last_of(delimiterStart)+delimiterStart.size(), b.size());
        aNum = aNum.substr(0, aNum.find(delimiterEnd));
        bNum = bNum.substr(0, bNum.find(delimiterEnd));

        return stol(aNum) < stol(bNum);
    }

    void findHistogramCounts(unsigned int nBins, std::vector<double> &vals,
                             Eigen::VectorXd &binCounts, Eigen::VectorXd &edges)
    {
        for (int i = 0; i < vals.size(); i++)
        {
            if (vals[i] > 0)
            {
                vals[i] = std::log(vals[i]);
            }
        }
        double maxVal = *std::max_element(&vals[0], &vals[0]+vals.size());
        double minVal = *std::min_element(&vals[0], &vals[0]+vals.size());
        double binSize = (maxVal - minVal)/nBins;
        edges.resize(nBins+1);
        edges = Eigen::VectorXd::LinSpaced(nBins+1, minVal, maxVal);

        for (size_t i = 0; i < vals.size(); i++)
        {
            double valFromLeft = vals[i] - minVal;
            double valFromRight = maxVal - vals[i];
            int ind;
            if (valFromLeft < 1e-3)
            {
                ind = 0;
            } else if (valFromRight < 1e-3)
            {
                ind = nBins - 1;
            } else
            {
                ind = ceil((vals[i] - minVal) / binSize) - 1;
            }
            binCounts(ind) = binCounts(ind) + 1;
        }
    }

    double findMedian(std::vector<double> &a) 
    {
        int n  = a.size();
        std::sort(a.begin(), a.end());
        if (n  % 2 != 0)
        {
            return a[n/2];
        } else
        {
            return (a[(n-1)/2] + a[(n)/2])/2;
        }
    }

    Eigen::Matrix4d homogeneous(double roll, double pitch, double yaw, 
                                double x, double y, double z)
    {
        Eigen::Matrix4d T;
        T.setZero();

        T(0,0) = cos(yaw) * cos(pitch);
        T(0,1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
        T(0,2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
        T(0,3) = x;
        T(1,0) = sin(yaw) * cos(pitch);
        T(1,1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
        T(1,2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
        T(1,3) = y;
        T(2,0) = -sin(pitch);
        T(2,1) = cos(pitch) * sin(roll);
        T(2,2) = cos(pitch) * cos(roll);
        T(2,3) = z;
        T(3,3) = 1;

        return T;
    }

    int otsu(Eigen::VectorXd histogramCounts)
    {
        int level = -1; // default value
        double total = histogramCounts.sum();
        int top = histogramCounts.rows();
        double sumB = 0;
        double wB = 0;
        double maximum = 0;
        double sum1 = histogramCounts.dot(Eigen::VectorXd::LinSpaced(histogramCounts.rows(), 0, top-1));

        for (int ii = 0; ii < top; ii++)
        {
            double wF = total - wB;
            if (wB > 0 && wF > 0)
            {
                double mF = (sum1 - sumB) / wF;
                double val = wB * wF * ((sumB / wB) - mF) * ((sumB / wB) - mF);
                if (val >= maximum)
                {
                    level = ii + 1;
                    maximum = val;
                }
            }
            wB = wB + histogramCounts(ii);
            sumB = sumB + (ii) * histogramCounts(ii);
        }
        return level;
    }

    std::vector<std::vector<double> > readPoseEstimates(const std::string &fileName)
    {
        // The pose estimates file must be in the KITTI format.
        unsigned int numColumns = 12;
        std::fstream file(fileName, std::ios_base::in);
        // if (!file) return EXIT_FAILURE;

        std::vector<std::vector<double> > poses;
        std::vector<double> pose(numColumns, 0.0);
        std::vector<double> dataFromFile;
        float pt;
        
        while (file >> pt)
        {
            dataFromFile.push_back(pt);
        }

        for (size_t i = 0; i < dataFromFile.size(); i+=numColumns)
        {
            for (unsigned int j = 0; j < numColumns; j++)
            {
                pose[j] = dataFromFile[i+j];
            }
            
            poses.push_back(pose);
        }

        return poses;
    }
}