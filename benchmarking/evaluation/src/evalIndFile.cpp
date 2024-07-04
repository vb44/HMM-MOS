#include <filesystem>
#include <iostream>
#include <unordered_map>
#include <string>
#include <vector>
#include <sstream>

#include "rapidcsv.h"

// Function prototypes.
void readScan(const std::string &fileName, double minRange, double maxRange,
              std::vector<std::vector<double> > &pc, std::vector<bool> &usePoint);
bool compareStrings(std::string a, std::string b);


std::vector<std::vector<int> > evalScans = {{89, 199, 309, 419, 529, 639, 749, 859, 969, 1079},
                                            {129, 279, 429, 579, 729, 879, 1029, 1179, 1329, 1479},
                                            {76, 256, 436, 616, 796, 976, 1156, 1336, 1516, 1696},
                                            {139, 299, 459, 619, 779, 939, 1099, 1259, 1419, 1579},
                                            {79, 179, 279, 379, 479, 579, 679, 779, 879, 979},
                                            {199, 399, 527, 599, 799, 999, 1199, 1399, 1599, 1799},
                                            {37, 149, 349, 549, 749, 949, 1149, 1349, 1549, 1749},
                                            {149, 319, 489, 659, 829, 999, 1169, 1339, 1509, 1679}};

int main(int argc, char** argv)
{
    if (argc != 7)
    {
        std::cerr << "Usage: ./eval gtFilePath estFilePath scanFolderPath seq minRange maxRange" << std::endl;
        return 1;
    }

    std::string gtFilePath = argv[1];
    std::string estFilePath = argv[2];
    std::string scanFolderPath = argv[3];
    int seq = std::stoi(argv[4])-1;
    double minRange = std::stod(argv[5]);
    double maxRange = std::stod(argv[6]);

    // Read the scans file names.
    std::vector<std::string> scanFiles;
    for (auto const& dir_entry : std::filesystem::directory_iterator(scanFolderPath))
    { 
        scanFiles.push_back(dir_entry.path());
    }
    std::sort(scanFiles.begin(), scanFiles.end(), compareStrings);
    
    // Read the ground truth and estimate files.
    rapidcsv::Document gtFile(gtFilePath, rapidcsv::LabelParams(-1,0));
    rapidcsv::Document estFile(estFilePath, rapidcsv::LabelParams(-1,0));

    // Setup up evaluation containers.
    std::unordered_map<int,int> gtInds;
    std::unordered_map<int,int> estInds;
    std::unordered_map<int,int> allInds;

    int numRows = 10;
    std::vector<int> row;
    std::vector<std::vector<double> > results;
    std::vector<std::vector<double> > pc;
    std::vector<bool> usePt;
    double iouSum = 0;

    for (int rowNum = 0; rowNum < numRows; rowNum++)
    {
        // Read the scans.
        pc.clear();
        usePt.clear();
        readScan(scanFiles[evalScans[seq][rowNum]-1],minRange,maxRange,pc,usePt);

        gtInds.clear();
        estInds.clear();
        allInds.clear();

        // Read the ground truth indicies.
        row.clear();
        row = gtFile.GetRow<int>(rowNum);
        for (int i = 0; i < row.size(); i++)
        {
            int ind = row[i];
            if (usePt[ind]) // Only add the index to the ground truth if in range.
            {
                gtInds[ind] = 1;
                allInds[ind] = 1;
            }
        }

        // Read the estimate indicies.
        row.clear();
        row = estFile.GetRow<int>(rowNum);
        for (auto ind : row)
        {
            estInds[ind] = 1;
            allInds[ind] = 1;
        }

        // std::cout << gtInds.size() << " " << estInds.size() << std::endl;

        int tp = 0;
        int fp = 0;
        int fn = 0;
        for (auto &it : allInds)
        {
            int ind = it.first;
            bool inGt = gtInds.contains(ind);
            bool inEst = estInds.contains(ind);

            if (inGt && inEst)
            {
                tp++;
            } else if (inGt && !inEst)
            {
                fn++;
            } else if (!inGt && inEst)
            {
                fp++;
            }
        }
        double iou = double(tp)/double(tp+fn+fp) * 100;
        double precision = double(tp)/double(tp+fp) * 100;
        double recall = double(tp)/double(tp+fn) * 100;
        iouSum += iou;
        results.push_back({iou,precision,recall});
    }

    // Print the result.
    // std::cout << std::fixed << std::setprecision(2) 
    //           << "|-------|-----------|-------|" << std::endl
    //           << "|  IoU  | Precision | Recall|" << std::endl
    //           << "|-------|-----------|-------|" << std::endl;
    // for (auto res : results)
    // {
    //     std::cout << std::fixed << std::setprecision(2) 
    //               << "| " << res[0] << " |   " << res[1] << "   | " << res[2] << " | " << std::endl; 
    // }
    // std::cout << "|-------|-----------|-------|" << std::endl;

    // std::cout << "Mean IOU: " << iouSum/10.0 << std::endl;
    std::cout << iouSum/10.0 << std::endl;

    return 0;
}

bool compareStrings(std::string a, std::string b)
{
    std::string delimiterStart = "/";
    std::string delimiterEnd = ".bin";
    
    std::string aNum = a.substr(a.find_last_of(delimiterStart) +
                                               delimiterStart.size(),
                                               a.size());
    std::string bNum = b.substr(b.find_last_of(delimiterStart) +
                                               delimiterStart.size(),
                                               b.size());
    aNum = aNum.substr(0, aNum.find(delimiterEnd));
    bNum = bNum.substr(0, bNum.find(delimiterEnd));

    return stol(aNum) < stol(bNum);
}

void readScan(const std::string &fileName, double minRange, double maxRange,
              std::vector<std::vector<double> > &pc, std::vector<bool> &usePoint)
{
    unsigned int numColumns = 4; // Files expected in the .bin KITTI format.
    
    // Read the .bin scan file.
    std::ifstream file(fileName, std::ios::in | std::ios::binary);   
    if (!file)
    {
        std::cerr << "./eval: Scan file " + fileName + 
                     " could not be opened! Exiting program." << std::endl;
        exit(1);
    }

    float item;
    std::vector<double> ptsFromFile;
    
    while (file.read((char*)&item, sizeof(item)))
    {
        ptsFromFile.push_back(item);
    }

    unsigned int numPts = ptsFromFile.size() / numColumns;

    for (unsigned int i = 0; i < numPts; i++)
    {
        int c = i*numColumns;
        double x = ptsFromFile[c];
        double y = ptsFromFile[c+1];
        double z = ptsFromFile[c+2];
        double norm = std::sqrt(pow(x,2)+pow(y,2)+pow(z,2));
        bool ptInRange = (norm >= minRange && norm <= maxRange);
        pc.push_back({x,y,z});
        usePoint.push_back(ptInRange);
    }

    file.close();
}