#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <sstream>

// Function prototypes.
void readScan(const std::string &fileName, double minRange, double maxRange,
              std::vector<std::vector<double> > &pc, std::vector<bool> &usePoint);
bool compareStrings(std::string a, std::string b);
void readLabel(const std::string &fileName, std::vector<int> &label);


int main(int argc, char** argv)
{
    if (argc != 6)
    {
        std::cerr << "Usage: ./eval gtFilePath estFilePath scanFolderPath minRange maxRange" << std::endl;
        return 1;
    }

    std::string gtFilePath = argv[1];
    std::string estFilePath = argv[2];
    std::string scanFolderPath = argv[3];
    double minRange = std::stod(argv[4]);
    double maxRange = std::stod(argv[5]);
    
    // std::string gtFilePath = "/media/vb/Gift/sipailou-livox-kitti/07/labels/";
    // std::string estFilePath = "/home/vb/Documents/public_repositories/QCD_MOS/results/labels/sipailou_07";
    // std::string scanFolderPath = "/media/vb/Gift/sipailou-livox-kitti/07/velodyne/";
    // double maxRange = 50;
    // double minRange = 3;

    // Read the scans, estimate and ground truth label file names.
    std::vector<std::string> scanFiles;
    std::vector<std::string> gtLabelFiles;
    std::vector<std::string> estLabelFiles;

    for (auto const& dir_entry : std::filesystem::directory_iterator(scanFolderPath))
    { 
        scanFiles.push_back(dir_entry.path());
    }
    std::sort(scanFiles.begin(), scanFiles.end(), compareStrings);

    for (auto const& dir_entry : std::filesystem::directory_iterator(estFilePath))
    { 
        estLabelFiles.push_back(dir_entry.path());
    }
    std::sort(estLabelFiles.begin(), estLabelFiles.end(), compareStrings);

    for (auto const& dir_entry : std::filesystem::directory_iterator(gtFilePath))
    { 
        gtLabelFiles.push_back(dir_entry.path());
    }
    std::sort(gtLabelFiles.begin(), gtLabelFiles.end(), compareStrings);
    
    int numScans = scanFiles.size();
    int numEstLabels = estLabelFiles.size();
    int numGtLabels = gtLabelFiles.size();

    if (numScans != numEstLabels && numScans != numGtLabels)
    {
        std::cerr << "./evalLabels The number of scans, estimate labels, and ground truth labels are not the same." << std::endl;
        return 1;
    }

    std::vector<int> gtLabel;
    std::vector<int> estLabel;
    std::vector<std::vector<double> > pc;
    std::vector<bool> usePt;
    int tp = 0;
    int fp = 0;
    int fn = 0;
    for (int i = 0; i < scanFiles.size(); i++)
    {
        gtLabel.clear();
        estLabel.clear();
        pc.clear();
        usePt.clear();

        readLabel(gtLabelFiles[i],gtLabel);
        readLabel(estLabelFiles[i],estLabel);
        readScan(scanFiles[i],minRange,maxRange,pc,usePt);

        for (int j = 0; j < pc.size(); j++)
        {
            if (usePt[j])
            {
                if (gtLabel[j] > 250 && estLabel[j] > 250)
                {
                    tp++;
                } else if (gtLabel[j] <= 250 && estLabel[j] > 250)
                {
                    fp++;   
                } else if (gtLabel[j] > 250 && estLabel[j] <= 250)
                {
                    fn++;
                }
            }   
        }
    }

    // Print the results
    double iou = double(tp)/double(tp+fn+fp) * 100;
    double precision = double(tp)/double(tp+fp) * 100;
    double recall = double(tp)/double(tp+fn) * 100;
    std::cout << "|-------|-----------|--------|" << std::endl;
    std::cout << "|  IoU  | Precision | Recall |" << std::endl;
    std::cout << "|-------|-----------|--------|" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "| " << iou << " |   " << precision << "   |  " << recall << " |" << std::endl;
    std::cout << "|-------|-----------|--------|" << std::endl;

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

void readLabel(const std::string &fileName, std::vector<int> &label)
{
    std::ifstream file(fileName, std::ios::in | std::ios::binary);   
    if (!file)
    {
        std::cerr << "./evalLabels: Scan file " + fileName + 
                     " could not be opened! Exiting program." << std::endl;
        exit(1);
    }

    uint32_t item;
    while (file.read((char*)&item, sizeof(item)))
    {
        label.push_back(item);
    }
    file.close();
}