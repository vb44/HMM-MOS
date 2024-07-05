# Moving Object Segmentation in Point Cloud Data using Hidden Markov Models

We propose a novel learning-free approach to segment moving objects in point cloud data.
The foundation of the approach lies in modelling each voxel using a Hidden Markov Model (HMM) and probabilistically integrating beliefs into a global map using an HMM filter.

This is the open-source implementation of the proposed approach. 
The code is portable, easy to understand, and modify.

The following includes:
1. [A summary of the MOS approach](#method).
2. [Benchmark datasets](#benchmark-datasets)
3. [The hardware requirements and dependencies installation](#hardware-and-dependencies).
4. [Installation of the HMM-MOS repository](#installation).
5. [An example of using HMM-MOS](#example).
6. [Sample results interpretation](#sample-results-interpretation).
7. [Results on different operating systems and processors](#results-on-different-operating-systems).
8. [References](#references).

## Method
The method is illustrated in the flowchart below.

![Proposed approach](/media/processFlowchart.png)

The method has nine configuration parameters.
1. ***A*** is the HMM state transition matrix (fixed for the MOS task)
2. ***voxelSize*** is the discretization size of the voxel map
3. ***&sigma (occupancy)*** is the voxel occupancy likelihood standard deviation
4. ***&sigma (free)*** is the voxel occupancy likelihood standard deviation.
5. ***pMin*** is the state change detection threshold.
6. ***m*** is the convolution kernel size.
7. ***minOtsu*** is the minimum number of connected voxels to be identified as dynamic
8. ***wLocal*** is the local window size used for the spatio-temporal convolution
9. ***wGlobal*** is the global window size used for removing voxels from the map

There are also hardware settings:
* ***rMax*** is the maximum radius of the point cloud. This is hardware-dependent and not a configuration parameter.
* ***rMin*** is the minimum radius of the point cloud. This is hardware-dependent and not a configuration parameter.

## Benchmark Datasets
We test our algorithm using four open-source datasets. Click the links below to see the download instructions.
* [Urban Dynamic Objects LiDAR Dataset (DOALS)](https://projects.asl.ethz.ch/datasets/doku.php?id=doals)
    * Download instructions are also available on the [open-source Dynablox page](https://github.com/ethz-asl/dynablox?tab=readme-ov-file#Datasets). The dataset consists of eight sequences recorded with a handheld LiDAR in indoor and outdoor environments. The dataset is recorded in rosbags (see **note 1** below). We estimate the LiDAR pose using [SiMpLE](https://github.com/vb44/SiMpLE).
* [Dynablox](https://github.com/ethz-asl/dynablox?tab=readme-ov-file#Datasets)
    * This qualitative dataset was released by Dynablox. The datasets consist of eight sequences recorded with a handheld LiDAR, capturing the dynamic motion of various objects in complex environments. The dataset is recorded in rosbags (see **note 1** below). We estimate the LiDAR pose using [SiMpLE](https://github.com/vb44/SiMpLE).
* [Sipailou Campus](https://github.com/xieKKKi/MotionBEV)
    * The dataset consists of eight sequences using a Livox Avia mounted to a mobile robot. The sequences are available in the same format as Semantic-KITTI using *.bin* files and corresponding ground truth in *.label* files. The provided sensor pose estimates are used.
* [Apollo Dataset](https://www.ipb.uni-bonn.de/html/projects/apollo_dataset/LiDAR-MOS.zip)
    * Data sourced from [here](https://github.com/PRBonn/4DMOS/issues/29). The dataset consists of multiple sequences recorded from a vehicle in an urban environment. The provided pose estimates are used.

**Note 1**: The HMM-MOS implementation provided here is designed to work with scan files in the *.bin* KITTI format with the sensor pose of each scan provided in a poses.txt file which is also in the KITTI format (nx12). To test the DOALS and Dynablox datasets, we converted each rosbag to a sequence of deskewed *.bin* scan files and estimated the sensor pose using SiMpLE to provide accurate odometry.

## Hardware and Dependencies
This implementation has been tested on Ubuntu 20.04.5/6 LTS (Focal Fossa) with an Intel Core i7-10700K CPU @ 3.80GHz x 16 and 62.5 GiB memory.
HMM-MOS uses a few open-source libraries for reading the algorithm configuration file, Kd-Trees, hash maps, matrix operations, and CPU threading.
The installation instructions are detailed below.

* Git is required to download the open-source libraries.
```bash
sudo apt install git
```
* Require a g++ compiler.
```bash
sudo apt install g++
```
* CMake is required to compile the libraries and the repository.
```bash
sudo apt install cmake
```
* Install the Eigen library for math operations [1].
```bash
sudo apt install libeigen3-dev
```
* Install Intel's Thread Building Blocks (TBB) library for CPU threading.
```bash
sudo apt install libtbb-dev
```
If the HMM-MOS repository build in the following section returns an error that it cannot find TBB for CMake, the following installation may help.
```bash
git clone https://github.com/oneapi-src/oneTBB
cd oneTBB
mkdir build && cd build
cmake ..
sudo make install
```
* Clone and install the *nanoflann* library for KD-tree operations.
```bash
git clone https://github.com/jlblancoc/nanoflann.git
cd nanoflann
mkdir build && cd build
cmake ..
sudo make install
```
* Install the *unordered dense* library for the hashmap implementation.
```bash
git clone https://github.com/martinus/unordered_dense
cd unordered_dense
mkdir build && cd build
cmake ..
sudo make install
```
* Install the *yaml-cpp* library for reading the configuration file.
```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake ..
sudo make install
```

Alternative options for any of the libraries can be used if desired.
The code is easy to change.

## Installation
Clone the repository.
```bash
git clone https://github.com/vb44/HMM-MOS.git
```

Create a build folder in the repository.
```bash
cd HMM-MOS
mkdir build && cd build
```

Run CMake.
```bash
cmake ../src
```

Make the executable.
```bash
make
```

## Example
The code only works with *.bin* files in the KITTI format.
However, the code is very easy to modify to suit the desired inputs and outputs.
When compiled, the HMM-MOS algorithm is run using a *.yaml* algorithm configuration file as shown below.
```bash
./hmmMOS config.yaml
```
Sample config files are included in the *config* folder.
An example is shown below.
```yaml
---
# File paths
scanPath: /pathToScans  # Must be .bin scan files.
posePath: /pathToPoses  # Must be in the KITTI format.
startScan: 1            # scan number [#]
endScan: 1079           # scan number [#]
minRange: 0.5           # [m]
maxRange: 120           # [m]
outputFile: true        # true/false
scanNumsToPrint: [969, 1079] # Leave as [] if not being used. Scans indexed from 1.
outputFileName: /outputFilePathAndName
outputLabels: true      # true/false
outputLabelFolder: /outputLabelFolderPath

# Configuration parameters
voxelSize: 0.5          # [m]
occupancySigma: 0.2     # [m]
freeSigma: 0.2          # [m]
beliefThreshold: 0.99   # probability [0-1]
convSize: 5             # odd integer
localWindowSize: 3      # local window size [#]
minOtsu: 3              # min dynamic voxels in convolution
globalWindowSize: 300   # global window size [#]
```   

## Sample Results Interpretation
<!-- How to interpret the output files and label files? -->
<!-- Benchmark results interpretation -->
<!-- Evaluation scripts -->
<!-- Provide the ground truth and scans -->
MOS results can be saved in (1) a single file with point cloud indicies for each scan per row for evaluation with *DOALS* ground truth, or (2) .label files from Semantic KITTI for evaluation with *Sipailou Campus* and *Apollo* datasets.

Our results for the DOALS dataset for various voxel sizes and sensor ranges are available under *benchmarking/sampleResults/*.

## Results on Different Operating Systems
<!-- What has it been tested on? -->

## References
[1] [eigen library](https://eigen.tuxfamily.org/dox/GettingStarted.html)