# Moving Object Segmentation in Point Cloud Data using Hidden Markov Models

We propose a novel learning-free approach to segment moving objects in point cloud data.
The foundation of the approach lies in modelling each voxel using a Hidden Markov Model (HMM) and probabilistically integrating beliefs into a global map using an HMM filter.

This is the open-source implementation of the proposed approach. 
The code is portable and easy to understand and modify.

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
3. ***&sigma; (occupancy)*** is the voxel occupancy likelihood standard deviation
4. ***&sigma; (free)*** is the voxel occupancy likelihood standard deviation.
5. ***pMin*** is the state change detection threshold.
6. ***m*** is the convolution kernel size.
7. ***minOtsu*** is the minimum number of connected voxels to be identified as dynamic
8. ***wLocal*** is the local window size used for the spatiotemporal convolution
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
    * Data sourced from [here](https://github.com/PRBonn/MapMOS?tab=readme-ov-file). The dataset consists of multiple sequences recorded from a vehicle in an urban environment. The provided pose estimates are used.

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
git clone https://github.com/oneapi-src/oneTBB.git
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
git clone https://github.com/martinus/unordered_dense.git
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
* Install the *rapidcsv* library for evaluating the DOALS sequences.
```bash
git clone https://github.com/d99kris/rapidcsv.git
cd rapidcsv
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
MOS results can be saved in,
* a *single file* with point cloud indicies for each scan per row for evaluation with *DOALS* ground truth, or,
* *.label* files from Semantic KITTI for evaluation with *Sipailou Campus* and *Apollo* datasets.

Our results for the DOALS dataset for various voxel sizes and sensor ranges are available under *benchmarking/sampleResults/*, and label files for all tested sequences can be found downloaded from [here](https:://addlink!.com).

### Build evaluation tools
Go to the *benchmarking/evaluation* folder.
```bash
cd benchmarking/evaluation
```
Make and enter the build directory.
```bash
mkdir build && cd build
```
Run *cmake* and *make*.
```bash
cmake ../src
make
```

### DOALS Evaluation
The DOALS dataset provides 10 manually labelled scans per sequence for evaluation in *indicies.csv* files.
An evaluation tool is provided in *benchmarking/evaluation*.
Once the evaluation tool is built as explained in the instructions above, use the tool directly or the provided bash script in the *benchmarking/evaluation/scripts* folder.

Using the tool directly.
```bash
cd benchmarking/evaluation/build/
./evalIndFile gtFilePath estFilePath scanFolderPath sequenceNum minRange maxRange
```

Using the provided script.
```bash
cd benchmarking/evaluation/scripts/
./checkDOALS.sh # Edit the script parameters.
```

An example output of the DOALS Hauptgebaeude sequence 1 20m range evaluation is shown below.
```bash
|-------|-----------|-------|
|  IoU  | Precision | Recall|
|-------|-----------|-------|
| 83.65 |   96.99   | 85.88 | 
| 83.82 |   91.49   | 90.91 | 
| 96.87 |   99.45   | 97.39 | 
| 90.14 |   99.45   | 90.59 | 
| 83.09 |   99.51   | 83.43 | 
| 86.32 |   98.67   | 87.33 | 
| 95.40 |   99.35   | 96.00 | 
| 85.72 |   98.24   | 87.06 | 
| 84.12 |   97.42   | 86.04 | 
| 96.08 |   99.81   | 96.25 | 
|-------|-----------|-------|
Mean IOU: 88.52
```

### Sipailou Campus Evaluation
The Sipailou campus dataset is provided by [MotionBEV](https://github.com/xieKKKi/MotionBEV/).
MotionBEV provide evaluation tools to compute the *validation* (seq 06) and *test* (seq 00, 07) IoUs.
To provide a fair evalution with the reuslts published in the [MotionBEV paper](https://ieeexplore.ieee.org/document/10287575), we use their evaluation tools.
The evaluation steps are outlined below.

***It is important to ensure the HMM-config has a minimum range of 3m and a maximum range of 50m as the ground truth is only labelled within these ranges.***

1. Clone the MotionBEV repository.
```bash
git clone https://github.com/xieKKKi/MotionBEV.git
```
2. Copy the label predictions from HMM-MOS to the *sipailou-livox-kitti* directory sequence folder. The required file structure is:
```bash
# sipailou-livox-kitti directory tree
# The HMM-MOS labels are copied into the predictions folder.

sipailou-livox-kitti
└── sequences
    ├── 00
    │   ├── calib.txt
    │   ├── labels
    │   ├── poses.txt
    │   ├── predictions
    │   └── velodyne
    ├── 06
    │   ├── calib.txt
    │   ├── labels
    │   ├── poses.txt
    │   ├── predictions
    │   └── velodyne
    └── 07
        ├── calib.txt
        ├── labels
        ├── poses.txt
        ├── predictions
        └── velodyne
```
3. Run the validation evaluation.
```bash
cd MotionBEV
python3 utils/evaluate_mos.py --dataset /pathToFolder/sipailou-livox-kitti/ --datacfg config/livox-SEU-MOS.yaml -s valid
```
The expected output is shown below.
```bash
********************************************************************************
INTERFACE:
Data:  /media/vb-dell/Expansion/datasets/sipailou-livox-kitti/
Predictions:  /media/vb-dell/Expansion/datasets/sipailou-livox-kitti/
Backend:  numpy
Split:  valid
Config:  config/livox-SEU-MOS.yaml
Limit:  None
Codalab:  None
********************************************************************************
Opening data config file config/livox-SEU-MOS.yaml
[IOU EVAL] IGNORE:  []
[IOU EVAL] INCLUDE:  [0 1]
labels:  3191
predictions:  3191
Evaluating sequences: 10% 20% 30% 40% 50% 60% 70% 80% 90% ********************************************************************************
below can be copied straight for paper table
iou_moving: 0.852
```
3. Run the test evaluation.
```bash
cd MotionBEV
python3 utils/evaluate_mos.py --dataset /pathToFolder/sipailou-livox-kitti/ --datacfg config/livox-SEU-MOS.yaml -s test
```
The expected output is shown below.
```bash
********************************************************************************
INTERFACE:
Data:  /media/vb-dell/Expansion/datasets/sipailou-livox-kitti/
Predictions:  /media/vb-dell/Expansion/datasets/sipailou-livox-kitti/
Backend:  numpy
Split:  test
Config:  config/livox-SEU-MOS.yaml
Limit:  None
Codalab:  None
********************************************************************************
Opening data config file config/livox-SEU-MOS.yaml
[IOU EVAL] IGNORE:  []
[IOU EVAL] INCLUDE:  [0 1]
labels:  6201
predictions:  6201
Evaluating sequences: 10% 20% 30% 40% 50% 60% 70% 80% 90% ********************************************************************************
below can be copied straight for paper table
iou_moving: 0.862
```

### Apollo Evaluation
The post processed and labelled Apollo dataset can be downloaded from the opensource MapMOS github page who have kindly made the data available [here](https://github.com/PRBonn/MapMOS).
The *semantic-kitti-api* tools are used to evaluate the results.
The steps are outlined below.

1. Clone the semantic-kitti-api.
```bash
git clone https://github.com/PRBonn/semantic-kitti-api.git
```
2. Copy the label predictions from HMM-MOS to the LiDAR-MOS directory. The required file structure is:
```bash
LiDAR-MOS
├── dataset_description.yml
└── sequences
    ├── 00
    │   ├── calib.txt
    │   ├── labels
    │   ├── poses.txt
    │   ├── predictions
    │   ├── suma_poses.txt
    │   ├── velodyne
    │   └── velodyne_poses_kitti.txt
    ├── 03
        ├── calib.txt
        ├── labels
        ├── poses.txt
        ├── predictions
        ├── suma_poses.txt
        ├── velodyne
        └── velodyne_poses_kitti.txt
```
3. We want to evaluate sequence 00 and 03. Edit the *config/semantic-kitti-mos.yaml* file to replace *8* in the valid field to *0*.
```python
split: # sequence numbers
  train:
    - 0
    - 1
    - 2
    - 3
    - 4
    - 5
    - 6
    - 7
    - 9
    - 10
  valid:
    - 0 # was 8 originally
  test:
    - 11
    - 12
    - 13
    - 14
    - 15
    - 16
    - 17
    - 18
    - 19
    - 20
    - 21
```
4. Run the evaluation for sequence 00 and 03.
```bash
python3 evaluate_mos.py --dataset /pathToFolder/LiDAR-MOS/ -s valid
```
The expected output is shown below.
```bash
********************************************************************************
INTERFACE:
Data:  /pathToFolder/LiDAR-MOS/
Predictions:  /pathToFolder/LiDAR-MOS/
Backend:  numpy
Split:  valid
Config:  config/semantic-kitti-mos.yaml
Limit:  None
Codalab:  None
********************************************************************************
Opening data config file config/semantic-kitti-mos.yaml
Ignoring xentropy class  0  in IoU evaluation
[IOU EVAL] IGNORE:  [0]
[IOU EVAL] INCLUDE:  [1 2]
labels:  2000
predictions:  2000
Evaluating sequences: 10% 20% 30% 40% 50% 60% 70% 80% 90% ********************************************************************************
below can be copied straight for paper table
iou_moving: 0.573
```
Following a similar process, replace the valid sequence to 3.
The expected output is shown below.
```bash
********************************************************************************
INTERFACE:
Data:  /pathToFolder/LiDAR-MOS/
Predictions:  /pathToFolder/LiDAR-MOS/
Backend:  numpy
Split:  valid
Config:  config/semantic-kitti-mos.yaml
Limit:  None
Codalab:  None
********************************************************************************
Opening data config file config/semantic-kitti-mos.yaml
Ignoring xentropy class  0  in IoU evaluation
[IOU EVAL] IGNORE:  [0]
[IOU EVAL] INCLUDE:  [1 2]
labels:  500
predictions:  500
Evaluating sequences: 10% 20% 30% 40% 50% 60% 70% 80% 90% ********************************************************************************
below can be copied straight for paper table
iou_moving: 0.905
```


<!-- ### Label Evaluation
Example usage of the label evaluation tool is provided below.
```bash
cd benchmarking/evaluation/build/
./evalLabels gtFilePath estFilePath scanFolderPath minRange maxRange startScan
```
An example output of the Sipailou Campus sequence 07 evaluation between ranges 3-50m is shown below.
```bash
|-------|-----------|--------|
|  IoU  | Precision | Recall |
|-------|-----------|--------|
| 87.48 |   92.52   |  94.14 |
|-------|-----------|--------|
``` -->

## Results on Different Operating Systems
<!-- What has it been tested on? -->

## References
[1] [eigen library](https://eigen.tuxfamily.org/dox/GettingStarted.html)
