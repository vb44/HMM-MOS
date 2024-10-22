# Moving Object Segmentation in Point Cloud Data using Hidden Markov Models
We propose a novel learning-free approach to segment moving objects in point cloud data.
The foundation of the approach lies in modelling each voxel using a Hidden Markov Model (HMM) and probabilistically integrating beliefs into a global map using an HMM filter.
We extend classic image processing to accurately detect dynamic objects in point cloud data.

The workshop paper ([found here!](https://github.com/vb44/HMM-MOS/blob/main/paper/Moving_Object_Segmentation_in_Point_Cloud_Data_using_Hidden_Markov_Models.pdf)) was accepted and presented to the IEEE IROS 2024 workshop on Long-Term Perception for Autonomy in Dynamic Human-shared Environments: What Do Robots Need?

[Click here for a demo!](https://github.com/user-attachments/assets/8ac52292-4b8b-4f97-a4ef-c546965338b5)

This is the open-source implementation of the proposed approach. 
The code is portable and easy to understand and modify.

The following includes:
1. [A summary of the MOS approach](#method)
2. [Benchmark datasets](#benchmark-datasets)
3. [The hardware requirements and dependencies installation](#hardware-and-dependencies)
4. [Installation of the HMM-MOS repository](#installation)
5. [An example of using HMM-MOS](#example)
6. [Sample results interpretation](#sample-results-interpretation)
7. [Results on different operating systems and processors](#results-on-different-operating-systems)
8. [References](#references)

## Method
<!-- The method is illustrated in the flowchart below.

![Proposed approach](/media/processFlowchart.png) -->

The method has ten configuration parameters.
1. ***A*** is the HMM state transition matrix (fixed for the MOS task)
2. ***voxelSize*** is the discretization size of the voxel map
3. ***&sigma; (occupancy)*** is the voxel occupancy likelihood standard deviation
4. ***pMin*** is the state change detection threshold.
5. ***m*** is the convolution kernel size.
6. ***minOtsu*** is the minimum number of connected voxels to be identified as dynamic
7. ***wLocal*** is the local window size used for the spatiotemporal convolution
8. ***wDynamic*** is the dynamic window size used for retaining dynamic detections
9. ***wGlobal*** is the global window size used for removing voxels from the map
10. ***delay*** is the number of scans to delay the prediction by - this is useful for incorporating more information before detecting dynamic points

There are also hardware settings:
* ***rMax*** is the maximum radius of the point cloud. This is hardware-dependent and not a configuration parameter.
* ***rMin*** is the minimum radius of the point cloud. This is hardware-dependent and not a configuration parameter.

## Benchmark Datasets
We test our algorithm using the following open-source datasets. Click the links below to see the download instructions.
* [HeLiMOS](https://sites.google.com/view/helimos)
    * The dataset is based on the KAIST05 sequence of the [HeliPR dataset](https://sites.google.com/view/heliprdataset) recorded by four different LiDARs. The annotated lables are used for evaluation.  We estimate the LiDAR pose using [SiMpLE](https://github.com/vb44/SiMpLE), with the estimated poses provided under the *datasetPoses* folder in this repository.

    <div style="display: flex; justify-content: center;">

    | **Method**                          | **L**  | **A**  | **O**  | **V**  | **Avg** |
    |-------------------------------------|--------|--------|--------|--------|---------|
    | 4DMOS, online [Mersch2022]          | 52.1   | 54.0   | 64.2   | 4.7    | 43.7    |
    | 4DMOS, delayed [Mersch2022]         | 59.0   | 58.3   | 70.4   | 5.4    | 48.3    |
    | MapMOS, Scan [Mersch2023]           | 58.9   | 63.2   | 81.4   | 4.3    | 52.0    |
    | MapMOS, Volume [Mersch2023]         | **62.7**| 66.6  | **82.9**| 5.8    | 54.5   |
    | **HMM-MOS**, Δ=0.25m                | 51.3   | 69.8   | 75.0   | 35.0   | 57.8    |
    | **HMM-MOS**, delayed (10 scans)     | 57.6   | **70.0**| 73.4  | **53.9**| **63.7**|

    </div>


* [Sipailou Campus](https://github.com/xieKKKi/MotionBEV)
    * The dataset consists of eight sequences using a Livox Avia mounted to a mobile robot. The sequences are available in the same format as Semantic-KITTI using *.bin* files and corresponding ground truth in *.label* files. The provided sensor pose estimates are used.

    <div style="display: flex; justify-content: center;">

    | **Method**                           | **IoU (%) (Validation)** | **IoU (%) (Test)** |
    |--------------------------------------|--------------------------|--------------------|
    | LMNet [Chen2021]                     | 5.37                     | 6.88               |
    | MotionSeg3D [Sun2022]                | 6.83                     | 6.72               |
    | 4DMOS [Mersch2022]                   | 78.54                    | 82.30              |
    | Motion-BEV [Zhou2023]                | 50.44                    | 52.02              |
    | Motion-BEV-h [Zhou2023]              | 70.94                    | 71.51              |
    | **HMM-MOS**, Δ=0.25m                 | **85.60**                | **87.00**          |

    </div>


* [Apollo Dataset](https://www.ipb.uni-bonn.de/html/projects/apollo_dataset/LiDAR-MOS.zip)
    * Data sourced from [here](https://github.com/PRBonn/MapMOS?tab=readme-ov-file#downloads). The dataset consists of multiple sequences recorded from a vehicle in an urban environment. The LiDAR pose estimates are provided at *benchmarking/datasetPoses/Apollo*.

    <div style="display: flex; justify-content: center;">

    | **Method**                           | **IoU (%)** |
    |--------------------------------------|-------------|
    | LMNet [Chen2021]                     | 13.7        |
    | MotionSeg3D, v1 [Sun2022]            | 6.5         |
    | MotionSeg3D, v2 [Sun2022]            | 8.8         |
    | 4DMOS, delayed [Mersch2022]          | 70.9        |
    | 4DMOS, online [Mersch2022]           | 68.7        |
    | MapMOS, Scan [Mersch2023]            | 79.2        |
    | MapMOS, Volumetric [Mersch2023]      | **81.7**    |
    | **HMM-MOS**, Δ=0.25m                 | **81.7**    |

    </div>

* [Urban Dynamic Objects LiDAR Dataset (DOALS)](https://projects.asl.ethz.ch/datasets/doku.php?id=doals)
    * Download instructions are also available on the [open-source Dynablox page](https://github.com/ethz-asl/dynablox?tab=readme-ov-file#Datasets). The dataset consists of eight sequences recorded with a handheld LiDAR in indoor and outdoor environments. The dataset is recorded in rosbags (see **note 1** below). We estimate the LiDAR pose using [SiMpLE](https://github.com/vb44/SiMpLE).

    <div style="display: flex; justify-content: center;">

    | **Method**                             | **ST**  | **SV**  | **HG**  | **ND**  |
    |----------------------------------------|---------|---------|---------|---------|
    | DOALS-3DMiniNet [Pfreundschuh2021]     | 84.0    | 82.0    | 82.0    | 80.0    |
    | 4DMOS [Mersch2022]                     | 38.8    | 50.6    | 71.1    | 40.2    |
    | LMNet [Chen2021] (Original)            | 6.0     | 7.5     | 4.6     | 3.0     |
    | LMNet [Chen2021] (Refit)               | 19.9    | 18.9    | 27.4    | 40.1    |
    | Dynablox [Schmid2023]                  | **86.2**| **83.2**| 84.1    | **81.6**|
    | **HMM-MOS**, Δ=0.20m                    | 82.7    | 80.8    | **85.9**| 81.4    |
    | LC Free Space [Modayil2008] (20m)      | 48.7    | 31.9    | 24.7    | 17.7    |
    | ST Normals [Falque2023] (20m)          | 80.0    | 81.0    | 85.0    | 76.0    |
    | Dynablox [Schmid2023] (20m)            | 87.3    | **87.8**| 86.0    | 83.1    |
    | **HMM-MOS**, Δ=0.20m (20m)              | **88.9**| 84.7    | **87.3**| **83.5**|

    </div>


* [Dynablox](https://github.com/ethz-asl/dynablox?tab=readme-ov-file#Datasets)
    * This qualitative dataset was released by Dynablox. The datasets consist of eight sequences recorded with a handheld LiDAR, capturing the dynamic motion of various objects in complex environments. The dataset is recorded in rosbags (see **note 1** below). We estimate the LiDAR pose using [SiMpLE](https://github.com/vb44/SiMpLE).

**Note 1**: The HMM-MOS implementation provided here is designed to work with scan files in the *.bin* KITTI format with the sensor pose of each scan provided in a poses.txt file which is also in the KITTI format (nx12). To test the DOALS and Dynablox datasets, we converted each rosbag to a sequence of deskewed *.bin* scan files and estimated the sensor pose using SiMpLE to provide accurate odometry. Comparison is made to other methods also using the deskewed point clouds.

## Hardware and Dependencies
This implementation has been tested on Ubuntu 20.04.5/6 LTS (Focal Fossa) with an Intel Core i7-10700K CPU @ 3.80GHz x 16 and 62.5 GiB memory.
HMM-MOS uses a few open-source libraries for reading the algorithm configuration file, Kd-Trees, hash maps, matrix operations, and CPU threading.
The installation instructions are detailed below.

* Git is required to download the open-source libraries.
```bash
sudo apt install git
```
* Install the g++ compiler.
```bash
sudo apt install g++
```
* Require a g++ v10 compiler.
```bash
sudo apt install g++-10
```
Ensure the newer compiler is used.
```bash
g++ --version
```
The expected output is shown below.
```bash
g++ (Ubuntu 10.5.0-1ubuntu1~20.04) 10.5.0
Copyright (C) 2020 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```
If the incorrect version is displayed, [set the correct compiler](https://askubuntu.com/questions/26498/how-to-choose-the-default-gcc-and-g-version).
```bash
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10
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
* Install the *boost* libraries.
```bash
sudo apt-get install libboost-all-dev
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
minRange: 0.5           # [m]
maxRange: 50            # [m]
outputFile: true        # true/false
scanNumsToPrint: [969, 1079] # Leave as [] if not being used. Scans indexed from 1.
outputFileName: /outputFilePathAndName
outputLabels: true      # true/false
outputLabelFolder: /outputLabelFolderPath

# Configuration parameters
voxelSize: 0.25                 # [m]
occupancySigma: 0.25             # lumped uncertainty [m]
beliefThreshold: 0.99           # probability [0-1]
convSize: 5                     # odd integer
localWindowSize: 3              # local window size [#]
globalWindowSize: 300           # global window size [#]
dynamicRegionWindowSize: 100    # global window size [#]
numScansDelay: 0                # set to zero for online performance [#]
minOtsu: 3                      # min dynamic voxels in spatiotemporal convolution [#]
```   

## Sample Results Interpretation
MOS results can be saved in,
* a *single file* with point cloud indicies for each scan per row for evaluation with *DOALS* ground truth, or,
* *.label* files from Semantic KITTI for evaluation with *Sipailou Campus* and *Apollo* datasets.

### HeLiMOS Evaluation
The ground truth labels are downloaded from HeLiMOS.
The *semantic-kitti-api* tools are used to evaluate the results.
The steps are outlined below.

1. Copy the relevant label predictions (for which there is a ground truth) from HMM-MOS to a sequences directory. The required file structure is shown below. The numbers correspond to the four LiDARS: 00 (Aeva), 01 (Avia), 02 (Ouster), 03 (Velodyne).
```bash
sequences
├── 00
│   ├── labels
│   ├── predictions
│   └── velodyne
├── 01
│   ├── labels
│   ├── predictions
│   └── velodyne
├── 02
│   ├── labels
│   ├── predictions
│   └── velodyne
└── 03
    ├── labels
    ├── predictions
    └── velodyne
```
2. For example, if we want to evaluate sequence 03 (Velodyne). Edit the *config/semantic-kitti-mos.yaml* file to replace *8* in the valid field to *3*.
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
    - 3 # was 8 originally
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
3. Run the evaluation for sequences 00 and 03.
```bash
python3 evaluate_mos.py --dataset /pathToFolder/
```
The expected output is shown below.
```bash
********************************************************************************
INTERFACE:
Data:  /pathToSequencesFolder/
Predictions:  /pathToSequencesFolder/
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
labels:  3164
predictions:  3164
Evaluating sequences: 10% 20% 30% 40% 50% 60% 70% 80% 90% ********************************************************************************
below can be copied straight for paper table
iou_moving: 0.539

```

### Sipailou Campus Evaluation
The Sipailou campus dataset is provided by [MotionBEV](https://github.com/xieKKKi/MotionBEV/).
MotionBEV use the Semantic Kitti API to compute the *validation* (seq 06) and *test* (seq 00, 07) IoUs.
To provide a fair evaluation with the results published in the [MotionBEV paper](https://ieeexplore.ieee.org/document/10287575), we use the same evaluation tools.
The evaluation steps are outlined below.

***It is important to ensure the HMM-config has a minimum range of 3m and a maximum range of 50m as the ground truth is only labelled within these ranges.***

1. Clone the [Semantic Kitti API](https://github.com/PRBonn/semantic-kitti-api) repository.
```bash
git clone https://github.com/PRBonn/semantic-kitti-api.git
```
2. Copy the label predictions to the *sipailou-livox-kitti* directory sequence folder. The required file structure is:
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
3. A copy of the *livox-SEU-MOS.yaml* evaluation config from the [MotionBEV repository](https://github.com/xieKKKi/MotionBEV) can be located at *HMM-MOS/config/eval/* for convenience.
4. Run the validation evaluation.
```bash
cd semantic-kitti-api
python3 evaluate_mos.py --dataset /pathToFolder/sipailou-livox-kitti/ --datacfg /pathToHmmMos/config/eval/livox-SEU-MOS.yaml -s valid
```
The expected output is shown below.
```bash
********************************************************************************
INTERFACE:
Data:  /pathToFolder/sipailou-livox-kitti/
Predictions:  /pathToFolder/sipailou-livox-kitti/
Backend:  numpy
Split:  valid
Config:  /pathToHmmMos/config/eval/livox-SEU-MOS.yaml
Limit:  None
Codalab:  None
********************************************************************************
Opening data config file /pathToHmmMos/config/eval/livox-SEU-MOS.yaml
[IOU EVAL] IGNORE:  []
[IOU EVAL] INCLUDE:  [0 1]
labels:  3191
predictions:  3191
Evaluating sequences: 10% 20% 30% 40% 50% 60% 70% 80% 90% ********************************************************************************
below can be copied straight for paper table
iou_moving: 0.856
```
5. Run the test evaluation.
```bash
python3 evaluate_mos.py --dataset /pathToFolder/sipailou-livox-kitti/ --datacfg /pathToHmmMos/config/eval/livox-SEU-MOS.yaml -s test
```
The expected output is shown below.
```bash
********************************************************************************
INTERFACE:
Data:  /pathToFolder/sipailou-livox-kitti/
Predictions:  /pathToFolder/sipailou-livox-kitti/
Backend:  numpy
Split:  test
Config:  /pathToHmmMos/config/eval/livox-SEU-MOS.yaml
Limit:  None
Codalab:  None
********************************************************************************
Opening data config file /pathToHmmMos/config/eval/livox-SEU-MOS.yaml
[IOU EVAL] IGNORE:  []
[IOU EVAL] INCLUDE:  [0 1]
labels:  6201
predictions:  6201
Evaluating sequences: 10% 20% 30% 40% 50% 60% 70% 80% 90% ********************************************************************************
below can be copied straight for paper table
iou_moving: 0.870
```

### DOALS Evaluation
The DOALS dataset provides 10 manually labelled scans per sequence for evaluation in *indicies.csv* files.
An evaluation tool is provided in *benchmarking/evaluation*.

#### Build evaluation tools for DOALS
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

#### Evaluate results
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
| 83.68 |   96.99   | 85.91 | 
| 84.27 |   91.50   | 91.42 | 
| 96.89 |   99.41   | 97.45 | 
| 92.83 |   99.40   | 93.35 | 
| 85.91 |   99.53   | 86.26 | 
| 86.22 |   98.48   | 87.39 | 
| 95.40 |   99.35   | 96.00 | 
| 87.70 |   98.06   | 89.25 | 
| 81.92 |   94.48   | 86.04 | 
| 97.33 |   99.77   | 97.55 | 
|-------|-----------|-------|
Mean IOU: 89.22
```

## References
[1] [eigen library](https://eigen.tuxfamily.org/dox/GettingStarted.html)
