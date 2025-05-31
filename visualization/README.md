# Visualization
This simple utility provides the ability to visualize the MOS results.
A sample result is shown and the usage is outlined below.

[Demo!]()

## Usage
1. Install the requirements.
```bash
pip install -r requirements.txt
```
2. Run the viewer.
```bash
python3 src/mos_viewer.py -h

usage: viewer.py [-h] --path PATH [--loop_delay LOOP_DELAY] [--point_size POINT_SIZE] [--loop] [--include_gt]

MOS Viewer

optional arguments:
  -h, --help            show this help message and exit
  --path PATH           Path to folder containing velodyne and prediction folders
  --loop_delay LOOP_DELAY
                        Delay in seconds between frames when looping
  --point_size POINT_SIZE
                        Size of the rendered point cloud points
  --loop                Automatically loop over the scans
  --include_gt          Include the ground truth labels.
```
3. Example command.
```bash
python3 src/mos_viewer.py config/config.yaml
```