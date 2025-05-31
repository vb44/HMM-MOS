import numpy as np
import open3d as o3d
from pathlib import Path
import sys
import time

class PointCloudViewer:
    def __init__(self, folder_path, loop=False, loop_delay=0.1, point_size=1, include_gt=False):
        self.folder_path = Path(folder_path)
        self.files = sorted(
            list((self.folder_path / "velodyne").glob("*.bin")),
            key=lambda f: float(f.stem)
        )
        self.prediction_files = sorted(
            list((self.folder_path / "predictions").glob("*.label"))
        )
        self.label_files = None
        
        self.index = 0
        self.loop = loop
        self.loop_delay = loop_delay
        self.point_size = point_size
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.pcd = o3d.geometry.PointCloud()
        self.frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.window_created = False
        self.include_gt = include_gt
        if (self.include_gt):
            self.label_files = sorted(
                list((self.folder_path / "labels").glob("*.label"))
            )

    def load_point_cloud_with_predictions(self, point_file, prediction_file, label_file=None):
        points = np.fromfile(point_file, dtype=np.float32).reshape(-1, 4)[:, :3]
        predictions = np.fromfile(prediction_file, dtype=np.uint32)
        
        # The labels were written using: uint32_t(label) & 0xFFFF
        predictions = predictions & 0xFFFF

        if predictions.shape[0] != points.shape[0]:
            raise ValueError(f"Mismatch between number of points ({points.shape[0]}) and predictions ({predictions.shape[0]})")
        
        colors = np.zeros((points.shape[0], 3))
        if (self.include_gt):
            labels = np.fromfile(label_file, dtype=np.uint32)
            if labels.shape[0] != points.shape[0]:
                raise ValueError(f"Mismatch between number of points ({points.shape[0]}) and labels ({labels.shape[0]})")

            colors[(predictions == 9) & (labels == 9)] = [141/255, 152/255, 167/255]    # true negative (static)
            colors[(predictions == 251) & (labels == 251)] = [0, 1, 0]                  # true positive (dynamic)
            colors[(predictions == 251) & (labels == 9)] = [1, 0, 0]                    # false positive 
            colors[(predictions == 9) & (labels == 251)] = [0, 96/255, 1]               # false negative
        else:
            colors[predictions == 9] = [141/255, 152/255, 167/255]      # static
            colors[predictions == 251] = [0, 1, 0]                      # moving

        return points, colors
        
    def show(self):
        self.create_window()

        # Initial point cloud and frame
        if (self.include_gt):
            points, colors = self.load_point_cloud_with_predictions(self.files[self.index],
                                                                    self.prediction_files[self.index],
                                                                    self.label_files[self.index])
        else:
            points, colors = self.load_point_cloud_with_predictions(self.files[self.index],
                                                                    self.prediction_files[self.index])
            
        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

        self.vis.add_geometry(self.pcd)
        self.vis.add_geometry(self.frame)

        render_option = self.vis.get_render_option()
        render_option.point_size = self.point_size

        self.vis.register_key_callback(262, self.next_frame)  # Right arrow
        self.vis.register_key_callback(263, self.prev_frame)  # Left arrow

        if self.loop:
            while True:
                self.update_geometry()
                self.index = (self.index + 1) % len(self.files)
                time.sleep(self.loop_delay)
                if not self.vis.poll_events():
                    break
                self.vis.update_renderer()
        else:
            self.vis.run()

        self.vis.destroy_window()

    def create_window(self):
        title = "MOS Viewer"
        self.vis.create_window(window_name=title, width=960, height=540)
        self.window_created = True

    def update_geometry(self):
        if (self.include_gt):
            points, colors = self.load_point_cloud_with_predictions(self.files[self.index],
                                                                    self.prediction_files[self.index],
                                                                    self.label_files[self.index])
        else:
            points, colors = self.load_point_cloud_with_predictions(self.files[self.index],
                                                                    self.prediction_files[self.index])
        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)
        self.vis.update_geometry(self.pcd)
        sys.stdout.write('\r' + ' ' * 80 + '\r')
        print(f"Now viewing: {self.files[self.index].name}", end='', flush=True)

    def next_frame(self, vis):
        if self.index < len(self.files) - 1:
            self.index += 1
            self.update_geometry()
        return False

    def prev_frame(self, vis):
        if self.index > 0:
            self.index -= 1
            self.update_geometry()
        return False


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MOS Viewer")
    parser.add_argument("--path", required=True, help="Path to folder containing velodyne and prediction folders")
    parser.add_argument("--loop_delay", type=float, default=0.1, help="Delay in seconds between frames when looping")
    parser.add_argument("--point_size", type=float, default=1.0, help="Size of the rendered point cloud points")
    parser.add_argument("--loop", action="store_true", help="Automatically loop over the scans")
    parser.add_argument("--include_gt", action="store_true", help="Include the ground truth labels.")
    args = parser.parse_args()

    viewer = PointCloudViewer(args.path, loop=args.loop, loop_delay=args.loop_delay, point_size=args.point_size, include_gt=args.include_gt)
    viewer.show()