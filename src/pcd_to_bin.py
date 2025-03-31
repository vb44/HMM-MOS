import os
import numpy as np
import open3d as o3d

def write_bin(input_file_name, sample_time):
    # Format the filename with zero-padding and the sample time
    # file_name = os.path.join(save_folder_file, ".bin")
    file_name = os.getcwd()+"/test.bin"
    file_name = os.path.join("/media/vb-dell/Expansion/datasets/ERS_Bpearl/2023-07-15-07-08-07-RS-Bpearl-Data_full_load7/bins/", f"{sample_time:09}.bin")
    pc = o3d.io.read_point_cloud(input_file_name)
    pc_pts = np.asarray(pc.points)
    
    print(file_name)
    
    # Prepare the data to write
    with open(file_name, "wb") as fout:
        for row in pc_pts:
            x = float(row[0])
            y = float(row[1])
            z = float(row[2])
            i = float(1.0)

            # Pack the data into binary format with 1 as the fourth value
            p = [x, y, z, i]
            fout.write(np.array(p, dtype=np.float32).tobytes())

mypath = "/media/vb-dell/Expansion/datasets/ERS_Bpearl/2023-07-15-07-08-07-RS-Bpearl-Data_full_load7/pcds"
onlyfiles = [f for f in os.listdir(mypath) if os.path.isfile(os.path.join(mypath, f))]


for i in range(len(onlyfiles)):
    input_file_name = mypath + "/" + onlyfiles[i]
    write_bin(input_file_name, i)
    

# import pdb; pdb.set_trace()
# write_height_grid_to_file_with_dyn()