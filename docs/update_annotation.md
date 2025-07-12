# Update Annotations
## Camera's extrinsic
There are bugs for camera's extrinsic in annotation file (`.yaml`) in rendered datasets, include `Multi-V2X` and `CoRTSG`. If you want to use the extrinsic directly, follow the steps.

### Bug description
In datasets of `Multi-V2X` and `CoRTSG`, the extrinsic matrix of a camera represents the transformation from LiDAR to camera, calculated as `C = AB`, where `A` denotes transformation from world to camera and `B` denotes transformation from lidar to world. However, during saving annotation, `C = AB` was mistaken as `C = BA`.

### Bug effects
If you use camera's extrinsic in annotation file, the results are wrong. Ohterwise, there is no effect.

### Bug fixing
The bug have been fixed in code of `CoRTSG/coriskyscene/data_collection`. 

For rendered data, recalculate extrinsic of the camera using recorded poses and update the annotation file accordingly. 
The two fixing scripts `fix_anno.py` and `replace_anno.py` lie in `CoRTSG/coriskyscene/data_collection/util/`. 

The updated data of `Multi-V2X` and `CoRTSG` has been uploaded to OpenDataLab ([here](https://opendatalab.com/Rongsong/CoRTSG) and [here](https://opendatalab.com/Rongsong/Multi-V2X)) with names of `Multi-V2X-fix` and `CoRTSG-fix`. If you download the datasets before that, to avoid re-download the whole datasets again, there are three methods to fix:

**(1) Fix inplace**

Modify `config` in `fix_anno.py` and run the script. This will recalculate the camera's extrinsic and resave annotation yaml file. 

**(2) Fix with fixed annotations (Recommended)**

Download `anno_fix_250601.zip` from OpenDataLab and unzip to local; modify `config` in `replace_anno.py` according to your unzip directory; run `python replace_anno.py`. The annotation yaml files would be replaced by the fixed ones.

**(3) Fix in code**

Recalculate extrinsic everytime you need to use it, e.g., in `OpenCOOD`:
```python
from opencood.utils.transformation_utils import x1_to_x2
yaml_path = "Your yaml path"
anno = load_yaml(yaml_path)
camera_name = "cameraFront"
lidar_to_camera = x1_to_x2(anno["lidar_pose"], anno[camera_name]["coords"])
anno[camera_name]["extrinsic"] = lidar_to_camera
``` 

Or download `anno_fix_250601.zip` from OpenDataLab and replace annotation file path with the fixed one. E.g, in `OpenCOOD`:
```python
# get file paths for point cloud, camera and annotation
new_anno_agent_dir = "New annotaion dir for agent"
yaml_file = os.path.join(new_anno_agent_dir, timestamp + ".yaml")
lidar_file = os.path.join(agent_dir, timestamp + '.pcd')
...
scenario_data[agent_id][timestamp] = {
    "yaml": yaml_file,
    "lidar": lidar_file,
    "camera0": camera_files,
    "view": view_file
}
```

### Verify
In this fork of [OpenCOOD](https://github.com/RadetzkyLi/OpenCOOD), one can use script `opencood/visualization/vis_camera.py` to visualize bouding boxes on images.

If you have already had a fork of OpenCOOD, copying `opencood/utils/camera_utils.py` and `opencood/visualization/vis_camera.py` to corresponding directory will work.
