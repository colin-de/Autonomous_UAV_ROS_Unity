# Vision
## Point Cloud Generation
To generate point cloud data we need depth image and the intrinsic matrix of the depth camera. This package depends on [depth_image_proc](../depth_image_proc) (source: https://github.com/ros-perception/image_pipeline/tree/melodic/depth_image_proc) to finish this part.

In the [launchfile](./launch/vision.launch) we first launch a depth_image_proc node, making it submit to the topic `/realsense/depth/camera_info`which contains the intrinsic matrix of th camera and the topic `/realsense/depth/image` which contains the raw depth image.

## Voxel Grid Generation
The vision node subscribes to the published point cloud and handles it with `pcl::VoxelGrid` filter. Here we set the leaf size of 0.2 meter to downsample the data. 
Because the coordinate system of the camera is different from the one used in the mapping part, the point cloud data is rotated by 90 degree around the y-axis and -90 degree around the z-axis.
