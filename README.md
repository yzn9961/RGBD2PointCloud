# RGBD2PointCloud

A simple demon to convert the Depth&RGB information capture by `Astra pro` depth camera into 3D point cloud.

一个将`Astra pro`深度相机采集到的图像信息转换为三维点云的简单项目。

![livingroom](./RGBD2PointCloud/demon/livingroom.gif)

项目包含以下程序:

- `RGBD2PointCloud`:将RGBD采集到的颜色以及深度信息转换为三维点云
- `RGBDcalibra`: RGBD相机标定(单目相机标定）
- `RGBD_photo`: RGBD图像拍摄，同时存储RGB与深度图像
- `RGBD_vid`: RGBD视频录制，同时存储RGB与深度图像序列

Structure of **RGBD2PointCloud**:

- `RGBD2PointCloud`: Convert depth map to 3D point cloud.
- `RGBDcalibra`: Calibrate the RGBD camera. (same as mono-camera calibration)
- `RGBD_photo`: Using the `Astra Pro` RGBD camera to take photo (simultaneously store the RGB pic and it's depth map).
- `RGBD_vid`: Using the `Astra Pro` RGBD camera to capture a video (RGB and Depth information are saved as image sequance).

# Input(输入)

![livingroom](./RGBD2PointCloud/demon/livingroom.png)

# Output(输出)

![livingroom](./RGBD2PointCloud/demon/livingroom.gif)