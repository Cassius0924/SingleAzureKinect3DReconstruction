# SingleAzureKinect3DReconstruction
此项目基于 Open3D 和 Azure Kinect DK 实现了三维重建。利用 Azure Kinect DK 捕获图像并记录 IMU 数据，利用 Open3D 实现三维重建。

# 依赖库
- Azure Kinect SDK
- Eigen
- Open3D

# 创建数据存放文件夹
```bash
mkdir build
cd build
mkdir pcd ply ply-data reg
```