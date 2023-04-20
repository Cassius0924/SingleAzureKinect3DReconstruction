# SingleAzureKinect3DReconstruction
此项目基于 Open3D 和 Azure Kinect DK 实现了三维重建。基于 IMU 传感器实现粗配准，基于彩色 ICP 算法实现精配准。

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