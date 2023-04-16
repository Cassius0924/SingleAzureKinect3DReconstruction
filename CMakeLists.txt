# Enable C++11
cmake_minimum_required(VERSION 3.10)

SET(CMAKE_CXX_STANDARD 17) #Open3D需要C++14以上版本
SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

#### AcquiringPointCloud  ####
# Define project name 
# project(AcquiringPointCloud)
SET(this_target AcquiringPointCloud)

# Kinect DK相机
find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a

# eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

# pcl
#find_package(PCL REQUIRED)
#set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.9")
#include_directories(${PCL_INCLUDE_DIRS})  #.h 文件
#link_directories(${PCL_LIBRARY_DIRS}) # .so文件
#add_definitions(${PCL_DEFINITIONS})

#Boost
#find_package(Boost REQUIRED COMPONENTS system)
#include_directories(${Boost_INCLUDE_DIRS})
#include_directories("/usr/include/boost")
#link_directories(${Boost_LIBRARY_DIRS})

#OPENSSL
#find_package(OpenSSL REQUIRED)

#VTK
#find_package(VTK REQUIRED)
#include_directories(${VTK_INCLUDE_DIRS})
#link_directories(${VTK_LIBRARY_DIRS})

#OPENCV
#find_package(OpenCV REQUIRED)
#include_directories(${OPENCV_INCLUDE_DIRS})
#link_directories(${OPENCV_LIBRARY_DIRS})

#OPEN3D
find_package(Open3D HINTS REQUIRED)

# Declare the executable target built from your sources
add_executable(AcquiringPointCloud AcquiringPointCloud.cpp CasAzureKinectExtrinsics.cpp)

# Link your application with other libraries
target_link_libraries(AcquiringPointCloud
        ${Eigen3_LIBRARIES}
        ${Open3D_LIBRARIES}
        k4a::k4a
        )
#libboost_system.so
#libboost_filesystem.so
#${OPENSSL_LIBRARIES}
#${Boost_LIBRARIES}
#${OpenCV_LIBS}
#${VTK_LIBRARIES}
#${PCL_LIBRARIES}



#### CasGeneratePointCloud ####
# 快速点云

SET(this_target CasGeneratePointCloud)

find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a

find_package(Open3D REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

add_executable(CasGeneratePointCloud CasGeneratePointCloud.cpp CasAzureKinectExtrinsics.cpp CasPointCloud.cpp)

target_link_libraries(CasGeneratePointCloud
        k4a::k4a
        ${Open3D_LIBRARIES}
        )

#### CasGenerateColorPointCloud ####
# 快速彩色点云

SET(this_target CasGenerateColorPointCloud)

find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a

find_package(Open3D REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

add_executable(CasGenerateColorPointCloud CasGenerateColorPointCloud.cpp CasAzureKinectExtrinsics.cpp CasPointCloud.cpp)

target_link_libraries(CasGenerateColorPointCloud
        k4a::k4a
        ${Open3D_LIBRARIES}
        )


#### CasGeneratePointCloud ####
# 快速点云

SET(this_target FastPointCloud)

find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a

add_executable(FastPointCloud FastPointCloud.cpp)

target_link_libraries(FastPointCloud
        k4a::k4a
        )



#### CasPointCloud ####
# Cas的点云处理

#SET(this_target CasPointCloud)

#find_package(Open3D HINTS REQUIRED)
#set(Open3D_INCLUDE_DIRS "/usr/local/include")
#include_directories(${Open3D_INCLUDE_DIRS})
#include_directories(/usr/local/include/open3d/3rdparty)
#link_directories(${Open3D_LIBRARY_DIRS})
#
#add_executable(CasPointCloud CasPointCloud.cpp)
#
#target_link_libraries(CasPointCloud ${Open3D_LIBRARIES})





#### ViewingPointCloud ####
# 点云可视化

SET(this_target CasViewingPointCloud)

find_package(Open3D HINTS REQUIRED)
#set(Open3D_INCLUDE_DIRS "/usr/local/include")
include_directories(${Open3D_INCLUDE_DIRS})
include_directories(/usr/local/include/open3d/3rdparty)
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(CasViewingPointCloud CasViewingPointCloud.cpp)

target_link_libraries(CasViewingPointCloud ${Open3D_LIBRARIES})





#### AzureKinectExtrinsic ####
# AzureKinect外部参数

#SET(this_target CasAzureKinectExtrinsic)
#
#find_package(k4a REQUIRED)

#OPEN3D
#find_package(Open3D REQUIRED)
#include_directories(${Open3D_INCLUDE_DIRS})
#include_directories(/usr/local/include/open3d/3rdparty)
#link_directories(${Open3D_LIBRARY_DIRS})

#add_executable(CasAzureKinectExtrinsic CasAzureKinectExtrinsics.cpp)
#
#target_link_libraries(CasAzureKinectExtrinsic
#        k4a::k4a
#        )





#### AzureKinectViewer ####
# 官方示例

SET(this_target AzureKinectViewer)

find_package(k4a REQUIRED)

#OPEN3D
find_package(Open3D REQUIRED)
include_directories(${Open3D_INCLUDE_DIRS})
include_directories(/usr/local/include/open3d/3rdparty)
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(AzureKinectViewer AzureKinectViewer.cpp)

target_link_libraries(AzureKinectViewer
        ${Open3D_LIBRARIES}
        k4a::k4a
        )



#### AzureKinectRecord ####
# 官方示例

SET(this_target AzureKinectRecord)

find_package(k4a REQUIRED)

#OPEN3D
find_package(Open3D REQUIRED)
include_directories(${Open3D_INCLUDE_DIRS})
include_directories(/usr/local/include/open3d/3rdparty)
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(AzureKinectRecord AzureKinectRecord.cpp CasAzureKinectExtrinsics.cpp)

target_link_libraries(AzureKinectRecord
        ${Open3D_LIBRARIES}
        k4a::k4a
        )



#### AzureKinectMKVReader ####
# 官方示例

SET(this_target AzureKinectMKVReader)

find_package(k4a REQUIRED)

#OPEN3D
find_package(Open3D REQUIRED)
include_directories(${Open3D_INCLUDE_DIRS})
include_directories(/usr/local/include/open3d/3rdparty)
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(AzureKinectMKVReader AzureKinectMKVReader.cpp)

target_link_libraries(AzureKinectMKVReader
        k4a::k4a
        ${Open3D_LIBRARIES}
        )







#### AcquiringImages  ####
# 保持注释，项目用不到
# Define project name
#SET(this_target AcquiringImages)
## Kinect DK相机
#find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a
## Find OpenCV
#find_package(OpenCV REQUIRED)
## Add includes
#include_directories(${OpenCV_INCLUDE_DIRS})
## Declare the executable target built from your sources
#add_executable(AcquiringImages AcquiringImages.cpp)
## Link your application with other libraries
#target_link_libraries(AcquiringImages
#        k4a::k4a
#        ${OpenCV_LIBS}
#        )


#### Open3DTest ####

# 保持注释，Open3D测试用

#SET(this_target Open3DTest)
#
#find_package(Open3D HINTS REQUIRED)
##set(Open3D_INCLUDE_DIRS "/usr/local/include")
#include_directories(${Open3D_INCLUDE_DIRS})
#include_directories(/usr/local/include/open3d/3rdparty)
#link_directories(${Open3D_LIBRARY_DIRS})
#
#add_executable(Open3DTest Open3DTest.cpp)
#
#target_link_libraries(Open3DTest ${Open3D_LIBRARIES})
