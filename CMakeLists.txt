# Enable C++11
cmake_minimum_required(VERSION 3.20)

SET(CMAKE_CXX_STANDARD 17) #Open3D需要C++14以上版本
SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
SET(CMAKE_EXE_LINKER_FLAGS "-lstdc++")

#### AcquiringPointCloud  ####
# Define project name 
project(CasPointCloud)
SET(this_target AcquiringPointCloud)

# Kinect DK相机
find_package(k4a REQUIRED)

# eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

# set(Protobuf_FOUND "True")
# set(Protobuf_VERSION "3.20.3")
# set(Protobuf_INCLUDE_DIRS "/usr/local/include")
# set(Protobuf_LIBRARIES "/usr/local/lib/libprotobuf.so;-lpthread")
# set(Protobuf_PROTOC_LIBRARIES "/usr/local/lib/libprotoc.so;-lpthread")
# set(Protobuf_LITE_LIBRARIES "/usr/local/lib/libprotobuf-lite.so;-lpthread")

# Protobuf
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/protobuf/*.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# OPEN3D
find_package(Open3D REQUIRED)
include_directories("/usr/local/include/open3d")

# JSON
find_package(jsoncpp REQUIRED)


add_executable(EmergencyTeleoperatedRobotSystem
        src/EmergencyTeleoperatedRobotSystem.cpp 
        src/CasAzureKinectExtrinsics.cpp 
        src/CasSoundSourceLocalization.cpp
        src/CasAzureKinect.cpp 
        src/CasUtility.cpp
        src/CasBot.cpp
        src/CasNetwork.cpp
        ${PROTO_SRCS} 
        ${PROTO_HDRS} 
        )

# add_executable(CasControlCar
#         src/CasControlCar.cpp
#         src/CasBot.cpp
#         src/CasUtility.cpp
#         )

# 指定头文件目录
include_directories(include)

target_link_libraries(EmergencyTeleoperatedRobotSystem
        k4a::k4a
        ${Open3D_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${Protobuf_LIBRARIES}
        asound
        fftw3
        )

# target_link_libraries(CasControlCar
#         ncurses
#         )
        



#### CasGeneratePointCloud ####
# 快速点云

#SET(this_target CasGeneratePointCloud)
#
#find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a
#
#find_package(Open3D REQUIRED)
#
#find_package(Eigen3 REQUIRED)
#include_directories("/usr/include/eigen3")
#
#add_executable(CasGeneratePointCloud CasGeneratePointCloud.cpp CasAzureKinectExtrinsics.cpp CasPointCloud.cpp)
#
#target_link_libraries(CasGeneratePointCloud
#        k4a::k4a
#        ${Open3D_LIBRARIES}
#        )

#### CasGenerateColorPointCloud ####
# 快速彩色点云

#SET(this_target CasGenerateColorPointCloud)
#
#find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a
#
#find_package(Open3D REQUIRED)
#
#find_package(Eigen3 REQUIRED)
#include_directories("/usr/include/eigen3")
#
#add_executable(CasGenerateColorPointCloud CasGenerateColorPointCloud.cpp CasAzureKinectExtrinsics.cpp CasPointCloud.cpp)
#
#target_link_libraries(CasGenerateColorPointCloud
#        k4a::k4a
#        ${Open3D_LIBRARIES}
#        )


#### CasGeneratePointCloud ####
# 快速点云

#SET(this_target FastPointCloud)
#
#find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a
#
#add_executable(FastPointCloud FastPointCloud.cpp)
#
#target_link_libraries(FastPointCloud
#        k4a::k4a
#        )



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








#### CasViewingPointCloud ####
# 点云可视化

# SET(this_target CasViewingPointCloud)

# find_package(Open3D HINTS REQUIRED)
# include_directories(${Open3D_INCLUDE_DIRS})
# include_directories(/usr/local/include/open3d/3rdparty)
# link_directories(${Open3D_LIBRARY_DIRS})

# add_executable(CasViewingPointCloud src/CasViewingPointCloud.cpp)

# target_link_libraries(CasViewingPointCloud ${Open3D_LIBRARIES})





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

#SET(this_target AzureKinectViewer)
#
#find_package(k4a REQUIRED)
#
##OPEN3D
#find_package(Open3D REQUIRED)
#include_directories(${Open3D_INCLUDE_DIRS})
#include_directories(/usr/local/include/open3d/3rdparty)
#link_directories(${Open3D_LIBRARY_DIRS})
#
#add_executable(AzureKinectViewer AzureKinectViewer.cpp)
#
#target_link_libraries(AzureKinectViewer
#        ${Open3D_LIBRARIES}
#        k4a::k4a
#        )



#### AzureKinectRecord ####
# 官方示例

#SET(this_target AzureKinectRecord)
#
#find_package(k4a REQUIRED)
#
##OPEN3D
#find_package(Open3D REQUIRED)
#include_directories(${Open3D_INCLUDE_DIRS})
#include_directories(/usr/local/include/open3d/3rdparty)
#link_directories(${Open3D_LIBRARY_DIRS})
#
#add_executable(AzureKinectRecord AzureKinectRecord.cpp CasAzureKinectExtrinsics.cpp)
#
#target_link_libraries(AzureKinectRecord
#        ${Open3D_LIBRARIES}
#        k4a::k4a
#        )



#### AzureKinectMKVReader ####
# 官方示例

#SET(this_target AzureKinectMKVReader)
#
#find_package(k4a REQUIRED)
#
##OPEN3D
#find_package(Open3D REQUIRED)
#include_directories(${Open3D_INCLUDE_DIRS})
#include_directories(/usr/local/include/open3d/3rdparty)
#link_directories(${Open3D_LIBRARY_DIRS})
#
#add_executable(AzureKinectMKVReader AzureKinectMKVReader.cpp)
#
#target_link_libraries(AzureKinectMKVReader
#        k4a::k4a
#        ${Open3D_LIBRARIES}
#        )
#