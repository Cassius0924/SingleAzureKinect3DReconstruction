//
// Created by root on 4/7/23.
//

#ifndef SMALL_AZURE_KINECT_DK_3D_RECONSTRUCTION_REGISTRATION_H
#define SMALL_AZURE_KINECT_DK_3D_RECONSTRUCTION_REGISTRATION_H

#include <open3d/Open3D.h>
#include <k4a/k4a.h>
#include <iostream>

namespace cas{
    //open3d数据
    namespace o3d {

        // 点云配准
        void registration(std::shared_ptr<open3d::geometry::PointCloud> source,
                          std::shared_ptr<open3d::geometry::PointCloud> target);

        // 点云相加
        void add(std::shared_ptr<open3d::geometry::PointCloud> source,
                  std::shared_ptr<open3d::geometry::PointCloud> target);

        // 保存文件
        void save(std::shared_ptr<open3d::geometry::PointCloud> source,
                  std::string filename);

        // 点云可视化
        void show(std::shared_ptr<open3d::geometry::PointCloud> source);

        // k4a_image_t转open3d点云
        void k4a_image_to_o3d_point_cloud(k4a_image_t image, std::shared_ptr<open3d::geometry::PointCloud> point_cloud);
    }

    //pcl数据
    namespace pcl{
        // TODO:PCL点云格式转Open3D点云格式
    }
}

#endif //SMALL_AZURE_KINECT_DK_3D_RECONSTRUCTION_REGISTRATION_H
