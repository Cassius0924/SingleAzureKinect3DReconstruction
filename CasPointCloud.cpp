//
// Created by root on 4/5/23.
//

#include "CasPointCloud.h"

#include <iostream>
#include <vector>

#include <open3d/Open3D.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/TransformationEstimation.h>

#include <Eigen/Core>

using namespace std;

void cas::o3d::registration(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target) {

    double threshold = 1.0; // 移动范围的阀值
    Eigen::Matrix4d trans_init = Eigen::Matrix4d::Identity(); // 4x4 identity matrix，这是一个转换矩阵，象征着没有任何位移，没有任何旋转，我们输入这个矩阵为初始变换

    // 运行ICP
    open3d::pipelines::registration::RegistrationResult reg_p2p;
    reg_p2p = open3d::pipelines::registration::RegistrationICP(
            *source, *target, threshold, trans_init,
            open3d::pipelines::registration::TransformationEstimationPointToPoint());

    // 将我们的点云依照输出的变换矩阵进行变换
    source->Transform(reg_p2p.transformation_);
}


void cas::o3d::add(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target){
    *source += *target;
}

void cas::o3d::save(std::shared_ptr<open3d::geometry::PointCloud> source, std::string path){
    open3d::io::WritePointCloud(path, *source);
}

void cas::o3d::show(std::shared_ptr<open3d::geometry::PointCloud> source){
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.push_back(source);

    // 使用 `DrawGeometries()` 函数来显示点云
    open3d::visualization::DrawGeometries(geometries);
}

void cas::o3d::k4a_image_to_o3d_point_cloud(const k4a_image_t point_cloud_image,std::shared_ptr<open3d::geometry::PointCloud> point_cloud) {
    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(point_cloud_image);

    k4a_float3_t *point_cloud_data = (k4a_float3_t *) (void *) k4a_image_get_buffer(point_cloud_image);

    std::vector<Eigen::Vector3d> points;    // 用于存储点云数据
    for (int i = 0; i < width * height; i++) {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z)) {
            continue;
        }
        points.push_back(Eigen::Vector3d(point_cloud_data[i].xyz.x, point_cloud_data[i].xyz.y, point_cloud_data[i].xyz.z));
    }
    point_cloud->points_ = points;
    //颜色
    std::vector<Eigen::Vector3d> colors;
    for (int i = 0; i < width * height; i++) {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z)) {
            continue;
        }
        colors.push_back(Eigen::Vector3d(1, 0, 0)); //红色
    }
    point_cloud->colors_ = colors;

}


//int main() {
    // 读取电脑中的 ply 点云文件
//    auto source = std::make_shared<open3d::geometry::PointCloud>();
//    auto target = std::make_shared<open3d::geometry::PointCloud>();
//    open3d::io::ReadPointCloud("ply-data/1.ply", *source);
//    open3d::io::ReadPointCloud("ply-data/2.ply", *target);
//
//     为两个点云上上不同的颜色
//    source->PaintUniformColor({1, 0.706, 0});    // source 为黄色
//    target->PaintUniformColor({0, 0.651, 0.929});// target 为蓝色

    // 为两个点云分别进行outlier removal
//    auto processed_source = source->RadiusOutlierRemoval(16, 0.5);
//    auto processed_target = target->RadiusOutlierRemoval(16, 0.5);

//    double threshold = 1.0; // 移动范围的阀值
//    Eigen::Matrix4d trans_init = Eigen::Matrix4d::Identity(); // 4x4 identity matrix，这是一个转换矩阵，象征着没有任何位移，没有任何旋转，我们输入这个矩阵为初始变换
//
//    // 运行ICP
//    open3d::pipelines::registration::RegistrationResult reg_p2p;
//    reg_p2p = open3d::pipelines::registration::RegistrationICP(
//            *source, *target, threshold, trans_init,
//            open3d::pipelines::registration::TransformationEstimationPointToPoint());
//
//    // 将我们的点云依照输出的变换矩阵进行变换
//    source->Transform(reg_p2p.transformation_);
//
//    // 点云相加：将两个点云合并到一起，这样我们就可以看到两个点云的对齐效果
//    *target += *source;

//    //保存结果
//    open3d::io::WritePointCloud("reg/reged.pcd", *target);

//    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
//    geometries.push_back(target);
//
//    // 使用 `DrawGeometries()` 函数来显示点云
//    open3d::visualization::DrawGeometries(geometries);

//    cas::o3d::registration(source, target); //ICP配准
//    cas::o3d::add(source, target);  //点云相加
//    cas::o3d::save(source, "reg/reged.pcd");    //保存结果
//    cas::o3d::show(source); //显示结果
//
//    return 0;
//}
