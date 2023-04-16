//
// Created by root on 4/7/23.
//

#include <iostream>
#include <stack>

#include <open3d/Open3D.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/geometry/PointCloud.h>

using namespace std;

stack<Eigen::Vector3d> getColorStack() {
    //颜色栈
    stack<Eigen::Vector3d> colorStack;
    colorStack.push(Eigen::Vector3d(1, 0.706, 0));  //黄色
    colorStack.push(Eigen::Vector3d(0, 0.651, 0.929));  //蓝色
    colorStack.push(Eigen::Vector3d(0.929, 0.596, 0.125));  //橙色
    colorStack.push(Eigen::Vector3d(0.929, 0.125, 0.125));  //红色
    colorStack.push(Eigen::Vector3d(0.125, 0.929, 0.125));  //绿色
    return colorStack;
}

int main(int argc, char **argv) {
    //参数：-c是否上色
    bool isColor = false;
    if (argc > 1 && strcmp(argv[1], "-c") == 0) {    //有参数，且第一个参数为-c
        isColor = true;
        argc--; //参数个数减1
        argv++; //参数向前移动1位
    }

    //参数：多个点云文件，pcd或ply
    //无参数，默认使用1.ply和2.ply
    if (argc == 1) {
        cout << "无参数, 默认使用1.ply and 2.ply" << endl;
        argc = 3;
        argv[1] = "ply-data/1.ply";
        argv[2] = "ply-data/2.ply";
    }

    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    // 为点云上上不同的颜色
    stack<Eigen::Vector3d> colorStack;
    if (isColor) {
        colorStack = getColorStack();
    }
    for (int i = 1; i < argc; i++) {
        auto cloud = open3d::io::CreatePointCloudFromFile(argv[i]);
        if (isColor) {
            if (colorStack.empty()) { //栈空，默认黄色
                cloud->PaintUniformColor(Eigen::Vector3d(1, 0.706, 0));
            } else {
                cloud->PaintUniformColor(colorStack.top());
                colorStack.pop();
            }
        }
        geometries.push_back(cloud);
    }
//    auto coord_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame();
//    open3d::visualization::DrawGeometries({cloud, coord_frame});
    open3d::visualization::DrawGeometries(geometries);


    return 0;
}

