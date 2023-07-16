#include <iostream>
#include <open3d/Open3D.h>
#include <vector>

using namespace std;

int main() {
    // 读取文件
    open3d::geometry::PointCloud final_cloud = *open3d::io::CreatePointCloudFromFile(
        "/home/ncistwlwsys/hezhizhou-projects/SingleAzureKinect3DReconstruction/src/cloud10.pcd");

    // 计算点云的最近邻距离，最近邻距离就是每个点到其最近邻点的距离
    vector<double> distances = final_cloud.ComputeNearestNeighborDistance();
    // 计算平均距离
    double avg_dist = accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    // 设置搜索半径
    double radius = avg_dist * 3;
    vector<double> radii = {radius, radius * 2};
    // 使用球形扫描算法创建三角网格
    final_cloud.RemoveStatisticalOutliers(50, 0.1); // 去除离群点

    // =======Copilot=======
    cout << "BPA..." << endl;
    cout << "当前的radius值为：" << radius << endl;
    auto bpa_mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(final_cloud, radii);
    // 保存三角网格
    open3d::io::WriteTriangleMesh(
        "/home/ncistwlwsys/hezhizhou-projects/SingleAzureKinect3DReconstruction/src/copilot_mesh.ply", *bpa_mesh, true,
        true);

    vector<double> radiis = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1};
    double best_quality = -1;
    open3d::geometry::TriangleMesh best_mesh;
    double best_radius = -1;

    for (double radius : radiis) {
        cout << "当前的radius值为：" << radius << endl;
        auto bpa_mesh2 = *open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(final_cloud, {radius});
        open3d::io::WriteTriangleMesh(
            "/home/ncistwlwsys/hezhizhou-projects/SingleAzureKinect3DReconstruction/src/best_mesh_" +
                to_string(radius) + ".ply",
            bpa_mesh2, true, true);
        // auto quality = bpa_mesh2.GetVolume();
        // if (quality > best_quality) {
        //     best_quality = quality;
        //     best_mesh = bpa_mesh2;
        //     best_radius = radius;
        // }
    }

    // open3d::io::WriteTriangleMesh("/home/ncistwlwsys/hezhizhou-projects/SingleAzureKinect3DReconstruction/src/best_mesh.ply",
    // best_mesh, true, true);

    cout << "最优的radii值为：" << best_radius << endl;
    cout << "最优的三角网格的质量指标为：" << best_quality << endl;
}