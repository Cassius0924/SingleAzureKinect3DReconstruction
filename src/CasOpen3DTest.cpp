//
// Created by HoChihChou on 4/9/23.
//

/*
 * 此文件仅用于测试Open3D库是可用
 * 后面将会删除
 */

#include <string>
#include <open3d/Open3D.h>

int main(int argc, char* argv[]) {
    if (argc == 2) {
        std::string option(argv[1]);
        if (option == "--skip-for-unit-test") {
            open3d::utility::LogInfo("Skiped for unit test.");
            return 0;
        }
    }

    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    sphere->ComputeVertexNormals();
    sphere->PaintUniformColor({ 0.0, 1.0, 0.0 });
    open3d::visualization::DrawGeometries({ sphere });
    return 0;
}