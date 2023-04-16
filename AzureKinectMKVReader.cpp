// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

//#include <json/json.h>

#include <chrono>
#include <fstream>
#include <thread>

#include "open3d/Open3D.h"
//#include "open3d/3rdparty/matdbg/JsonWriter.h"

using namespace open3d;

/*
 * 将json写入文件
 */
//void WriteJsonToFile(const std::string &filename, const Json::Value &value) {
//    std::ofstream out(filename);
//    if (!out.is_open()) {
//        utility::LogError("Cannot write to {}", filename);
//    }
//
//    Json::StreamWriterBuilder builder;
//    builder["commentStyle"] = "None";
//    builder["indentation"] = "\t";
//    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
//    writer->write(value, &out);
//}

/*
 * 生成dataset的config.json
 */
//Json::Value GenerateDatasetConfig(const std::string &output_path) {
//    Json::Value value;
//
//    utility::LogInfo("Writing to config.json");
//    utility::LogInfo(
//            "Please change path_dataset and path_intrinsic when you move the "
//            "dataset.");
//
//    if (output_path[0] == '/') {  // global dir
//        value["path_dataset"] = output_path;
//        value["path_intrinsic"] = output_path + "/intrinsic.json";
//    } else {  // relative dir
//        auto pwd = utility::filesystem::GetWorkingDirectory();
//        value["path_dataset"] = pwd + "/" + output_path;
//        value["path_intrinsic"] = pwd + "/" + output_path + "/intrinsic.json";
//    }
//
//    value["name"] = "Azure Kinect Record";
//    value["depth_max"] = 3.0;
//    value["voxel_size"] = 0.05;
//    value["depth_diff_max"] = 0.07;
//    value["preference_loop_closure_odometry"] = 0.1;
//    value["preference_loop_closure_registration"] = 5.0;
//    value["tsdf_cubic_size"] = 3.0;
//    value["icp_method"] = "color";
//    value["global_registration"] = "ransac";
//    value["python_multi_threading"] = true;
//
//    return value;
//}

void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > AzureKinectMKVReader --input input.mkv [--output] [path]");
    // clang-format on
    utility::LogInfo("");
}


int main(int argc, char **argv) {
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc == 1 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"}) ||
        !utility::ProgramOptionExists(argc, argv, "--input")) {
        PrintHelp();
        return 1;
    }

    std::string mkv_filename =
            utility::GetProgramOptionAsString(argc, argv, "--input");

    // 是否写入图片
    bool write_image = false;
    // 图片输出路径
    std::string output_path;
    // 如果有输出路径，则写入图片
    if (!utility::ProgramOptionExists(argc, argv, "--output")) {
        utility::LogInfo("No output image path, only play mkv.");
    } else {
        output_path = utility::GetProgramOptionAsString(argc, argv, "--output");
        if (output_path.empty()) {  // 路径为空
            utility::LogError("Output path {} is empty, only play mkv.",
                              output_path);
            return 1;
        }
        if (utility::filesystem::DirectoryExists(output_path)) {    // 路径已存在
            utility::LogError("Output path {} already existing, only play mkv.",
                              output_path);
            return 1;
        } else if (!utility::filesystem::MakeDirectory(output_path)) {  // 路径创建失败
            utility::LogError("Unable to create path {}, only play mkv.",
                              output_path);
            return 1;
        } else {
            utility::LogInfo("Decompress images to {}", output_path);
            utility::filesystem::MakeDirectoryHierarchy(output_path + "/color");
            utility::filesystem::MakeDirectoryHierarchy(output_path + "/depth");
            utility::filesystem::MakeDirectoryHierarchy(output_path + "/pointcloud");
            write_image = true;
        }
    }

    // 读取mkv文件
    io::MKVReader mkv_reader;
    // 打开mkv文件
    mkv_reader.Open(mkv_filename);
    if (!mkv_reader.IsOpened()) {   // 打开失败
        utility::LogError("Unable to open {}", mkv_filename);
        return 1;
    }

    // 退出标志
    bool flag_exit = false;
    // 播放标志
    bool flag_play = true;
    visualization::VisualizerWithKeyCallback vis;
    // 注册按键：ESC
    vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
                            [&](visualization::Visualizer *vis) {
                                flag_exit = true;
                                return true;
                            });
    // 注册按键：空格
    vis.RegisterKeyCallback(
            GLFW_KEY_SPACE, [&](visualization::Visualizer *vis) {
                if (flag_play) {
                    utility::LogInfo(
                            "Playback paused, press [SPACE] to continue");
                } else {
                    utility::LogInfo(
                            "Playback resumed, press [SPACE] to pause");
                }
                flag_play = !flag_play;
                return true;
            });

    // 创建窗口
    vis.CreateVisualizerWindow("Open3D Azure Kinect MKV player", 1920, 540);
    utility::LogInfo(
            "Starting to play. Press [SPACE] to pause. Press [ESC] to "
            "exit.");

    // 是否添加了几何体
    bool is_geometry_added = false;
    int idx = 0;
    // 如果需要写入图片，则写入intrinsic.json和config.json
//    if (write_image) {
//        io::WriteIJsonConvertibleToJSON(
//                fmt::format("{}/intrinsic.json", output_path),
//                mkv_reader.GetMetadata());
//        WriteJsonToFile(fmt::format("{}/config.json", output_path),
//                        GenerateDatasetConfig(output_path));
//    }
    // 循环读取mkv文件
    while (!mkv_reader.IsEOF() && !flag_exit) {
        // 如果播放标志为true
        if (flag_play) {
            // 读取下一帧
            auto im_rgbd = mkv_reader.NextFrame();
            // 如果读取失败，则跳过
            if (im_rgbd == nullptr) continue;

            // 如果没有添加几何体，则添加
            if (!is_geometry_added) {
                vis.AddGeometry(im_rgbd);
                is_geometry_added = true;
            }

            // 如果需要写入图片，则写入图片
            if (write_image) {
                // 写入color图片
                auto color_file = fmt::format("{0}/color/{1:05d}.jpg", output_path, idx);
                utility::LogInfo("Writing to {}", color_file);
                io::WriteImage(color_file, im_rgbd->color_);

                // 写入depth图片
                auto depth_file = fmt::format("{0}/depth/{1:05d}.png", output_path, idx);
                utility::LogInfo("Writing to {}", depth_file);
                io::WriteImage(depth_file, im_rgbd->depth_);

                // 转点云
                auto rgbd_image = geometry::RGBDImage::CreateFromColorAndDepth(im_rgbd->color_, im_rgbd->depth_);
                auto pcd = geometry::PointCloud::CreateFromRGBDImage(*rgbd_image, camera::PinholeCameraIntrinsic(camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault));

                // 写入点云
                auto pcd_file = fmt::format("{0}/pointcloud/{1:05d}.pcd", output_path, idx);
                utility::LogInfo("Writing to {}", pcd_file);
                io::WritePointCloud(pcd_file, *pcd);

                ++idx;
            }
        }

        // 更新窗口
        vis.UpdateGeometry();
        vis.PollEvents();
        vis.UpdateRender();
    }

    mkv_reader.Close();
}
