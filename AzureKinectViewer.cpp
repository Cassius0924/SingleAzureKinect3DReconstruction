// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <assert.h>
#include <k4a/k4a.h>
#include <math.h>

#include <atomic>
#include <csignal>
#include <ctime>
#include <iostream>

#include "open3d/Open3D.h"

using namespace open3d;

void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > AzureKinectViewer [options]");
    utility::LogInfo("Basic options:");
    utility::LogInfo("    --help, -h                : Print help information.");
    utility::LogInfo("    --config                  : Config .json file (default: none)");
    utility::LogInfo("    --list                    : List the currently connected K4A devices");
    utility::LogInfo("    --device                  : Specify the device index to use (default: 0)");
    utility::LogInfo("    -a                        : Align depth with color image (default: disabled)");
    // clang-format on
    utility::LogInfo("");
}

int main(int argc, char **argv) {
    if (argc < 1 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    if (utility::ProgramOptionExists(argc, argv, "--list")) {
        io::AzureKinectSensor::ListDevices();
        return 0;
    }

    // 读取传感器配置
    io::AzureKinectSensorConfig sensor_config;
    // 判断是否指定了传感器配置文件
    if (utility::ProgramOptionExists(argc, argv, "--config")) {
        auto config_filename =
                utility::GetProgramOptionAsString(argc, argv, "--config", "");
        if (!io::ReadIJsonConvertibleFromJSON(config_filename, sensor_config)) {
            utility::LogInfo("Invalid sensor config");
            return 1;
        }
    } else {
        utility::LogInfo("Use default sensor config");
    }

    // 获取传感器索引
    int sensor_index = utility::GetProgramOptionAsInt(argc, argv, "--device", 0);

    // 判断传感器索引是否合法
    if (sensor_index < 0 || sensor_index > 255) {
        utility::LogWarning("Sensor index must between [0, 255]: {}",
                            sensor_index);
        return 1;
    }

    // 判断是否对齐深度图
    bool enable_align_depth_to_color = utility::ProgramOptionExists(argc, argv, "-a");

    // Init sensor
    // 初始化传感器
    io::AzureKinectSensor sensor(sensor_config);
    if (!sensor.Connect(sensor_index)) {
        utility::LogWarning("Failed to connect to sensor, abort.");
        return 1;
    }

    // Start viewing
    // 开始查看
    bool flag_exit = false; // 退出标志
    bool is_geometry_added = false; // 是否添加几何体
    visualization::VisualizerWithKeyCallback vis;   // 可视化窗口
    // 注册按键回调函数：按下ESC键退出
    vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
                            [&](visualization::Visualizer *vis) {
                                flag_exit = true;
                                return false;
                            });

    // 创建可视化窗口
    vis.CreateVisualizerWindow("Open3D Azure Kinect Recorder", 1920, 540);

    // 循环获取图像
    do {
        // 捕获一帧图像
        auto im_rgbd = sensor.CaptureFrame(enable_align_depth_to_color);
        // 判断图像是否有效
        if (im_rgbd == nullptr) {
            utility::LogInfo("Invalid capture, skipping this frame");
            continue;
        }

        // 添加几何体
        if (!is_geometry_added) {   // 如果没有添加几何体
            vis.AddGeometry(im_rgbd);   // 添加几何体：图像
            is_geometry_added = true;   // 设置已添加几何体
        }

        // Update visualizer
        // 更新可视化窗口
        vis.UpdateGeometry();   // 更新几何体
        vis.PollEvents();   // 处理事件
        vis.UpdateRender(); // 更新渲染

    } while (!flag_exit);

    return 0;
}
