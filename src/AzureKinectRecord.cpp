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
#include <k4a/k4a.hpp>
#include <math.h>

#include <atomic>
#include <csignal>
#include <ctime>
#include <iostream>

#include "open3d/Open3D.h"
#include "CasAzureKinectExtrinsics.h"


using namespace open3d;

void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > AzureKinectRecord [options]");
    utility::LogInfo("Basic options:");
    utility::LogInfo("    --help, -h                : Print help information.");
    utility::LogInfo("    --config                  : Config .json file (default: none)");
    utility::LogInfo("    --list                    : List the currently connected K4A devices");
    utility::LogInfo("    --device                  : Specify the device index to use (default: 0)");
    utility::LogInfo("    --output                  : Output mkv file name (default: current_time.mkv)");
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

    io::AzureKinectSensorConfig sensor_config;
    if (utility::ProgramOptionExists(argc, argv, "--config")) {
        auto config_filename =
                utility::GetProgramOptionAsString(argc, argv, "--config", "");
        bool success = io::ReadIJsonConvertibleFromJSON(config_filename,
                                                        sensor_config);
        if (!success) {
            utility::LogInfo("Invalid sensor config");
            return 1;
        }
    } else {
        utility::LogInfo("Use default sensor config");
    }

    // 获取传感器索引
    int sensor_index =
            utility::GetProgramOptionAsInt(argc, argv, "--device", 0);
    if (sensor_index < 0 || sensor_index > 255) {
        utility::LogWarning("Sensor index must between [0, 255]: {}",
                            sensor_index);
        return 1;
    }

    // 获取是否对齐深度图
    bool enable_align_depth_to_color =
            utility::ProgramOptionExists(argc, argv, "-a");

    // 获取输出文件名
    std::string recording_filename = utility::GetProgramOptionAsString(
            argc, argv, "--output", utility::GetCurrentTimeStamp() + ".mkv");
    utility::LogInfo("Prepare writing to {}", recording_filename);

    // Init recorder
    io::AzureKinectRecorder recorder(sensor_config, sensor_index);
    if (!recorder.InitSensor()) {
        utility::LogWarning("Failed to connect to sensor, abort.");
        return 1;
    }

    bool flag_record = false;
    bool flag_exit = false;
    bool is_geometry_added = false;
    visualization::VisualizerWithKeyCallback vis;
    vis.RegisterKeyCallback(
            GLFW_KEY_SPACE, [&](visualization::Visualizer *vis) {
                if (flag_record) {
                    utility::LogInfo(
                            "Recording paused. "
                            "Press [SPACE] to continue. "
                            "Press [ESC] to save and exit.");
                    flag_record = false;
                } else if (!recorder.IsRecordCreated()) {
                    if (recorder.OpenRecord(recording_filename)) {
                        utility::LogInfo(
                                "Recording started. "
                                "Press [SPACE] to pause. "
                                "Press [ESC] to save and exit.");
                        flag_record = true;
                    }  // else flag_record keeps false
                } else {
                    utility::LogInfo(
                            "Recording resumed, video may be discontinuous. "
                            "Press [SPACE] to pause. "
                            "Press [ESC] to save and exit.");
                    flag_record = true;
                }
                return false;
            });

    vis.RegisterKeyCallback(
            GLFW_KEY_ESCAPE, [&](visualization::Visualizer *vis) {
                flag_exit = true;
                if (recorder.IsRecordCreated()) {
                    utility::LogInfo("Recording finished.");
                } else {
                    utility::LogInfo("Nothing has been recorded.");
                }
                return false;
            });

    utility::LogInfo(
            "In the visualizer window, "
            "press [SPACE] to start recording, "
            "press [ESC] to exit.");

    vis.CreateVisualizerWindow("Open3D Azure Kinect Recorder", 1920, 540);



    // 初始化Azure Kinect设备
//    k4a::device device = cas::openAzureKinectDevice();
//
//    // 配置Azure Kinect设备
//    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;       //NFOV
//    config.color_resolution = K4A_COLOR_RESOLUTION_720P;    //720P
//    config.camera_fps = K4A_FRAMES_PER_SECOND_30;           //30FPS
//    cas::configureAzureKinectDevice(device, config);
//
//    device.start_imu();
//
//    uint64_t dt;
//    uint64_t temp = 0;
//
//    cas::EulerAngle prevAngle(0, 0, 0);


    do {
        //捕获一针
//        k4a_capture_t capture;
//        k4a_device_get_capture(device.handle(), &capture, K4A_WAIT_INFINITE);
//
//        k4a_imu_sample_t imu_sample;
//        device.get_imu_sample(&imu_sample);
//
//        float gx = imu_sample.gyro_sample.xyz.x;
//        float gy = imu_sample.gyro_sample.xyz.y;
//        float gz = imu_sample.gyro_sample.xyz.z;
//
//        dt = (imu_sample.acc_timestamp_usec - temp);
//        float dtf = (float) dt / 1000000;
//        temp = imu_sample.acc_timestamp_usec;
//        if (temp == 0) {
//            continue;
//        }
//        prevAngle = calculateOrientation(gx, gy, gz, dtf, prevAngle);
//        printf("roll:%.4f\tpitch:%.4f\tyaw:%.4f\tdt:%f\n", prevAngle.roll, prevAngle.pitch, prevAngle.yaw, dtf);
//

        auto im_rgbd = recorder.RecordFrame(flag_record, enable_align_depth_to_color);
        if (im_rgbd == nullptr) {
            utility::LogDebug("Invalid capture, skipping this frame");
            continue;
        }

        if (!is_geometry_added) {
            vis.AddGeometry(im_rgbd);
            is_geometry_added = true;
        }

        // Update visualizer
        vis.UpdateGeometry();
        vis.PollEvents();
        vis.UpdateRender();
    } while (!flag_exit);

    recorder.CloseRecord();

    return 0;
}

