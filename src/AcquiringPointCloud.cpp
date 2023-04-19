/*
 * SingleAzureKinect3DReconstruction
 * 此项目基于 Open3D 和 Azure Kinect DK 实现了三维重建。利用 Azure Kinect DK
 * 捕获图像并记录 IMU 数据，利用 Open3D 实现三维重建。
 *
 * Created by Cassius0924 on 2020/03/03.
 * 
 */

#include <chrono>
#include <iostream>
#include <k4a/k4a.hpp>
#include <string>
#include <thread>
#include <vector>

// Open3D
#include <open3d/Open3D.h>

// WebSocket
// #include "CasWebSocket.h"

#include "CasAzureKinectExtrinsics.h"

#define INTERVAL_RADIAN 0.785
#define EXIT_ANGLE 3.2

using namespace std;

int main(int argc, char *argv[]) {
    // 发现已连接的设备数
    const uint32_t device_count = k4a::device::get_installed_count();
    if (0 == device_count) {
        cout << "Error: no K4A devices found. " << endl;
        return -1;
    } else {
        cout << "Found " << device_count << " connected devices. " << endl;
        if (1 != device_count) {// 超过1个设备，也输出错误信息。
            cout << "Error: more than one K4A devices found. " << endl;
            return -1;
        } else {// 该示例代码仅限对1个设备操作
            cout << "Done: found 1 K4A device. " << endl;
        }
    }

    // 打开（默认）设备
    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
    cout << "Done: open device. " << endl;

    // 配置并启动设备
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture
    device.start_cameras(&config);
    cout << "Done: start camera." << endl;

    // 稳定化
    k4a::capture capture;
    int i_auto = 0;      // 用来稳定，类似自动曝光
    int i_auto_error = 0;// 统计自动曝光的失败次数
    while (true) {
        if (device.get_capture(&capture)) {
            cout << i_auto << ". Capture several frames to give auto-exposure" << endl;
            // 跳过前 n 个（成功的数据采集）循环，用来稳定
            if (i_auto != 30) {
                i_auto++;
                continue;
            } else {
                cout << "Done: auto-exposure" << endl;
                break;// 跳出该循环，完成相机的稳定过程
            }
        } else {
            cout << i_auto_error << ". K4A_WAIT_RESULT_TIMEOUT." << endl;
            if (i_auto_error != 30) {
                i_auto_error++;
                continue;
            } else {
                cout << "Error: failed to give auto-exposure. " << endl;
                return -1;
            }
        }
    }
    cout << "-----------------------------------" << endl;
    cout << "----- Have Started Kinect DK. -----" << endl;
    cout << "-----------------------------------" << endl;
    // 从设备获取捕获
    k4a::image rgb_image;
    k4a::image depth_image;
    // k4a::image transformed_depthImage;
    // k4a::image point_cloud_image;

    int color_image_width_pixels = 0;
    int color_image_height_pixels = 0;

    // 创建一个结构体，用来保存rgb_image, depth_image, width, height
    struct ImageData {
        k4a::image rgb_image;
        k4a::image depth_image;
        cas::EulerAngle angle;
    };

    // 创建容器用于保存ImageData
    vector<ImageData> image_data_vector;

    // 先创建个WebSocket对象
    // 服务器地址：ws://175.178.56.40:8080/ws-test
    // WebSocket webSocket("175.178.56.40","8080","/ws-test");
    // // WebSocket webSocket("ws://39.108.216.190/ws");
    // //连接服务器
    // if (webSocket.connect()) {
    //    cout << "connect success" << endl;
    // } else {
    //    cout << "connect failed" << endl;
    // }

    // 启动 IMU
    k4a_imu_sample_t imu_sample;
    device.start_imu();

    //    float prev_roll = 100;
    //    float prev_pitch = 100;
    float prev_yaw = 100;

    // 定义欧拉角
    cas::EulerAngle prev_angle(0, 0, 0);
    // 定义最终的点云
    auto final_cloud = make_shared<open3d::geometry::PointCloud>();
    float temp = 0;
    int flag = 0;

    // 定义互斥锁
    mutex camera_mutex;
    mutex cloud_mutex;
    // 定义条件变量
    condition_variable camera_cv;
    condition_variable cloud_cv;
    // 定义共享变量
    bool ready_to_break = false;
    bool need_break = false;

    int camera_task_count = 0;
    int cloud_task_count = 0;

    cas::EulerAngle cur_angle(0, 0, 0);

    // 定义相机线程
    thread camera_thread([&]() {
        while (true) {
            // 获取互斥锁
            unique_lock<mutex> lock(camera_mutex);

            // 等待条件变量
            while (camera_task_count <= 0) {// 当任务数大于0时，才开始执行
                camera_cv.wait(lock);
            }

            ImageData image_data;

            device.get_capture(&capture);

            // 存入结构体
            image_data.angle = prev_angle;// 获取当前时刻的欧拉角
            image_data.rgb_image = capture.get_color_image();
            image_data.depth_image = capture.get_depth_image();

            // 存入容器
            image_data_vector.push_back(image_data);

            cloud_task_count++;
            if (camera_task_count > 0) {
                camera_task_count--;
            }

            cout << "通知计算线程: " << cloud_task_count << endl;
            cloud_cv.notify_one();
        }
    });

    // 点云计算线程
    thread cloud_thread([&]() {
        while (true) {
            unique_lock<mutex> lock(cloud_mutex);

            while (cloud_task_count <= 0) {
                cloud_cv.wait(lock);
            }

            cout << "计算线程开始计算: " << cloud_task_count << endl;

            // 从容器中取出数据
            ImageData image_data = image_data_vector[0];
            image_data_vector.erase(image_data_vector.begin());// 删除第一个元素

            k4a::image rgb_image_item = image_data.rgb_image;
            k4a::image depth_image_item = image_data.depth_image;

            int color_image_width_pixels = rgb_image_item.get_width_pixels();
            int color_image_height_pixels = rgb_image_item.get_height_pixels();

            k4a::calibration k4a_calibration = device.get_calibration(config.depth_mode, config.color_resolution);
            k4a::transformation k4a_transformation = k4a::transformation(k4a_calibration);

            k4a::image transformed_depthImage = k4a::image::create(
                    K4A_IMAGE_FORMAT_DEPTH16, color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * (int) sizeof(uint16_t));
            k4a::image point_cloud_image = k4a::image::create(
                    K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * 3 * (int) sizeof(int16_t));

            // if (depth_image.get_width_pixels() == rgb_image_item.get_width_pixels() &&
            //     depth_image.get_height_pixels() == rgb_image_item.get_height_pixels()) {
            //     copy(depth_image.get_buffer(),
            //          depth_image.get_buffer() + depth_image.get_height_pixels() *
            //                                             depth_image.get_width_pixels() *
            //                                             (int) sizeof(uint16_t),
            //          transformed_depthImage.get_buffer());
            //     cout << "if" << endl;
            // } else {
            //     cout << "else" << endl;
            // k4a_transformation.depth_image_to_color_camera(depth_image_item, &transformed_depthImage);
            // }

            k4a_transformation.depth_image_to_color_camera(depth_image_item, &transformed_depthImage);

            k4a_transformation.depth_image_to_point_cloud(transformed_depthImage, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

            auto cloud = make_shared<open3d::geometry::PointCloud>();

            const int16_t *point_cloud_image_data = reinterpret_cast<const int16_t *>(point_cloud_image.get_buffer());
            const uint8_t *color_image_data = rgb_image_item.get_buffer();

            for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++) {
                if (point_cloud_image_data[3 * i + 0] != 0 &&
                    point_cloud_image_data[3 * i + 1] != 0 &&
                    point_cloud_image_data[3 * i + 2] != 0) {
                    cloud->points_.push_back(
                            Eigen::Vector3d(point_cloud_image_data[3 * i + 0] / 1000.0f,
                                            point_cloud_image_data[3 * i + 1] / 1000.0f,
                                            point_cloud_image_data[3 * i + 2] / 1000.0f));
                    cloud->colors_.push_back(
                            Eigen::Vector3d(color_image_data[4 * i + 2] / 255.0f,
                                            color_image_data[4 * i + 1] / 255.0f,
                                            color_image_data[4 * i + 0] / 255.0f));
                } else {
                    cloud->points_.push_back(Eigen::Vector3d(0, 0, 0));
                    cloud->colors_.push_back(Eigen::Vector3d(0, 0, 0));
                }
            }

            // Eigen::Matrix3d rotation_matrix;
            // rotation_matrix << cas::eulerAngle2RotationMatrix(image_data.angle);

            //# 点到面的ICP
            // current_transformation = np.identity(4)
            // print("2. 在原始点云上应用点到平面ICP配准来精准对齐，距离阈值0.02。")
            // result_icp = o3d.pipelines.registration.registration_icp(source, target, 0.02, current_transformation,
            //                                                          o3d.pipelines.registration.TransformationEstimationPointToPlane())
            // print(result_icp)
            // draw_registration_result_original_color(source, target, result_icp.transformation)
            // 改为C++实现
            if (flag == 0) {
                flag == 1;
            } else {
                Eigen::Matrix4d transformation_icp = Eigen::Matrix4d::Identity();
                auto result_icp = open3d::pipelines::registration::RegistrationICP(*final_cloud, *cloud, 0.02, transformation_icp, open3d::pipelines::registration::TransformationEstimationPointToPlane());
                final_cloud->Transform(result_icp.transformation_);


                // // Open3D实现彩色ICP配准
                // open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance checker(0.05);
                // auto correspondence_set = open3d::pipelines::registration::Compute
                //         open3d::registration::Feature feature_source,
                //      feature_target;
                // feature_source = open3d::registration::ComputePointFeatureFromXYZRGB(*final_cloud);
                // feature_target = open3d::registration::ComputePointFeatureFromXYZRGB(*cloud);
                // open3d::pipelines::registration::TransformationEstimationPointToPointWithNormals estimation(true);
                // estimation.estimate(*source, *target, feature_source, feature_target, correspondence_set);
                // auto transformation_icp = estimation.getInformation().transformation_;

                // final_cloud->Transform(transformation_icp);
            }


            // if (flag == 0) {
            //     flag = 1;
            // } else {
            //     Eigen::Vector3d center << 0, 0, 0;
            //     cloud->Rotate(rotation_matrix, center);
            // }
            // cloud = cloud->VoxelDownSample(0.01);
            *final_cloud += *cloud;

            cloud_task_count--;
            cout << "计算完毕" << endl;
            if (ready_to_break && cloud_task_count <= 0) {
                need_break = true;
            }

            // need_cal_cloud = false;
            // TODO: 将捕获的图像存到容器中，新建一个线程，将容器中的图像逐份处理
            // 由于图像处理比较耗时，所以需要新建一个线程来处理图像

            // TODO: 实时显示点云
            // TODO: 消除冗余点。点云相加后会导致很多点重合，需要消除冗余点。

            //==========================================
            // 先把点云数据序列化成字节数组
            // ofstream ofs("pcd_oarchive/" + filename_pc + ".pcd");
            // boost::archive::text_oarchive oa(ofs);
            // oa << *cloud_filtered;
            // ofs.close();
            // ifstream ifs("pcd/" + filename_pc + ".pcd");
            // stringstream ss;
            // ss << ifs.rdbuf();
            // ifs.close();
            // string serialized_data = ss.str();
            // webSocket.sendPointCloud(serialized_data);
            //==========================================
            // }
        }
    });

    // 主线程获取IMU数据
    while (true) {
        if (device.get_imu_sample(&imu_sample, chrono::milliseconds(0))) {// 获取当前的IMU数据
            // float gx = imu_sample.gyro_sample.xyz.x;
            // float gy = imu_sample.gyro_sample.xyz.y;
            float gx = 0;
            float gy = 0;
            float gz = imu_sample.gyro_sample.xyz.z;

            float dtf = (float) (imu_sample.acc_timestamp_usec - temp) / 1000000;
            if (temp == 0) {
                temp = (float) imu_sample.acc_timestamp_usec;
                continue;
            }
            temp = (float) imu_sample.acc_timestamp_usec;

            // 计算欧拉角，先绕x轴旋转roll，再绕y轴旋转pitch，最后绕z轴旋转yaw
            prev_angle = calculateOrientation(gx, gy, gz, dtf, prev_angle);

            if (abs(prev_angle.yaw - prev_yaw) > INTERVAL_RADIAN) {
                // 启动相机
                // prev_roll = prev_angle.roll;
                // prev_pitch = prev_angle.pitch;
                prev_yaw = prev_angle.yaw;
                cout << "saving..." << endl;
                // 修改共享变量
                camera_task_count++;
                cout << "通知相机线程:" << camera_task_count << endl;
                camera_cv.notify_one();
            }
        }

        // 如果旋转角度大于π，就退出
        if (abs(prev_angle.yaw) > EXIT_ANGLE) {
            cout << "旋转角度大于π，退出" << endl;
            ready_to_break = true;

            cout << "等待线程结束..." << endl;
            while (true) {
                if (need_break) {
                    break;
                }
            }
            final_cloud->VoxelDownSample(0.01);
            open3d::io::WritePointCloud("ply/final_cloud.ply", *final_cloud);
            break;
        }
    }

    cout << "程序结束" << endl;
    // 释放，关闭设备
    rgb_image.reset();
    depth_image.reset();
    capture.reset();
    device.close();

    return 1;
}
