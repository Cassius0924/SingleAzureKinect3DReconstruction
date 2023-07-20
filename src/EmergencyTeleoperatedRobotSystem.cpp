// Created by Cassius0924 on 2020/03/03.
//

/*
 * SingleAzureKinect3DReconstruction
 * 此项目基于 Open3D 和 Azure Kinect DK 实现了三维重建。利用 Azure Kinect DK
 * 捕获图像并记录 IMU 数据，利用 Open3D 实现三维重建。
 */

#include "CasAzureKinect.h"
#include "CasAzureKinectExtrinsics.h"
#include "CasBot.h"
#include "CasConfig.h"
#include "CasNetwork.h"
#include "CasSoundSourceLocalization.h"
#include "DataMessage.pb.h"

#include <iostream>
#include <k4a/k4a.hpp>
#include <string>

#include <chrono>
#include <thread>
#include <vector>

#include <open3d/Open3D.h>

using namespace std;
using namespace open3d;

int main(int argc, char **argv) { // TODO: 可以传参，传入配置文件路径
    // 读取配置文件
    cas::CasConfig program_config("../default.conf");

    float INTERVAL_RADIAN = program_config.get_float("interval_radian");
    float EXIT_RADIAN = program_config.get_float("exit_radian");
    float VOXEL_SIZE = program_config.get_float("voxel_size");
    float FINAL_VOXEL_SIZE = program_config.get_float("final_voxel_size");
    float SMALL_RADIUS_MULTIPLIER = program_config.get_float("small_radius_multiplier");
    float LARGE_RADIUS_MULTIPLIER = program_config.get_float("large_radius_multiplier");
    // string WEB_SOCKET_SERVER_ADDRESS = program_config.get("web_socket_server_address");
    // int WEB_SOCKET_SERVER_PORT = program_config.get_int("web_socket_server_port");
    // string WEB_SOCKET_SERVER_PATH = program_config.get("web_socket_server_path");
    bool IS_WRITE_FILE = program_config.get_bool("is_write_file");
    string CLOUD_FILE_PATH = program_config.get("cloud_file_path");
    string MESH_FILE_PATH = program_config.get("mesh_file_path");
    bool IS_CREATE_SERVER = program_config.get_bool("is_create_server");
    bool IS_CONNECT_ARM = program_config.get_bool("is_connect_arm");
    bool IS_CONNECT_KINECT = program_config.get_bool("is_connect_kinect");
    bool ENABLE_SOUND_SOURCE_LOCALIZATION = program_config.get_bool("enable_sound_source_localization");
    int SERVER_PORT = program_config.get_int("protobuf_server_port");
    unsigned int SAMPLE_RATE = program_config.get_int("sample_rate");
    int SAMPLES = program_config.get_int("samples");
    int CHANNELS = program_config.get_int("channels");
    string MICROPHONE_NAME = program_config.get("microphone_name");
    string BOT_ARM_SERIAL_PORT_NAME = program_config.get("bot_arm_serial_prot_name");
    string BOT_CAR_SERIAL_PORT_NAME = program_config.get("bot_car_serial_prot_name");
    string STM32_SERIAL_PORT_NAME = program_config.get("stm32_serial_prot_name");
    string FIRST_MOTOR_ROTATION = program_config.get("first_motor_rotation");
    float TEMPERATURE_THRESHOLD = program_config.get_float("temperature_threshold");
    float HUMIDITY_THRESHOLD = program_config.get_float("humidity_threshold");
    string RECORDINGS_FOLDER_PATH = program_config.get("recordings_folder_path");

    k4a::device device;
    k4a::capture capture;
    k4a_device_configuration_t config;

    cas::bot::BotArm bot_arm(BOT_ARM_SERIAL_PORT_NAME);
    cas::bot::BotMotor bot_motor(STM32_SERIAL_PORT_NAME);
    cas::bot::BotCar bot_car(BOT_CAR_SERIAL_PORT_NAME, (char)0x12, 0.6);

    // 发现已连接的设备数
    if (cas::kinect::checkKinectNum(1) == false) {
        return 0;
    }

    // 打开（默认）设备
    // device = k4a::device::open(K4A_DEVICE_DEFAULT);
    // cout << "打开设备。" << endl;

    // 配置并启动设备
    // config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    // config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    // config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    // config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    // config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    // config.synchronized_images_only = true; // 只输出同步的图像，即同步深度图和彩色图
    // device.start_cameras(&config);
    // cout << "开启相机。" << endl;

    // 稳定化
    // cas::kinect::stabilizeCamera(device);
    // cout << "------------------------------------" << endl;
    // cout << "----- 成功启动 Azure Kinect DK -----" << endl;
    // cout << "------------------------------------" << endl;

    if (IS_CONNECT_ARM) {
        bot_arm.reset();
        cout << "机械臂复位成功" << endl;
    }

    // 创建服务器等待连接
    cas::net::Client client(SERVER_PORT);

    // 判断旋转方向
    // if (bot_motor.rotate(FIRST_MOTOR_ROTATION)) {
    //     cout << "舵机旋转成功" << endl;
    //     if (FIRST_MOTOR_ROTATION == "F") {
    //         FIRST_MOTOR_ROTATION = "R";
    //         program_config.set("first_motor_rotation", "R");
    //     } else if (FIRST_MOTOR_ROTATION == "R") {
    //         FIRST_MOTOR_ROTATION = "F";
    //         program_config.set("first_motor_rotation", "F");
    //     } else {
    //         cerr << "未知的舵机旋转方向！" << endl;
    //     }
    // }

    // 定义互斥锁
    mutex arm_mutex;
    mutex client_mutex;
    mutex stm32_mutex;
    mutex car_mutex;
    mutex timer_mutex;

    // 定义条件变量
    condition_variable arm_cv;
    condition_variable stm32_cv;
    condition_variable client_cv;
    condition_variable timer_cv;
    // 定义共享变量
    bool ready_to_break = false;
    bool need_break = false;

    bool need_reconnstrcution = true;
    int camera_task_count = 0;
    int cloud_task_count = 0;
    bool arm_finished = false;

    bool flag_recording = false;

    cas::EulerAngle cur_angle(0, 0, 0);

    // thread timer_thread([&]() {
    //     unique_lock<mutex> lock(timer_mutex);
    //     while (true) {
    //         timer_cv.wait(lock);
    //     }
    //     this_thread::sleep_for(std::chrono::milliseconds(5000));
    //     flag_recording = true;
    // });

    // 定义相机线程
    // thread camera_thread([&]() {
    //     while (true) {
    //         // 获取互斥锁
    //         unique_lock<mutex> lock(camera_mutex);

    //         // 等待条件变量
    //         while (camera_task_count <= 0) { // 当任务数大于0时，才开始执行
    //             camera_cv.wait(lock);
    //         }

    //         ImageData image_data;
    //         device.get_capture(&capture);

    //         // 存入结构体
    //         image_data.angle = prev_angle; // 获取当前时刻的欧拉角
    //         image_data.rgb_image = capture.get_color_image();
    //         image_data.depth_image = capture.get_depth_image();

    //         // 存入容器
    //         {
    //             lock_guard<mutex> lock(image_data_mutex);
    //             image_data_vector.push_back(image_data);
    //         }

    //         cloud_task_count++;
    //         if (camera_task_count > 0) {
    //             camera_task_count--;
    //         }

    //         cout << "通知计算线程: " << cloud_task_count << endl;
    //         cloud_cv.notify_one();
    //     }
    // });

    // 点云计算线程
    // thread cloud_thread([&]() {
    //     open3d::geometry::PointCloud cloud;
    //     k4a::calibration k4a_calibration = device.get_calibration(config.depth_mode, config.color_resolution);
    //     k4a::transformation k4a_transformation = k4a::transformation(k4a_calibration);
    //     Eigen::Matrix4d transformation_icp = Eigen::Matrix4d::Identity();
    //     float relative_fitness = 1e-6;
    //     float relative_rmse = 1e-6;
    //     int max_iteration = 30;
    //     open3d::pipelines::registration::ICPConvergenceCriteria icp_criteria(relative_fitness, relative_rmse,
    //                                                                          max_iteration);
    //     open3d::geometry::KDTreeSearchParamHybrid kd_tree_param(VOXEL_SIZE * 2, 30);

    //     while (true) {
    //         unique_lock<mutex> lock(cloud_mutex);

    //         while (cloud_task_count <= 0) {
    //             cloud_cv.wait(lock);
    //         }

    //         cout << "计算线程开始计算: " << cloud_task_count << endl;

    //         // 从容器中取出数据
    //         ImageData image_data;
    //         {
    //             lock_guard<mutex> lock(image_data_mutex);
    //             if (!image_data_vector.empty()) {
    //                 image_data = image_data_vector.front();
    //                 image_data_vector.erase(image_data_vector.begin());
    //             }
    //         }

    //         k4a::image rgb_image_item = image_data.rgb_image;
    //         k4a::image depth_image_item = image_data.depth_image;

    //         int color_image_width_pixels = rgb_image_item.get_width_pixels();
    //         int color_image_height_pixels = rgb_image_item.get_height_pixels();

    //         k4a::image transformed_depthImage =
    //             k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, color_image_width_pixels, color_image_height_pixels,
    //                                color_image_width_pixels * (int)sizeof(uint16_t));
    //         k4a::image point_cloud_image =
    //             k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels, color_image_height_pixels,
    //                                color_image_width_pixels * 3 * (int)sizeof(int16_t));

    //         k4a_transformation.depth_image_to_color_camera(depth_image_item, &transformed_depthImage);

    //         k4a_transformation.depth_image_to_point_cloud(transformed_depthImage, K4A_CALIBRATION_TYPE_COLOR,
    //                                                       &point_cloud_image);

    //         cloud.points_.resize(color_image_width_pixels * color_image_height_pixels);
    //         cloud.colors_.resize(color_image_width_pixels * color_image_height_pixels);

    //         const int16_t *point_cloud_image_data = reinterpret_cast<const int16_t
    //         *>(point_cloud_image.get_buffer()); const uint8_t *color_image_data = rgb_image_item.get_buffer();

    //         for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++) {
    //             if (point_cloud_image_data[3 * i + 0] != 0 && point_cloud_image_data[3 * i + 1] != 0 &&
    //                 point_cloud_image_data[3 * i + 2] != 0) {
    //                 cloud.points_[i] = Eigen::Vector3d(point_cloud_image_data[3 * i + 0] / 1000.0f,
    //                                                    point_cloud_image_data[3 * i + 1] / 1000.0f,
    //                                                    point_cloud_image_data[3 * i + 2] / 1000.0f);
    //                 cloud.colors_[i] =
    //                     Eigen::Vector3d(color_image_data[4 * i + 2] / 255.0f, color_image_data[4 * i + 1] /
    //                     255.0f,
    //                                     color_image_data[4 * i + 0] / 255.0f);
    //             } else {
    //                 cloud.points_[i] = Eigen::Vector3d::Zero();
    //                 cloud.colors_[i] = Eigen::Vector3d::Zero();
    //             }
    //         }

    //         Eigen::Matrix3d rotation_matrix;
    //         rotation_matrix << cas::eulerAngle2RotationMatrix(image_data.angle);

    //         // if (VOXEL_SIZE > 0) {
    //         cloud = *cloud.VoxelDownSample(VOXEL_SIZE);
    //         // }

    //         if (registration_flag == 0) {
    //             registration_flag = 1;
    //         } else {
    //             // 粗配准: 基于 IMU 数据的旋转矩阵
    //             Eigen::Vector3d center = Eigen::Vector3d::Zero();
    //             cloud.Rotate(rotation_matrix, center);

    //             // 精配准: 基于彩色 ICP 算法。
    //             final_cloud.EstimateNormals(kd_tree_param); // 估计法向量
    //             cloud.EstimateNormals(kd_tree_param);
    //             auto result_icp = open3d::pipelines::registration::RegistrationICP(
    //                 cloud, final_cloud, VOXEL_SIZE * 2, transformation_icp,
    //                 open3d::pipelines::registration::TransformationEstimationPointToPlane(), icp_criteria);
    //             cloud.Transform(result_icp.transformation_);
    //         }

    //         final_cloud += cloud;

    //         cloud_task_count--;
    //         cout << "计算完毕" << endl;
    //         if (ready_to_break && cloud_task_count <= 0) {
    //             need_break = true;
    //         }
    //     }
    // });

    // 声源定位线程
    // if (ENABLE_SOUND_SOURCE_LOCALIZATION) {
    //     thread ssl_thread([&]() {
    //         cas::ssl::SoundSourceDetector sound_source_detector(SAMPLE_RATE,
    //         SAMPLES, CHANNELS, MICROPHONE_NAME);
    //         sound_source_detector.start();
    //         while (true) {
    //             // lock_guard<mutex> lock(ssl_mutex);
    //             Eigen::Vector3f sound_source =
    //             sound_source_detector.locate();
    //             // cout << sound_source << endl;
    //             cas::proto::DataMessage data_message;
    //             data_message.set_type(cas::proto::DataMessage::SOUND_SOURCE);
    //             cas::proto::SoundSource *sound_source_message =
    //             data_message.mutable_sound_source();
    //             sound_source_message->set_x(sound_source[0]);
    //             sound_source_message->set_y(sound_source[1]);
    //             sound_source_message->set_z(sound_source[2]);
    //             cout << "声源位置: " << sound_source[0] << ", " <<
    //             sound_source[1] << ", " << sound_source[2] << endl;
    //             ostringstream output_stream(ios::binary);
    //             // 将 pg 对象序列化到内存输出流中
    //             if (!data_message.SerializeToOstream(&output_stream)) {
    //                 cerr << "序列化声源位置消息失败" << endl;
    //             }
    //             // 获取序列化后的数据并发送到网络对端
    //             string serialized_data = output_stream.str();
    //             if (send(client_fd, serialized_data.data(),
    //             serialized_data.size(), 0) < 0) {
    //                 cerr << "发送声源位置消息失败" << endl;
    //             } else {
    //                 cout << "发送声源位置消息成功" << endl;
    //             }
    //         }
    //     });
    // }

    // 机械臂线程
    // thread bot_arm_thread([&]() {
    // unsigned char bot_arm_buffer[128];
    // while (true) {
    // lock_guard<mutex> lock(arm_mutex);
    // int len = bot_arm.recvData(bot_arm_buffer, 128);
    // 打印数据
    // for (int i = 0; i < len; i++) {
    //     cout << (int)bot_arm_buffer[i] << " ";
    // }
    // cout << endl;
    // }
    // });

    thread receive_client_thread([&]() {
        char client_buffer[1024];
        while (true) {
            // TODO: 是否需要加锁？
            // lock_guard<mutex> lock(client_mutex);

            cas::proto::DataMessage data_message;
            if (!client.recvMessage(data_message)) {
                continue;
            }
            switch ((int)data_message.type()) {
                case (int)cas::proto::DataMessage::BOT_MOTOR: {
                    cout << "收到重建请求" << endl;
                    if (bot_motor.rotate(FIRST_MOTOR_ROTATION)) {
                        cout << "舵机旋转成功" << endl;
                        if (FIRST_MOTOR_ROTATION == "F") {
                            FIRST_MOTOR_ROTATION = "R";
                            program_config.set("first_motor_rotation", "R");
                        } else if (FIRST_MOTOR_ROTATION == "R") {
                            FIRST_MOTOR_ROTATION = "F";
                            program_config.set("first_motor_rotation", "F");
                        } else {
                            cerr << "未知的舵机旋转方向！" << endl;
                        }
                    }
                    break;
                }
                case (int)cas::proto::DataMessage::BOT_CAR: {
                    cout << "收到机器人数据" << endl;
                    int seq_length = data_message.bot_car().move_sequence_size();
                    int sequence_flag = data_message.bot_car().move_sequence(0);
                    cout << "序列长度: " << seq_length << endl;
                    float seq[seq_length];
                    for (int i = 0; i < seq_length; i++) {
                        cout << data_message.bot_car().move_sequence(i) << " ";
                        seq[i] = data_message.bot_car().move_sequence(i);
                    }
                    bot_car.executeMoveSequence(seq, seq_length);
                    break;
                }
                case (int)cas::proto::DataMessage::BOT_ARM: {
                    int length = data_message.bot_arm().data_buffer().length();
                    int angles[6];
                    char recv_buffer1[length];
                    memcpy(recv_buffer1, data_message.bot_arm().data_buffer().data(), length);
                    for (int i = 0; i < 6; i++) {
                        if (recv_buffer1[5 + 2 * i] + recv_buffer1[4 + 2 * i] * 256 > 33000) {
                            angles[i] = (recv_buffer1[5 + 2 * i] + recv_buffer1[4 + 2 * i] * 256 - 65536) / 100;
                        } else {
                            angles[i] = (recv_buffer1[5 + 2 * i] + recv_buffer1[4 + 2 * i] * 256) / 100;
                        }
                    }
                    bot_arm.execute(data_message.bot_arm().data_buffer().data(), length);
                    bot_arm.sendCommand(cas::bot::BotArm::CommandSet::READ_ANGLE);
                    break;
                }
                case (int)cas::proto::DataMessage::BOT_GRIPPER: {
                    cout << "收到机械臂夹爪数据" << endl;
                    int status = data_message.bot_gripper().status();
                    if (status == 1) {
                        bot_arm.openGripper(0x32);
                    } else if (status == 0) {
                        bot_arm.closeGripper();
                    }
                    break;
                }
                case (int)cas::proto::DataMessage::OTHER:
                default:
                    cout << "未知的客户端数据类型" << endl;
                    break;
            }
        }
    });

    thread receive_stm32_thread([&]() {
        unsigned char stm32_buffer[32];
        // int i = 0; vv av
        while (true) {
            unique_lock<mutex> lock(stm32_mutex);
            // 等待接受到stm32的数据
            bot_motor.recvData(stm32_buffer, 32);

            char data_type = stm32_buffer[0];
            // 解析stm32数据
            switch (data_type) {
                case 'M': {
                    if (stm32_buffer[4] == 'D' && stm32_buffer[5] == 'O' && stm32_buffer[6] == 'N' &&
                        stm32_buffer[7] == 'E') {
                        cout << "舵机旋转完成" << endl;
                        // if (i++ > 0) {
                        flag_recording = true;
                        // }

                        // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    }
                    // "M$F DONE", "M$R DONE"
                    // 舵机旋转反馈
                    break;
                }
                case 'T': {
                    float humi = stm32_buffer[1] + stm32_buffer[2] / 10.0;
                    float temp = stm32_buffer[3] + stm32_buffer[4] / 10.0;
                    cas::proto::DataMessage data_message;
                    data_message.set_type(cas::proto::DataMessage::TEMP_AND_HUMI);
                    cas::proto::TempAndHumi *temp_and_humi = data_message.mutable_temp_and_humi();
                    temp_and_humi->set_humi(humi);
                    temp_and_humi->set_temp(temp);
                    unique_lock<mutex> lock(client_mutex);
                    client.sendMessage(data_message);

                    // cout << "温度: " << temp << " 湿度: " << humi << endl;
                    break;
                }
            }
        }
    });

    // Kinect 配置文件
    io::AzureKinectSensorConfig sensor_config;
    string azure_kinect_config_file = "../azure_kinect_sensor_conf.json";
    io::ReadIJsonConvertibleFromJSON(azure_kinect_config_file, sensor_config);
    // 初始化 Kinect 相机
    io::AzureKinectRecorder recorder(sensor_config, 0);
    if (!recorder.InitSensor()) {
        cerr << "初始化相机失败!" << endl;
        return -1;
    }
    // 此次录制文件夹
    string recording_folder_path = RECORDINGS_FOLDER_PATH + "recording_" + utility::GetCurrentTimeStamp() + "/";
    utility::filesystem::MakeDirectoryHierarchy(recording_folder_path); // 创建此次录制文件夹
    string recording_file_name = "recon_" + utility::GetCurrentTimeStamp() + ".mkv";
    // MKV 文件路径
    string mkv_file_path = recording_folder_path + recording_file_name;
    // MKV 解析文件夹
    // string mkv_parse_folder_path = recording_folder_path + recording_file_name + "/";
    // utility::filesystem::MakeDirectoryHierarchy(mkv_parse_folder_path);

    // 开始录制
    recorder.OpenRecord(mkv_file_path);

    // 开始旋转相机
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (bot_motor.rotate(FIRST_MOTOR_ROTATION)) {
        cout << "舵机旋转成功" << endl;
        if (FIRST_MOTOR_ROTATION == "F") {
            FIRST_MOTOR_ROTATION = "R";
            program_config.set("first_motor_rotation", "R");
        } else if (FIRST_MOTOR_ROTATION == "R") {
            FIRST_MOTOR_ROTATION = "F";
            program_config.set("first_motor_rotation", "F");
        } else {
            cerr << "未知的舵机旋转方向！" << endl;
        }
    }
    // timer_cv.notify_one();

    // 开启深度和彩色图像的对齐
    bool enable_align_depth_to_color = true;

    // MKV文件读取器
    io::MKVReader mkv_reader;

    // 体素网格参数
    float voxel_size = 3.f / 512.f;
    float trunc_voxel_multiplier = 8.0f;
    int block_resolution = 16;
    int block_count = 10000;

    // 里程计参数
    float depth_scale = 1000.f;
    float depth_max = 3.f;
    float depth_diff = 0.07f;

    // 相机内参
    camera::PinholeCameraIntrinsic intrinsic =
        camera::PinholeCameraIntrinsic(camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);

    // auto focal_length = intrinsic.GetFocalLength();
    // auto principal_point = intrinsic.GetPrincipalPoint();
    // core::Tensor intrinsic_t = core::Tensor::Init<double>(
    //     {{focal_length.first, 0, principal_point.first}, {0, focal_length.second, principal_point.second}, {0, 0,
    //     1}});

    // 设备类型
    core::Device cuda_ = core::Device("cuda:0");

    while (true) {
        if (need_reconnstrcution) {
            // true 表示开始录制
            auto im_rgbd = recorder.RecordFrame(true, enable_align_depth_to_color);
            if (im_rgbd == nullptr) {
                cerr << "获取图像失败! 跳过此帧" << endl;
                continue;
            }

            if (flag_recording) {
                // TODO: 可以封装
                if (recorder.IsRecordCreated()) {
                    cout << "录制完毕" << endl;
                } else {
                    cerr << "录制失败！" << endl;
                    return -1;
                }
                // 完成一次重建录制，关闭相机
                recorder.CloseRecord();

                // 读取 mkv 文件中的数据
                // FIXME: 不需要保存成文件
                // utility::filesystem::MakeDirectory(mkv_parse_folder_path + "/color");
                // utility::filesystem::MakeDirectory(mkv_parse_folder_path + "/depth");

                // 读取录制的 mkv 文件
                mkv_reader.Open(mkv_file_path);

                intrinsic = mkv_reader.GetMetadata().intrinsics_;
                auto focal_length = intrinsic.GetFocalLength();
                auto principal_point = intrinsic.GetPrincipalPoint();
                core::Tensor intrinsic_t = core::Tensor::Init<double>({{focal_length.first, 0, principal_point.first},
                                                                       {0, focal_length.second, principal_point.second},
                                                                       {0, 0, 1}});

                if (!mkv_reader.IsOpened()) { // 打开失败
                    cerr << "打开 MKV 文件失败！" << endl;
                    return -1;
                }

                std::shared_ptr<geometry::RGBDImage> im_rgbd;
                // int ok = 1;
                // while (!mkv_reader.IsEOF()) {
                // 读取第一个有效帧
                do {
                    im_rgbd = mkv_reader.NextFrame();
                } while (im_rgbd == nullptr);

                // 初始化 SLAM 模型
                core::Tensor T_frame_to_model = core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));
                t::pipelines::slam::Model model(voxel_size, block_resolution, block_count, T_frame_to_model, cuda_);

                // 读取深度图像帧
                // 使用t::geometry::Image的静态方法FromLegacy，将geometry::Image转换为t::geometry::Image
                t::geometry::Image ref_depth = t::geometry::Image::FromLegacy(im_rgbd->depth_, cuda_);
                t::pipelines::slam::Frame input_frame(ref_depth.GetRows(), ref_depth.GetCols(), intrinsic_t, cuda_);
                t::pipelines::slam::Frame raycast_frame(ref_depth.GetRows(), ref_depth.GetCols(), intrinsic_t, cuda_);

                int i = 0;
                // 循环读取mkv文件
                while (!mkv_reader.IsEOF()) {
                    // TODO: 保存成 MKV 解析文件

                    cout << "处理中：" << i << endl;
                    // 读取一帧
                    auto im_rgbd = mkv_reader.NextFrame();

                    if (im_rgbd == nullptr) { // 读取失败则跳过
                        continue;
                    }

                    input_frame.SetDataFromImage("depth", t::geometry::Image::FromLegacy(im_rgbd->depth_, cuda_));
                    input_frame.SetDataFromImage("color", t::geometry::Image::FromLegacy(im_rgbd->color_, cuda_));

                    // 里程计跟踪
                    bool tracking_success = true;

                    if (i++ > 0) {
                        t::pipelines::odometry::OdometryResult result;
                        try {
                            result =
                                model.TrackFrameToModel(input_frame, raycast_frame, depth_scale, depth_max, depth_diff);
                            core::Tensor translation = result.transformation_.Slice(0, 0, 3).Slice(1, 3, 4);
                            double translation_norm = sqrt((translation * translation).Sum({0, 1}).Item<double>());

                            if (result.fitness_ >= 0.1 && translation_norm < 0.15) {
                                T_frame_to_model = T_frame_to_model.Matmul(result.transformation_);
                            } else {
                                tracking_success = false;
                            }
                        } catch (const runtime_error &e) {
                            cout << e.what() << endl;
                            tracking_success = false;
                            --i;
                        }
                    }

                    model.UpdateFramePose(i, T_frame_to_model);
                    if (tracking_success) {
                        model.Integrate(input_frame, depth_scale, depth_max, trunc_voxel_multiplier);
                    }
                    model.SynthesizeModelFrame(raycast_frame, depth_scale, 0.1, depth_max, trunc_voxel_multiplier, false);
                }

                auto des_mesh = model.ExtractTriangleMesh().ToLegacy();

                if (IS_CREATE_SERVER) {
                    cout << "开始发送数据" << endl;
                    cas::proto::DataMessage data_message;
                    // 设置消息类型
                    data_message.set_type(cas::proto::DataMessage::MESH);
                    cas::proto::Mesh *mesh_message = data_message.mutable_mesh();
                    // 顶点坐标
                    const vector<Eigen::Vector3d> &vertices = des_mesh.vertices_;
                    // 顶点索引
                    const vector<Eigen::Vector3i> &triangles = des_mesh.triangles_;
                    // 顶点颜色
                    const vector<Eigen::Vector3d> &colors = des_mesh.vertex_colors_;

                    int write_count = 0;
                    for (int i = 0; i < triangles.size(); i++) {
                        cas::proto::V1 *v1 = mesh_message->add_v1();
                        int v1_index = triangles[i][0];
                        v1->set_x(vertices[v1_index][0]);
                        v1->set_y(vertices[v1_index][1]);
                        v1->set_z(vertices[v1_index][2]);

                        cas::proto::V2 *v2 = mesh_message->add_v2();
                        int v2_index = triangles[i][1];
                        v2->set_x(vertices[v2_index][0]);
                        v2->set_y(vertices[v2_index][1]);
                        v2->set_z(vertices[v2_index][2]);

                        cas::proto::V3 *v3 = mesh_message->add_v3();
                        int v3_index = triangles[i][2];
                        v3->set_x(vertices[v3_index][0]);
                        v3->set_y(vertices[v3_index][1]);
                        v3->set_z(vertices[v3_index][2]);

                        mesh_message->add_r((colors[v1_index][0] + colors[v2_index][0] + colors[v3_index][0]) / 3.0);
                        mesh_message->add_g((colors[v1_index][1] + colors[v2_index][1] + colors[v3_index][1]) / 3.0);
                        mesh_message->add_b((colors[v1_index][2] + colors[v2_index][2] + colors[v3_index][2]) / 3.0);

                        if ((i + 1) % 500 == 0 || i == (triangles.size() - 1)) {
                            unique_lock<mutex> lock(client_mutex);
                            client.sendMessage(data_message);
                            mesh_message->Clear();
                            write_count++;
                        }
                    }
                    cout << "===============================" << endl;
                    cout << "== 发送完毕. 一共发送了 " << write_count << " 次" << endl;
                    cout << "== 面片数量: " << triangles.size() << endl;
                    cout << "===============================" << endl;
                }

                // FIXME:
                // std::this_thread::sleep_for(std::chrono::milliseconds(500));
                client.sendExitMeshMessage();
            need_reconnstrcution = false;
            }
        }
    }
    return 1;
}