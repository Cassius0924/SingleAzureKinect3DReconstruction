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
#include "CasNetwork.h"
#include "CasSoundSourceLocalization.h"
#include "CasUtility.h"
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
using namespace cas::utility;

void onRotated(cas::utility::Config &config, string &FIRST_MOTOR_ROTATION) {
    Debug::CoutSuccess("舵机旋转成功");
    if (FIRST_MOTOR_ROTATION == "F") {
        FIRST_MOTOR_ROTATION = "R";
        config.set("first_motor_rotation", "R");
    } else if (FIRST_MOTOR_ROTATION == "R") {
        FIRST_MOTOR_ROTATION = "F";
        config.set("first_motor_rotation", "F");
    } else {
        Debug::CoutError("未知的舵机旋转方向！");
    }
}

int main(int argc, char **argv) { // TODO: 可以传参，传入配置文件路径
    // 读取配置文件
    string config_file_path = "../default.conf";
    if (argc > 1) {
        config_file_path = argv[1];
    }
    cas::utility::Config program_config(config_file_path);

    float INTERVAL_RADIAN = program_config.getFloat("interval_radian");
    float EXIT_RADIAN = program_config.getFloat("exit_radian");
    float VOXEL_SIZE = program_config.getFloat("voxel_size");
    float FINAL_VOXEL_SIZE = program_config.getFloat("final_voxel_size");
    float SMALL_RADIUS_MULTIPLIER = program_config.getFloat("small_radius_multiplier");
    float LARGE_RADIUS_MULTIPLIER = program_config.getFloat("large_radius_multiplier");
    // string WEB_SOCKET_SERVER_ADDRESS = program_config.get("web_socket_server_address");
    // int WEB_SOCKET_SERVER_PORT = program_config.getInt("web_socket_server_port");
    // string WEB_SOCKET_SERVER_PATH = program_config.get("web_socket_server_path");
    bool IS_WRITE_FILE = program_config.getBool("is_write_file");
    string CLOUD_FILE_PATH = program_config.get("cloud_file_path");
    string MESH_FILE_PATH = program_config.get("mesh_file_path");
    bool IS_CREATE_SERVER = program_config.getBool("is_create_server");
    bool IS_CONNECT_ARM = program_config.getBool("is_connect_arm");
    bool IS_CONNECT_KINECT = program_config.getBool("is_connect_kinect");
    bool ENABLE_SOUND_SOURCE_LOCALIZATION = program_config.getBool("enable_sound_source_localization");
    int SERVER_PORT = program_config.getInt("protobuf_server_port");
    unsigned int SAMPLE_RATE = program_config.getInt("sample_rate");
    int SAMPLES = program_config.getInt("samples");
    int CHANNELS = program_config.getInt("channels");
    string MICROPHONE_NAME = program_config.get("microphone_name");
    string BOT_ARM_SERIAL_PORT_NAME = program_config.get("bot_arm_serial_prot_name");
    string BOT_CAR_SERIAL_PORT_NAME = program_config.get("bot_car_serial_prot_name");
    string STM32_SERIAL_PORT_NAME = program_config.get("stm32_serial_prot_name");
    string FIRST_MOTOR_ROTATION = program_config.get("first_motor_rotation");
    float TEMPERATURE_THRESHOLD = program_config.getFloat("temperature_threshold");
    float HUMIDITY_THRESHOLD = program_config.getFloat("humidity_threshold");
    string RECORDINGS_FOLDER_PATH = program_config.get("recordings_folder_path");
    float BLOCK_VOXEL_SIZE = program_config.getFloat("block_voxel_size");
    float TRUNC_VOXEL_MULTIPLIER = program_config.getFloat("trunc_voxel_multiplier");
    int BLOCK_RESOLUTION = program_config.getInt("block_resolution");
    int BLOCK_COUNT = program_config.getInt("block_count");
    float DEPTH_SCALE = program_config.getFloat("depth_scale");
    float DEPTH_MAX = program_config.getFloat("depth_max");
    float DEPTH_DIFF = program_config.getFloat("depth_diff");

    k4a::device device;
    // k4a_device_t device;

    cas::bot::BotArm bot_arm(BOT_ARM_SERIAL_PORT_NAME);
    cas::bot::BotMotor bot_motor(STM32_SERIAL_PORT_NAME);
    cas::bot::BotCar bot_car(BOT_CAR_SERIAL_PORT_NAME, (char)0x12, 0.62);
    cas::bot::BotLed bot_led(STM32_SERIAL_PORT_NAME);

    // 发现已连接的设备数
    if (cas::kinect::checkKinectNum(1) == false) {
        return 0;
    }

    bot_motor.rotate(FIRST_MOTOR_ROTATION, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

    // 打开（默认）设备
    device = k4a::device::open(K4A_DEVICE_DEFAULT);
    // k4a_device_open(0, &device);
    // cout << "打开 Azure Kinect 设备" << endl;
    k4a_device_configuration_t config;

    // 配置并启动设备
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG; // TODO: 试试 BGRA32
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true; // 只输出同步的图像，即同步深度图和彩色图
    // k4a_device_start_cameras(device, &config);
    device.start_cameras(&config);
    cout << "开启相机。" << endl;

    // 稳定化
    cas::kinect::stabilizeCamera(device);
    cout << "------------------------------------" << endl;
    cout << "----- 成功启动 Azure Kinect DK -----" << endl;
    cout << "------------------------------------" << endl;

    k4a::calibration k4a_calibration = device.get_calibration(config.depth_mode, config.color_resolution);
    // _k4a_calibration_t k4a_calibration;
    // k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &k4a_calibration);
    k4a::transformation k4a_transformation = k4a::transformation(k4a_calibration);
    // k4a_transformation_t k4a_transformation = k4a_transformation_create(&k4a_calibration);

    if (IS_CONNECT_ARM) {
        bot_arm.reset();
        Debug::CoutSuccess("机械臂复位成功");
    }

    // LED亮红
    bot_led.setLedColor(cas::bot::BotLed::LedColor::RED);

    // 创建服务器等待连接
    cas::net::Client client(SERVER_PORT, [&]() {
        // LED亮绿
        bot_led.setLedColor(cas::bot::BotLed::LedColor::GREEN);
    });

    // 定义互斥锁
    mutex arm_mutex;
    mutex client_mutex;
    mutex stm32_mutex;
    mutex car_mutex;
    mutex recon_mutex;

    // 定义条件变量
    condition_variable arm_cv;
    condition_variable stm32_cv;
    condition_variable client_cv;
    condition_variable timer_cv;
    // 定义共享变量
    bool ready_to_break = false;
    bool need_break = false;

    bool need_reconnstrcution = true;
    bool flag_recording = true;

    cas::proto::KinectMode::Mode kinect_mode = cas::proto::KinectMode::REAL_TIME;

    // 声源定位线程
    thread ssl_thread([&]() {
        if (ENABLE_SOUND_SOURCE_LOCALIZATION) {
            cas::ssl::SoundSourceDetector sound_source_detector(SAMPLE_RATE, SAMPLES, CHANNELS, MICROPHONE_NAME);
            sound_source_detector.start();
            while (true) {
                // lock_guard<mutex> lock(ssl_mutex);
                Eigen::Vector3f sound_source = sound_source_detector.locate();
                cas::proto::DataMessage data_message;
                data_message.set_type(cas::proto::DataMessage::SOUND_SOURCE);
                cas::proto::SoundSource *sound_source_message = data_message.mutable_sound_source();
                sound_source_message->set_x(sound_source[0]);
                sound_source_message->set_y(sound_source[1]);
                sound_source_message->set_z(sound_source[2]);
                Debug::CoutInfo("声源位置: {}, {}, {}", sound_source[0], sound_source[1], sound_source[2]);
                client.sendMessage(data_message);
            }
        }
    });

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
            cas::proto::DataMessage data_message;
            if (!client.recvMessage(data_message)) {
                continue;
            }
            switch ((int)data_message.type()) {
                case (int)cas::proto::DataMessage::BOT_MOTOR: {
                    Debug::CoutSuccess("收到重建请求");
                    // recorder = io::AzureKinectRecorder(sensor_config, 0);
                    // io::AzureKinectRecorder recorder(sensor_config, 0);

                    // recorder = new io::AzureKinectRecorder(sensor_config, 0);
                    // if (!recorder.InitSensor()) {
                    //     cerr << "初始化相机失败!" << endl;
                    //     return -1;
                    // }

                    unique_lock<mutex> lock(recon_mutex);
                    flag_recording = true;
                    need_reconnstrcution = true;
                    break;
                }
                case (int)cas::proto::DataMessage::BOT_CAR: {
                    Debug::CoutSuccess("收到机器人数据");
                    int seq_length = data_message.bot_car().move_sequence_size();
                    int sequence_flag = data_message.bot_car().move_sequence(0);
                    Debug::CoutDebug("序列长度: {}", seq_length);
                    float seq[seq_length];
                    for (int i = 0; i < seq_length; i++) {
                        // cout << data_message.bot_car().move_sequence(i) << " ";
                        Debug::CoutDebug("序列: {}", data_message.bot_car().move_sequence(i));
                        seq[i] = data_message.bot_car().move_sequence(i);
                    }
                    bot_car.executeMoveSequence(seq, seq_length);
                    break;
                }
                case (int)cas::proto::DataMessage::BOT_ARM: {
                    Debug::CoutSuccess("收到机械臂数据");
                    int length = data_message.bot_arm().data_buffer().length();
                    // int angles[6];
                    // char recv_buffer1[length];
                    // memcpy(recv_buffer1, data_message.bot_arm().data_buffer().data(), length);
                    // for (int i = 0; i < 6; i++) {
                    //     if (recv_buffer1[5 + 2 * i] + recv_buffer1[4 + 2 * i] * 256 > 33000) {
                    //         angles[i] = (recv_buffer1[5 + 2 * i] + recv_buffer1[4 + 2 * i] * 256 - 65536) / 100;
                    //     } else {
                    //         angles[i] = (recv_buffer1[5 + 2 * i] + recv_buffer1[4 + 2 * i] * 256) / 100;
                    //     }
                    // }
                    bot_arm.execute(data_message.bot_arm().data_buffer().data(), length);
                    bot_arm.sendCommand(cas::bot::BotArm::CommandSet::READ_ANGLE);
                    break;
                }
                case (int)cas::proto::DataMessage::BOT_GRIPPER: {
                    Debug::CoutSuccess("收到机械臂夹爪数据");
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
                    Debug::CoutError("未知的客户端数据类型");
                    break;
            }
        }
    });

    thread receive_stm32_thread([&]() {
        unsigned char stm32_buffer[32];
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
                        flag_recording = false;
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
                    break;
                }
            }
        }
    });

    // 开启深度和彩色图像的对齐
    bool enable_align_depth_to_color = true;

    // MKV文件读取器

    // 体素网格参数
    float voxel_size = BLOCK_VOXEL_SIZE;
    float trunc_voxel_multiplier = TRUNC_VOXEL_MULTIPLIER;
    int block_resolution = BLOCK_RESOLUTION;
    int block_count = BLOCK_COUNT;

    // 里程计参数
    float depth_scale = DEPTH_SCALE;
    float depth_max = DEPTH_MAX;
    float depth_diff = DEPTH_DIFF;

    // 相机内参
    camera::PinholeCameraIntrinsic intrinsic =
        camera::PinholeCameraIntrinsic(camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);

    // 设备类型
    core::Device cuda_ = core::Device("cuda:0");

    // io::AzureKinectSensorConfig sensor_config;
    // string azure_kinect_config_file = "../azure_kinect_sensor_conf.json";
    // io::ReadIJsonConvertibleFromJSON(azure_kinect_config_file, sensor_config);
    // io::AzureKinectSensor sensor(sensor_config);
    // io::AzureKinectRecorder recorder(sensor_config, 0);
    // if (!recorder.InitSensor()) {
    //     Debug::CoutError("初始化相机失败!");
    //     return -1;
    // }

    // sensor.Connect(0);
    // Debug::CoutSuccess("相机初始化成功");

    while (true) {
        switch (kinect_mode) {
            case cas::proto::KinectMode::RECONSTRCUTION: {
                Debug::CoutInfo("切换至重建模式");

                if (!need_reconnstrcution) {
                    continue;
                }

                this_thread::sleep_for(std::chrono::milliseconds(1000));

                // io::MKVReader mkv_reader;
                // cout << "1" << endl;
                // // 初始化 Kinect 相机
                // // 此次录制文件夹
                // string recording_folder_path = RECORDINGS_FOLDER_PATH + "recording_" + utility::GetCurrentTimeStamp()
                // +
                // "/"; utility::filesystem::MakeDirectoryHierarchy(recording_folder_path); // 创建此次录制文件夹 string
                // recording_file_name = "recon_" + utility::GetCurrentTimeStamp() + ".mkv"; cout << "2" << endl;
                // // MKV 文件路径
                // string mkv_file_path = recording_folder_path + recording_file_name;
                // cout << "3" << endl;

                // // 开始录制
                // recorder.OpenRecord(mkv_file_path);
                // cout << "4" << endl;

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

                while (true) {
                    // true 表示开始录制
                    // auto im_rgbd = recorder.RecordFrame(true, enable_align_depth_to_color);
                    // if (im_rgbd == nullptr) {
                    //     cerr << "获取图像失败! 跳过此帧" << endl;
                    //     continue;
                    // }

                    // if (flag_recording) {
                    // TODO: 可以封装
                    // if (recorder.IsRecordCreated()) {
                    //     cout << "录制完毕" << endl;
                    // } else {
                    //     cerr << "录制失败！" << endl;
                    //     return -1;
                    // }
                    // // 完成一次重建录制，关闭相机
                    // recorder.CloseRecord();
                    // recorder.~AzureKinectRecorder();

                    // 读取 mkv 文件中的数据

                    // FIXME: 不需要保存成文件
                    // utility::filesystem::MakeDirectory(mkv_parse_folder_path + "/color");
                    // utility::filesystem::MakeDirectory(mkv_parse_folder_path + "/depth");

                    // 读取录制的 mkv 文件
                    // mkv_reader.Open(mkv_file_path);

                    // intrinsic = mkv_reader.GetMetadata().intrinsics_;
                    // auto focal_length = intrinsic.GetFocalLength();
                    // auto principal_point = intrinsic.GetPrincipalPoint();
                    // core::Tensor intrinsic_t =
                    //     core::Tensor::Init<double>({{focal_length.first, 0, principal_point.first},
                    //                                 {0, focal_length.second, principal_point.second},
                    //                                 {0, 0, 1}});

                    core::Tensor intrinsic_t =
                        core::Tensor::Init<double>({{963.205, 0, 1012.87}, {0, 962.543, 777.369}, {0, 0, 1}});

                    // if (!mkv_reader.IsOpened()) { // 打开失败
                    //     cerr << "打开 MKV 文件失败！" << endl;
                    //     return -1;
                    // }

                    std::shared_ptr<geometry::RGBDImage> im_rgbd;

                    // 读取第一个有效帧
                    do {
                        // im_rgbd = sensor.CaptureFrame(true);
                    } while (im_rgbd == nullptr);

                    // 初始化 SLAM 模型
                    core::Tensor T_frame_to_model = core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));
                    t::pipelines::slam::Model model(voxel_size, block_resolution, block_count, T_frame_to_model, cuda_);

                    // 读取深度图像帧
                    // 使用t::geometry::Image的静态方法FromLegacy，将geometry::Image转换为t::geometry::Image
                    t::geometry::Image ref_depth = t::geometry::Image::FromLegacy(im_rgbd->depth_, cuda_);
                    t::pipelines::slam::Frame input_frame(ref_depth.GetRows(), ref_depth.GetCols(), intrinsic_t, cuda_);
                    t::pipelines::slam::Frame raycast_frame(ref_depth.GetRows(), ref_depth.GetCols(), intrinsic_t,
                                                            cuda_);
                    cout << "%6" << endl;

                    int i = 0;
                    // 循环读取mkv文件
                    while (flag_recording) {
                        // TODO: 保存成 MKV 解析文件

                        cout << "处理中：" << i << endl;

                        // 读取一帧
                        // auto im_rgbd = mkv_reader.NextFrame();

                        // im_rgbd = sensor.CaptureFrame(true);

                        if (im_rgbd == nullptr) { // 读取失败则跳过
                            continue;
                        }

                        input_frame.SetDataFromImage("depth", t::geometry::Image::FromLegacy(im_rgbd->depth_, cuda_));
                        input_frame.SetDataFromImage("color", t::geometry::Image::FromLegacy(im_rgbd->color_, cuda_));

                        // 里程计跟踪
                        bool tracking_success = true;

                        if (i > 0) {
                            t::pipelines::odometry::OdometryResult result;
                            try {
                                result = model.TrackFrameToModel(input_frame, raycast_frame, depth_scale, depth_max,
                                                                 depth_diff);
                                core::Tensor translation = result.transformation_.Slice(0, 0, 3).Slice(1, 3, 4);
                                double translation_norm = sqrt((translation * translation).Sum({0, 1}).Item<double>());

                                if (result.fitness_ >= 0.1 && translation_norm < 0.15) {
                                    T_frame_to_model = T_frame_to_model.Matmul(result.transformation_);
                                } else {
                                    tracking_success = false;
                                    cout << "失败啦！" << endl;
                                }
                                cout << "fitness: " << result.fitness_ << "， translation_norm: " << translation_norm
                                     << endl;
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
                        model.SynthesizeModelFrame(raycast_frame, depth_scale, 0.1, depth_max, trunc_voxel_multiplier,
                                                   false);
                        i++;
                    }

                    // sensor.Disconnect();

                    // mkv_reader.Close();
                    // mkv_reader.~MKVReader();
                    auto des_mesh = model.ExtractTriangleMesh().ToLegacy();
                    model.~Model();
                    io::WriteTriangleMesh("ply/slam_mesh.ply", des_mesh);
                    break;
                }
            }
            case cas::proto::KinectMode::REAL_TIME: {
                Debug::CoutInfo("切换至实时模式");

                open3d::geometry::PointCloud cloud;

                k4a::capture capture;
                device.get_capture(&capture);

                k4a::image rgb_image_item = capture.get_color_image();
                k4a::image depth_image_item = capture.get_depth_image();

                int color_image_width_pixels = rgb_image_item.get_width_pixels();
                int color_image_height_pixels = rgb_image_item.get_height_pixels();

                k4a::image transformed_depthImage =
                    k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, color_image_width_pixels, color_image_height_pixels,
                                       color_image_width_pixels * (int)sizeof(uint16_t));
                k4a::image point_cloud_image =
                    k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels, color_image_height_pixels,
                                       color_image_width_pixels * 3 * (int)sizeof(int16_t));

                k4a_transformation.depth_image_to_color_camera(depth_image_item, &transformed_depthImage);

                k4a_transformation.depth_image_to_point_cloud(transformed_depthImage, K4A_CALIBRATION_TYPE_COLOR,
                                                              &point_cloud_image);

                cloud.points_.resize(color_image_width_pixels * color_image_height_pixels);
                cloud.colors_.resize(color_image_width_pixels * color_image_height_pixels);

                const int16_t *point_cloud_image_data =
                    reinterpret_cast<const int16_t *>(point_cloud_image.get_buffer());
                const uint8_t *color_image_data = rgb_image_item.get_buffer();

                for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++) {
                    if (point_cloud_image_data[3 * i + 0] != 0 && point_cloud_image_data[3 * i + 1] != 0 &&
                        point_cloud_image_data[3 * i + 2] != 0) {
                        cloud.points_[i] = Eigen::Vector3d(point_cloud_image_data[3 * i + 0] / 1000.0f,
                                                           point_cloud_image_data[3 * i + 1] / 1000.0f,
                                                           point_cloud_image_data[3 * i + 2] / 1000.0f);
                        cloud.colors_[i] =
                            Eigen::Vector3d(color_image_data[4 * i + 2] / 255.0f, color_image_data[4 * i + 1] /
                            255.0f,
                                            color_image_data[4 * i + 0] / 255.0f);
                    } else {
                        cloud.points_[i] = Eigen::Vector3d::Zero();
                        cloud.colors_[i] = Eigen::Vector3d::Zero();
                    }
                }

                // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV

                // k4a::capture capture;

                // Debug::CoutDebug("实时 1 帧");
                // 将iamge转点云
                // auto cloud = *geometry::PointCloud::CreateFromRGBDImage(
                //     *image, camera::PinholeCameraIntrinsic(
                //                 camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault));

                auto point_cloud = *cloud.VoxelDownSample(0.03);

                geometry::KDTreeSearchParamHybrid kd_tree_param(0.03 * 2, 30);

                point_cloud.EstimateNormals(kd_tree_param);

                // point_cloud转mesh
                vector<double> distances = point_cloud.ComputeNearestNeighborDistance();
                // 计算平均距离
                double avg_dist = accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
                // 设置搜索半径
                double radius = avg_dist * LARGE_RADIUS_MULTIPLIER;
                vector<double> radii = {radius, radius * 2};
                auto mesh = geometry::TriangleMesh::CreateFromPointCloudBallPivoting(point_cloud, radii);

                if (IS_CREATE_SERVER) {
                    Debug::CoutDebug("开始发送数据");
                    cas::proto::DataMessage data_message;
                    // 设置消息类型
                    data_message.set_type(cas::proto::DataMessage::MESH);
                    cas::proto::Mesh *mesh_message = data_message.mutable_mesh();
                    // 顶点坐标
                    const vector<Eigen::Vector3d> &vertices = mesh->vertices_;
                    // 顶点索引
                    const vector<Eigen::Vector3i> &triangles = mesh->triangles_;
                    // 顶点颜色
                    const vector<Eigen::Vector3d> &colors = mesh->vertex_colors_;

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
                        Debug::CoutFlush("已发送：{}", write_count);
                    }
                    Debug::CoutSection("发送完毕", "一共发送了 {} 次\n 面片数量 {} ", write_count, triangles.size());
                }
                client.sendExitMeshMessage();
                break;
            }
            default: {
                Debug::CoutError("未知的 Kinect 模式");
                break;
            }
        }
    }

    return 1;
}
