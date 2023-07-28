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

    // k4a::capture capture;
    // k4a_device_configuration_t config;

    cas::bot::BotArm bot_arm(BOT_ARM_SERIAL_PORT_NAME);
    cas::bot::BotMotor bot_motor(STM32_SERIAL_PORT_NAME);
    cas::bot::BotCar bot_car(BOT_CAR_SERIAL_PORT_NAME, (char)0x12, 0.62);
    cas::bot::BotLed bot_led(STM32_SERIAL_PORT_NAME);

    // 发现已连接的设备数
    if (cas::kinect::checkKinectNum(1) == false) {
        return 0;
    }

    bot_motor.rotate(FIRST_MOTOR_ROTATION, [&](){
        onRotated(program_config, FIRST_MOTOR_ROTATION);
    });

    // 打开（默认）设备
    // device = k4a::device::open(K4A_DEVICE_DEFAULT);
    // cout << "打开 Azure Kinect 设备" << endl;

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

                    // recording_file_name = "recon_" + utility::GetCurrentTimeStamp() + ".mkv";
                    // mkv_file_path = recording_folder_path + recording_file_name;
                    // recorder.OpenRecord(mkv_file_path);

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

    io::AzureKinectSensorConfig sensor_config;
    string azure_kinect_config_file = "../azure_kinect_sensor_conf.json";
    io::ReadIJsonConvertibleFromJSON(azure_kinect_config_file, sensor_config);
    io::AzureKinectSensor sensor(sensor_config);
    // io::AzureKinectRecorder recorder(sensor_config, 0);
    // if (!recorder.InitSensor()) {
    //     Debug::CoutError("初始化相机失败!");
    //     return -1;
    // }

    sensor.Connect(0);
    Debug::CoutSuccess("相机初始化成功");


    while (true) {
        switch (kinect_mode) {
            case cas::proto::KinectMode::RECONSTRCUTION: {
                Debug::CoutInfo("切换至重建模式");

                break;
            }
            case cas::proto::KinectMode::REAL_TIME: {
                Debug::CoutInfo("切换至实时模式");

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

