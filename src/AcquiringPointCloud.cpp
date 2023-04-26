/*
 * SingleAzureKinect3DReconstruction
 * 此项目基于 Open3D 和 Azure Kinect DK 实现了三维重建。利用 Azure Kinect DK
 * 捕获图像并记录 IMU 数据，利用 Open3D 实现三维重建。
 *
 * Created by Cassius0924 on 2020/03/03.
 * 
 */

#include "CasAzureKinectExtrinsics.h"
#include "CasConfig.h"
#include "Hello.h"
#include "polygon.pb.h"

// #include "CasWebSocket.h"

#include <iostream>
#include <k4a/k4a.hpp>
#include <string>

#include <chrono>
#include <thread>
#include <vector>

// Open3D
#include <open3d/Open3D.h>

#define INTERVAL_RADIAN 0.4
#define EXIT_RADIAN 3.2
#define VOXEL_SIZE 0.05
#define FINAL_VOXEL_SIZE 0.01

using namespace std;

int main(int argc, char **argv) {
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

    // 读取配置文件
    // cas::CasConfig config("default.conf");
    // cout << "读取配置文件成功" << endl;

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

    Hello hi;

    unsigned char tempbuff[1024];
    unsigned char databuff[18];

    int fd_network;
    int server_socket_fd = -1;

    int recv_long = 0;

    pthread_t recv_threadID = -1;

    int client_fd = -1;
    struct sockaddr_in *addr = (struct sockaddr_in *) malloc(sizeof(struct sockaddr_in));
    socklen_t addr_len = (socklen_t) sizeof(*addr);
    memset(addr, 0, sizeof(*addr));

    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(5001);

    char ip_local[32 + 1] = {0};

    if (hi.get_local_ip(ip_local) != 0) {
        printf("ip地址\n");
        return -1;
    }
    inet_aton(ip_local, &sockaddr.sin_addr);

    server_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_fd < 0) {
        perror("Socket create failed!\n");
        return -1;
    }

    if (bind(server_socket_fd, (struct sockaddr *) &sockaddr, sizeof(sockaddr)) != 0)//∞Û∂®µÿ÷∑∫Õ∂Àø⁄∫≈
    {
        perror("Socket bind failed!\n");
        close(server_socket_fd);
        return -1;
    }
    if (listen(server_socket_fd, 1) != 0)
    {
        perror("Socket listen failed!\n");
        close(server_socket_fd);
        return -1;
    }

    //等待客户端连接
    cout << "等待客户端连接..." << endl;

    client_fd = accept(server_socket_fd, (struct sockaddr *) addr, &addr_len);

    if (client_fd < 0) {
        perror("Socket accept failed\n");
        close(server_socket_fd);
        free(addr);
        return -1;
    } else {
        printf("IP地址: %s:%d", inet_ntoa(addr->sin_addr), ntohs(addr->sin_port));
    };

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

            Eigen::Matrix3d rotation_matrix;
            rotation_matrix << cas::eulerAngle2RotationMatrix(image_data.angle);

            cloud = cloud->VoxelDownSample(VOXEL_SIZE);

            if (flag == 0) {
                flag = 1;
            } else {
                //粗配准: 基于 IMU 数据的旋转矩阵
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                cloud->Rotate(rotation_matrix, center);

                //精配准: 基于彩色 ICP 算法
                final_cloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(VOXEL_SIZE * 2, 30));//法向量估计
                cloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(VOXEL_SIZE * 2, 30));
                Eigen::Matrix4d transformation_icp = Eigen::Matrix4d::Identity();
                float relative_fitness = 1e-6;
                float relative_rmse = 1e-6;
                int max_iteration = 30;
                auto result_icp = open3d::pipelines::registration::RegistrationICP(*cloud, *final_cloud, VOXEL_SIZE * 2, transformation_icp, open3d::pipelines::registration::TransformationEstimationPointToPlane(), open3d::pipelines::registration::ICPConvergenceCriteria(relative_fitness, relative_rmse, max_iteration));
                cloud->Transform(result_icp.transformation_);
            }

            *final_cloud += *cloud;

            cloud_task_count--;
            cout << "计算完毕" << endl;
            if (ready_to_break && cloud_task_count <= 0) {
                need_break = true;
            }


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
        if (abs(prev_angle.yaw) > EXIT_RADIAN) {
            cout << "旋转角度大于π，退出" << endl;
            ready_to_break = true;

            cout << "等待线程结束..." << endl;
            while (true) {
                if (need_break) {
                    break;
                }
            }

            // double alpha = 0.01;// alpha值越大，生成的网格越稀疏
            // auto mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*final_cloud, alpha);

            // Boll-Pivoting 球形扫描算法（BPA）
            // 计算点云的最近邻距离
            vector<double> distances = final_cloud->ComputeNearestNeighborDistance();
            // 计算平均距离
            double avg_dist = accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
            // 设置搜索半径
            double radius = 3 * avg_dist;
            vector<double> radii = {radius, radius * 2};
            // 使用球形扫描算法创建三角网格
            auto pba_mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*final_cloud, radii);
            // 使用四边形网格的几何减简算法对三角网格进行几何减简
            // auto des_mesh = pba_mesh->SimplifyQuadricDecimation(100000);
            auto des_mesh = pba_mesh;
            // des_mesh->RemoveDuplicatedTriangles();// 去除重复三角形
            // des_mesh->RemoveDuplicatedVertices(); // 去除重复顶点
            // des_mesh->RemoveNonManifoldEdges();   // 去除非流形边
            // des_mesh->RemoveDegenerateTriangles();// 去除退化的三角形


            // Poisson 泊松重建（不适合室内环境）
            // auto poisson_mesh = get<0>(open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*final_cloud, 8, 0, 1.1, false));

            // final_cloud->VoxelDownSample(FINAL_VOXEL_SIZE);

            open3d::io::WritePointCloud("ply/final_cloud.ply", *final_cloud);
            open3d::io::WriteTriangleMesh("ply/final_mesh.ply", *des_mesh);

            // Protobuf 发数据
            // 先for循环遍历Mesh数据，把数据放到protobuf的结构体中，把面片的三个顶点分别放到v1、v2、v3中
            // 然后再把protobuf的结构体序列化成字节数组，再发送出去

            // q: Open3D顶点是如何组成面片的?
            // a:


            my_package::pg pg_message;

            // 顶点坐标
            const std::vector<Eigen::Vector3d> &vertices = des_mesh->vertices_;
            // 顶点索引
            const std::vector<Eigen::Vector3i> &triangles = des_mesh->triangles_;
            // 顶点颜色
            const std::vector<Eigen::Vector3d> &colors = des_mesh->vertex_colors_;

            int write_count = 0;
            int all = 0;
            cout << triangles[1][0] << " " << triangles[1][1] << " " << triangles[1][2] << endl;
            for (int i = 0; i < triangles.size(); i++) {
                my_package::V1 *v1 = pg_message.add_v1();
                int v1_index = triangles[i][0];
                v1->set_x(vertices[v1_index][0]);
                v1->set_y(vertices[v1_index][1]);
                v1->set_z(vertices[v1_index][2]);
                // cout << "v1:" << v1->x() << "," << v1->y() << "," << v1->z() << endl;

                my_package::V2 *v2 = pg_message.add_v2();
                int v2_index = triangles[i][1];
                v2->set_x(vertices[v2_index][0]);
                v2->set_y(vertices[v2_index][1]);
                v2->set_z(vertices[v2_index][2]);
                // cout << "v2:" << v2->x() << "," << v2->y() << "," << v2->z() << endl;

                my_package::V3 *v3 = pg_message.add_v3();
                int v3_index = triangles[i][2];
                v3->set_x(vertices[v3_index][0]);
                v3->set_y(vertices[v3_index][1]);
                v3->set_z(vertices[v3_index][2]);
                // cout << "v3:" << v3->x() << "," << v3->y() << "," << v3->z() << endl;
                
                my_package::T *t = pg_message.add_t();
                t->set_t1(v1_index);
                t->set_t2(v2_index);
                t->set_t3(v3_index);

                float color_r = (colors[v1_index][0] + colors[v2_index][0] + colors[v3_index][0]) / 3.0;
                float color_g = (colors[v1_index][1] + colors[v2_index][1] + colors[v3_index][1]) / 3.0;
                float color_b = (colors[v1_index][2] + colors[v2_index][2] + colors[v3_index][2]) / 3.0;
                pg_message.add_r(color_r);
                pg_message.add_g(color_g);
                pg_message.add_b(color_b);
                // cout << "color = " << color_r << "," << color_g << "," << color_b << endl;

                if ((i + 1) % 800 == 0 || i == triangles.size() - 1) {
                    // 创建一个内存输出流
                    std::ostringstream output_stream(std::ios::binary);

                    // 将 pg 对象序列化到内存输出流中
                    if (!pg_message.SerializeToOstream(&output_stream)) {
                        std::cerr << "Failed to serialize pg message" << std::endl;
                    }

                    // 获取序列化后的数据并发送到网络对端
                    std::string serialized_data = output_stream.str();
                    if (write(client_fd, serialized_data.data(), serialized_data.size()) < 0) {
                        std::cerr << "Failed to send data to the network endpoint" << std::endl;
                    }

                    all += serialized_data.size();

                    pg_message.Clear();

                    // cout << "第" << write_count << "次发送，数据大小为: " << pbSize << endl;
                    write_count++;
                }

                // if ((i + 1) % ONCE_SEND_MESH_COUNT == 0) {
                // if ((i + 1) % 500 == 0) {
                //     int pbSize = pg_message.ByteSize();

                //     //分配内存
                //     string strPb = "";
                //     strPb.resize(pbSize);

                //     //序列化
                //     uint8_t *szData = (uint8_t *) strPb.c_str();
                //     pg_message.SerializeToArray(szData, pbSize);

                //     cout << "start write" << endl;
                //     write(client_fd, szData, pbSize);
                //     cout << "finish write" << endl;
                //     // 为什么客户端接受第一次的数据大小是正确的，但是第二次接受的数据就不对了？
                //     // 因为客户端接受数据的时候，是按照缓冲区大小来接受的，如果缓冲区大小不够，就会出现数据不完整的情况

                //     pg_message.Clear();

                //     cout << "第" << write_count << "次发送，数据大小为: " << pbSize << endl;
                //     write_count++;
                // }
            }
            cout << "======================" << endl;
            cout << "发送完毕. 一共发送了 " << write_count << " 次" << endl;
            cout << "面片数量: " << triangles.size() << endl;
            cout << "======================" << endl;
            cout << all << endl;
            break;
        }
    }
    // 断开连接
    close(client_fd);
    cout << "连接已断开" << endl;

    // 释放，关闭设备
    rgb_image.reset();
    depth_image.reset();
    capture.reset();
    device.close();
    cout << "程序结束" << endl;

    return 1;
}
