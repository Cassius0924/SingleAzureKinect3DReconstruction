//
//
// Created by root on 4/7/23.
//


#include "iostream"
#include <cmath>
//导入Kinect库
#include <k4a/k4a.hpp>
#include <k4a/k4a.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "CasAzureKinectExtrinsics.h"

using namespace std;
using namespace Eigen;

//C++：
//k4a::device cas::openAzureKinectDevice() {
//    const uint32_t device_count = k4a::device::get_installed_count();
//    if (device_count == 0) {
//        cerr << "No k4a devices found!（未找到 K4a 设备）" << endl;
//        return NULL;
//    } else if (device_count != 1) {
//        cerr << "More than one k4a devices found!（找到多个 K4a 设备）" << endl;
//        return NULL;
//    }
//    //尝试打开设备
//    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
//    if (!device) {
//        cerr << "Failed to open k4a device!（打开 K4a 设备失败）" << endl;
//        return NULL;
//    }
//    cout << "Open k4a device successfully!（打开 K4a 设备成功）" << endl;
//    return device;
//}

//打开 Azure Kinect 设备
bool cas::openAzureKinectDevice(k4a_device_t *device) {
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0) {
        printf("No k4a devices found!（未找到 K4a 设备）\n");
        return false;
    } else if (device_count != 1) {
        printf("More than one k4a devices found!（找到多个 K4a 设备）\n");
        return false;
    }
    if (k4a_device_open(K4A_DEVICE_DEFAULT, device) != K4A_RESULT_SUCCEEDED) {
        printf("Failed to open k4a device!（打开 K4a 设备失败）\n");
        return false;
    }

    printf("Open k4a device successfully!（打开 K4a 设备成功）\n");
    return true;
}

//配置 Azure Kinect 设备
//void cas::configureAzureKinectDevice(k4a::device &device, k4a_device_configuration_t config) {
//    //设置设备的配置
//    device.start_cameras(&config);
//    //输出配置信息
//    cout << "== Azure Kinect Device Configuration ==" << endl;
//    cout << "Depth mode: " << config.depth_mode << endl;
//    cout << "Color resolution: " << config.color_resolution << endl;
//    cout << "Camera FPS: " << config.camera_fps << endl;
//    cout << "======================================" << endl;
//}


//输出配置信息
bool cas::configureAzureKinectDevice(k4a_device_t device, k4a_device_configuration_t config) {
    //设置设备的配置
    if (k4a_device_start_cameras(device, &config) != K4A_RESULT_SUCCEEDED) {
        printf("Failed to start cameras!（启动相机设备失败）\n");
        return false;
    }
    printf("== Azure Kinect Device Configuration ==\n");
    printf("Depth mode: %d\n", config.depth_mode);
    printf("Color format: %d\n", config.color_format);
    printf("Color resolution: %d\n", config.color_resolution);
    printf("Camera FPS: %d\n", config.camera_fps);
    printf("=======================================\n");
    return true;
}

// 获取标定数据
bool
cas::getAzureKinectCalibration(k4a_device_t device, k4a_device_configuration_t config, k4a_calibration_t *calibration) {
    if (k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, calibration) !=
        K4A_RESULT_SUCCEEDED) {
        printf("Failed to get calibration!（获取标定数据失败）\n");
        return false;
    }
    return true;
}

// 获取捕获
bool cas::getAzureKinectCapture(k4a_device_t device, k4a_capture_t *capture, int32_t timeout_in_ms) {
    int result = false;
    switch (k4a_device_get_capture(device, capture, timeout_in_ms)) {
        case K4A_WAIT_RESULT_SUCCEEDED:
            result = true;
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture!（等待捕获超时）\n");
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture！（读取捕获失败）\n");
            break;
    }
    return result;
}

// 获取深度图
bool cas::getAzureKinectDepthImage(k4a_capture_t capture, k4a_image_t *depth_image) {
    *depth_image = k4a_capture_get_depth_image(capture);
    if (*depth_image == 0) {
        printf("Failed to get depth image from capture!（从捕获中获取深度图失败）\n");
        return false;
    }
    return true;
}

// 获取彩色图
bool cas::getAzureKinectColorImage(k4a_capture_t capture, k4a_image_t *color_image) {
    *color_image = k4a_capture_get_color_image(capture);
    if (*color_image == 0) {
        printf("Failed to get color image from capture!（从捕获中获取彩色图失败）\n");
        return false;
    }
    return true;
}

// 启动 IMU 传感器
bool cas::startAzureKinectImu(k4a_device_t device) {
    bool result = false;
    switch (k4a_device_start_imu(device)) {
        case K4A_RESULT_SUCCEEDED:
            result = true;
            break;
        case K4A_RESULT_FAILED:
            printf("Failed to start IMU!（启动 IMU 失败）\n");
            break;
    }
    return result;
}

// 获取IMU样本
bool cas::getAzureKinectImuSample(k4a_device_t device, k4a_imu_sample_t *imu_sample, int32_t timeout_in_ms) {
    bool result = false;
    switch (k4a_device_get_imu_sample(device, imu_sample, timeout_in_ms)) {
        case K4A_WAIT_RESULT_SUCCEEDED:
            result = true;
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for an imu sample!（等待 IMU 样本超时）\n");
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read an imu sample!（读取 IMU 样本失败）\n");
            break;
    }
    return result;
}


//定义一个欧拉角类，用来表示姿态角
//roll:绕x轴旋转的角度，pitch:绕y轴旋转的角度，yaw:绕z轴旋转的角度
cas::EulerAngle::EulerAngle(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}


//定义一个函数，用来根据陀螺仪的数据和上一时刻的姿态，计算当前时刻的姿态
cas::EulerAngle cas::calculateOrientation(double gx, double gy, double gz, double dt, const EulerAngle &prevAngle) {
    //创建一个陀螺仪变换矩阵对象
    Matrix3d gyroMatrix;
    double sr = sin(prevAngle.roll);
    double cr = cos(prevAngle.roll);
    double cp = cos(prevAngle.pitch);
    double tp = tan(prevAngle.pitch);

    gyroMatrix << 1, sr * tp, cr * tp,
            0, cr, -sr,
            0, sr / cp, cr / cp;

    //创建一个陀螺仪角速度向量对象
    Vector3d gyroVector(gx, gy, gz);

    //计算欧拉角速度向量
    Vector3d eulerVector = gyroMatrix * gyroVector;

    //计算欧拉角增量
    double dRoll = eulerVector(0) * dt;
    double dPitch = eulerVector(1) * dt;
    double dYaw = eulerVector(2) * dt;

    //计算当前时刻的欧拉角
    double roll = prevAngle.roll + dRoll;
    double pitch = prevAngle.pitch + dPitch;
    double yaw = prevAngle.yaw + dYaw;

    //返回当前时刻的欧拉角对象
    return cas::EulerAngle(roll, pitch, yaw);    //单位：弧度，范围：-π~π。π
}

Matrix3d cas::eulerAngle2RotationMatrix(const EulerAngle &angle) {
    Matrix3d rotationMatrix;
    rotationMatrix << (AngleAxisd(angle.roll, Vector3d::UnitX()) *
                      AngleAxisd(angle.pitch, Vector3d::UnitY()) *
                      AngleAxisd(angle.yaw, Vector3d::UnitZ())).toRotationMatrix();

    Vector3d ea = rotationMatrix.eulerAngles(2, 1, 0);

//    double cr = cos(angle.roll);
//    double sr = sin(angle.roll);
//    double cp = cos(angle.pitch);
//    double sp = sin(angle.pitch);
//    double cy = cos(angle.yaw);
//    double sy = sin(angle.yaw);
//    rotationMatrix <<
//        cp * cy,                -cp * sy,               sp,
//        cr * sy + sr * sp * cy, cr * cy - sr * sp * sy, -sr * cp,
//        sr * sy - cr * sp * cy, sr * cy + cr * sp * sy, cr * cp;

    // gyro陀螺仪坐标系：x轴向后，y轴向左，z轴向下
    // camera相机坐标系：x轴向右，y轴向下，z轴向前

    // 绕x轴旋转矩阵：
    // [1, 0, 0]
    // [0, cos, -sin]
    // [0, sin, cos]
    // 绕y轴旋转矩阵：
    // [cos, 0, sin]
    // [0, 1, 0]
    // [-sin, 0, cos]
    // 绕z轴旋转矩阵：
    // [cos, -sin, 0]
    // [sin, cos, 0]
    // [0, 0, 1]

    // 陀螺仪坐标系到相机坐标系的旋转矩阵
    // 绕x轴旋转90°再绕y轴旋转-90°

    // 绕x轴旋转90矩阵：
    // [1, 0, 0]
    // [0, 0, -1]
    // [0, 1, 0]
    // 绕y轴旋转-90矩阵：
    // [0, 0, 1]
    // [0, 1, 0]
    // [-1, 0, 0]
    // q: 为什么不是[0, 0, -1]，[0, 1, 0]，[1, 0, 0]？
    // a: 因为相机坐标系的z轴向前，所以绕x轴旋转90°后，相机坐标系的z轴向下，所以绕y轴旋转-90°后，相机坐标系的z轴向左

//    Matrix3d gyro2Camera;
//    gyro2Camera << 0, 0, 1,
//            -1, 0, 0,
//            0, -1, 0;

    //以下方法用于将旋转矩阵从陀螺仪坐标系转换到相机坐标系
    rotationMatrix.row(1).swap(rotationMatrix.row(2));
    rotationMatrix.col(1).swap(rotationMatrix.col(2));
    rotationMatrix.transposeInPlace();

//    rotationMatrix << gyro2Camera * rotationMatrix;

    return rotationMatrix;
}

//int main(int argc, char *argv[]) {
//    // 初始化Azure Kinect设备
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
////    uint64_t dt = 1.0 / 30; //采样间隔，单位秒
//    uint64_t dt;
//
//    // 通过陀螺仪数据计算出的欧拉角
//    cas::EulerAngle prevAngle(0, 0, 0);
//
//    uint64_t temp = 0;
//    // 持续输出设备的姿态信息
//    while (true) {
//        //捕获一针
//        k4a_capture_t capture;
//        k4a_device_get_capture(device.handle(), &capture, K4A_WAIT_INFINITE);
//
//        string s;
//        k4a_imu_sample_t imu_sample;
//
////        k4a_device_get_imu_sample(device.handle(), &imu_sample, K4A_WAIT_INFINITE);
//        device.get_imu_sample(&imu_sample);
//
//        // 陀螺仪信息
//        // gyro_sample.xyz：陀螺仪的三轴角速度数据，单位为弧度每秒
////        printf(" |x:%.4f y:%.4f z: %.4f| \n",
////               imu_sample.gyro_sample.xyz.x, imu_sample.gyro_sample.xyz.y, imu_sample.gyro_sample.xyz.z);
//
//        float gx = imu_sample.gyro_sample.xyz.x;
//        float gy = imu_sample.gyro_sample.xyz.y;
//        float gz = imu_sample.gyro_sample.xyz.z;
//
//        //单位微妙，转换为秒
//        //uint64_t转换为double
//        dt = (imu_sample.acc_timestamp_usec - temp);
//        float dtf = (float) dt / 1000000;
//        temp = imu_sample.acc_timestamp_usec;
//        if (temp == 0) {
//            continue;
//        }
//        prevAngle = calculateOrientation(gx, gy, gz, dtf, prevAngle);
////        printf("roll:%.4f\tpitch:%.4f\tyaw:%.4f\tdt:%f\n", prevAngle.roll, prevAngle.pitch, prevAngle.yaw, dtf);
////        printf("roll:%.4f\tpitch:%.4f\tyaw:%.4f\n", prevAngle.roll, prevAngle.pitch, prevAngle.yaw);
//    }
//}
