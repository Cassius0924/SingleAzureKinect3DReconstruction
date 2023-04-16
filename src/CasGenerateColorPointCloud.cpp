//

// Created by HoChihChou on 4/9/23.
//

#include "CasAzureKinectExtrinsics.h"
#include "CasPointCloud.h"

#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <k4a/k4a.h>
#include <open3d/Open3D.h>

// xy_table 是一个二维数组，用于存储每个像素点对应的三维坐标
// 此函数的作用是将深度图像转换为三维坐标
static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table) {
    k4a_float2_t *table_data = (k4a_float2_t *) (void *) k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++) {
        p.xy.y = (float) y;
        for (int x = 0; x < width; x++, idx++) {
            p.xy.x = (float) x;

            // 将像素点转换为三维坐标
            k4a_calibration_2d_to_3d(
                    calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            // 如果 valid 为 0，说明该点不在相机的视野范围内
            if (valid) {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            } else {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

static void generate_point_cloud(const k4a_image_t depth_image,const k4a_image_t color_image, const k4a_image_t xy_table,
                     k4a_image_t point_cloud, int *point_count) {
    // 获取图像的宽度和高度
    int width = k4a_image_get_width_pixels(depth_image);
    int height = k4a_image_get_height_pixels(depth_image);

//    cout << "Depth width: " << width << endl;
//    cout << "Depth height: " << height << endl;
//    cout << "Color width: " << k4a_image_get_width_pixels(color_image) << endl;
//    cout << "Color height: " << k4a_image_get_height_pixels(color_image) << endl;

    uint16_t *depth_data = (uint16_t *) (void *) k4a_image_get_buffer(depth_image);
    k4a_float2_t *xy_table_data = (k4a_float2_t *) (void *) k4a_image_get_buffer(xy_table);
    k4a_float3_t *point_cloud_data = (k4a_float3_t *) (void *) k4a_image_get_buffer(point_cloud);

    *point_count = 0;
    for (int i = 0; i < width * height; i++) {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y)) {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float) depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float) depth_data[i];
            point_cloud_data[i].xyz.z = (float) depth_data[i];

            (*point_count)++;
        } else {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }
}

static shared_ptr<open3d::geometry::PointCloud> generate_o3d_point_cloud(const k4a_image_t color_image, const k4a_image_t point_cloud){
    // 获取图像的宽度和高度
    int width = k4a_image_get_width_pixels(color_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t *point_cloud_data = (int16_t *) (void *) k4a_image_get_buffer(point_cloud);
    uint8_t *color_image_data = (uint8_t *) (void *) k4a_image_get_buffer(color_image);

    auto pcd = make_shared<open3d::geometry::PointCloud>();
    for(int i = 0; i < width * height; i++) {
        if(!isnan(point_cloud_data[i * 3]) && !isnan(point_cloud_data[i * 3 + 1]) && !isnan(point_cloud_data[i * 3 + 2])) {
            pcd->points_.push_back(Eigen::Vector3d(point_cloud_data[i * 3] / 1000.0, point_cloud_data[i * 3 + 1] / 1000.0, point_cloud_data[i * 3 + 2] / 1000.0));
            pcd->colors_.push_back(Eigen::Vector3d(color_image_data[i * 4 + 2] / 255.0, color_image_data[i * 4 + 1] / 255.0, color_image_data[i * 4] / 255.0));
        } else {
            pcd->points_.push_back(Eigen::Vector3d(nan(""), nan(""), nan("")));
            pcd->colors_.push_back(Eigen::Vector3d(nan(""), nan(""), nan("")));
        }
    }
    return pcd;
}

static void write_point_cloud(const char *file_name, const k4a_image_t point_cloud, int point_count) {
    int width = k4a_image_get_width_pixels(point_cloud);
    int height = k4a_image_get_height_pixels(point_cloud);

    k4a_float3_t *point_cloud_data = (k4a_float3_t *) (void *) k4a_image_get_buffer(point_cloud);

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex" << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < width * height; i++) {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z)) {
            continue;
        }

        ss << (float) point_cloud_data[i].xyz.x << " " << (float) point_cloud_data[i].xyz.y << " "
           << (float) point_cloud_data[i].xyz.z << std::endl;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize) ss.str().length());

    printf("Saved point cloud to %s\n", file_name);
}

int main(int argc, char **argv) {
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    k4a_capture_t capture = NULL;
    int file_count = 1;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t xy_table = NULL;
    k4a_image_t point_cloud = NULL;
    k4a_imu_sample_t imu_sample;

    int color_image_width = k4a_image_get_width_pixels(color_image);
    int color_image_height = k4a_image_get_height_pixels(color_image);

    float prev_radian = 0;

    cas::EulerAngle prevAngle(0, 0, 0);

    float temp = 0;

    int point_count = 0;

    if (!cas::openAzureKinectDevice(&device)) {
        goto Exit;
    }

    if (!cas::getAzureKinectCalibration(device, config, &calibration)) {
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

    // 此步用于创建一个图像，用于存储深度图像的坐标映射表
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                     color_image_width,
                     color_image_height,
                     color_image_width * (int) sizeof(uint16_t),
                     &xy_table);

//    create_xy_table(&calibration, xy_table);

    // 此步用于创建一个图像，用于存储点云数据
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     color_image_width,
                     color_image_height,
                     color_image_width* 3 * (int) sizeof(int16_t),
                     &point_cloud);

    k4a_transformation_depth_image_to_color_camera(transformation, depth_image, xy_table);

    k4a_transformation_depth_image_to_point_cloud(transformation, depth_image, K4A_CALIBRATION_TYPE_DEPTH, point_cloud);


    if (!cas::configureAzureKinectDevice(device, config)) {
        goto Exit;
    }

    if (!cas::startAzureKinectImu(device)) {
        goto Exit;
    }

    while (cas::getAzureKinectImuSample(device, &imu_sample, TIMEOUT_IN_MS)) {

        float gx = imu_sample.gyro_sample.xyz.x;
        float gy = imu_sample.gyro_sample.xyz.y;
        float gz = imu_sample.gyro_sample.xyz.z;

        float dt = (float) imu_sample.acc_timestamp_usec - temp;
        float dtf = (float) dt / 1000000;
        temp = (float) imu_sample.acc_timestamp_usec;
        if (temp == 0) {
            continue;
        }

        prevAngle = calculateOrientation(gx, gy, gz, dtf, prevAngle);
//        float r = prevAngle.roll * 180 / M_PI;
//        float p = prevAngle.pitch * 180 / M_PI;
//        float y = prevAngle.yaw * 180 / M_PI;

//        printf("roll:%.4f\tpitch:%.4f\tyaw:%.4f\n", prevAngle.roll, prevAngle.pitch, prevAngle.yaw);
//        printf("roll:%.4f %.4f\tpitch:%.4f %.4f\tyaw:%.4f %.4f\n", prevAngle.roll, r, prevAngle.pitch, p, prevAngle.yaw, y);

        if (abs(prevAngle.yaw - prev_radian) > 0.3491) {
            prev_radian = prevAngle.yaw;
            printf("saving...\n");
            if (!cas::getAzureKinectCapture(device, &capture, TIMEOUT_IN_MS)) {
                goto Exit;
            }

            if (!cas::getAzureKinectDepthImage(capture, &depth_image)) {
                goto Exit;
            }

            if (!cas::getAzureKinectColorImage(capture, &color_image)) {
                goto Exit;
            }

//            auto source = std::make_shared<open3d::geometry::PointCloud>();
//            generate_point_cloud(depth_image, color_image, xy_table, point_cloud, &point_count);
//            cas::o3d::k4a_image_to_o3d_point_cloud(point_cloud, source);
//            point_cloud, color_image
            auto source = generate_o3d_point_cloud(color_image, point_cloud);

//            Eigen::Vector3d center = Eigen::Vector3d::Zero();
//            source->Rotate(cas::eulerAngle2RotationMatrix(prevAngle), center);
            std::string file_name = "reg/" + to_string(file_count) + ".ply";
            open3d::io::WritePointCloud(file_name, *source);
//            write_point_cloud(file_name.c_str(), point_cloud, point_count);


            //打印旋转角
            cout << "旋转角:" + to_string(prevAngle.roll) + " " + to_string(prevAngle.pitch) + " " +
                    to_string(prevAngle.yaw) << endl;
            //打印旋转矩阵
            cout << cas::eulerAngle2RotationMatrix(prevAngle) << endl;

            file_count++;
        }

    }

    k4a_image_release(color_image);
    k4a_image_release(depth_image);
    k4a_capture_release(capture);
    k4a_image_release(xy_table);
    k4a_image_release(point_cloud);

    returnCode = 0;
    Exit:
    if (device != NULL) {
        k4a_device_close(device);
    }

    return returnCode;
}
