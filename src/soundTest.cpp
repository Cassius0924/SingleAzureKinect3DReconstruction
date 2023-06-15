// #include "CasAzureKinect.h"
#include "CasBot.h"
// #include "CasSoundSourceLocalization.h"

#include <iostream>

#include <open3d/Open3D.h>
#include <DataMessage.pb.h>
#include <string>

using namespace std;

int main() {
    int fd = cas::bot::initMotor("dev/ttyUSB0");
    auto final_cloud = make_shared<open3d::geometry::PointCloud>();
    cas::proto::DataMessage data_message;
    data_message.set_type(cas::proto::DataMessage::SOUND_SOURCE);
    // 控制小车移动
    // int fd = cas::bot::initBotCar("/dev/ttyUSB0");
    // int c = 0;

    // unsigned char buffer[10] = {0};
    // while (true) {
    //     cin >> c;
    //     if (c == 1) {
    //         buffer[0] = 0xff;
    //         buffer[1] = 0xfe;
    //         buffer[2] = 0x12;
    //         buffer[3] = 0x14;
    //         buffer[4] = 0x01;
    //         buffer[5] = 0x00;
    //         buffer[6] = 0x00;
    //         buffer[7] = 0x00;
    //         buffer[8] = 0x00;
    //         buffer[9] = 0x00;
    //     } else if (c == 0) {
    //         buffer[0] = 0xff;
    //         buffer[1] = 0xfe;
    //         buffer[2] = 0x00;
    //         buffer[3] = 0x00;
    //         buffer[4] = 0x00;
    //         buffer[5] = 0x00;
    //         buffer[6] = 0x00;
    //         buffer[7] = 0x00;
    //         buffer[8] = 0x00;
    //         buffer[9] = 0x00;
    //     }
        int c = 1;
        // if (write(fd, buffer, 10) < 0) {
        if (write(fd, &c, 1) <0) {
            cout << "发送失败" << endl;
        } else {
            cout << "发送成功" << endl;
        }
    // }


    // cas::ssl::SoundSourceDetector sound_source_detector(44100, 2205, 7 ,"plughw:2,0");
    // sound_source_detector.start();
    // while (true) {
    //     // unique_lock<mutex> lock(ssl_mutex);
    //     Eigen::Vector3f sound_source = sound_source_detector.locate();
    //     cout << "声源位置: " << sound_source[0] << ", " << sound_source[1] << ", " << sound_source[2] << endl;
    //     // return 0;
    // }
}