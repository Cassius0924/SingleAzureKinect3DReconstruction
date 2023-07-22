// #include "CasAzureKinect.h"
#include "CasBot.h"
#include "CasSoundSourceLocalization.h"

#include <iostream>
#include <ncurses.h>
#include <string>
#include <thread>

using namespace std;

int main() {
    // 控制小车移动
    cas::bot::BotCar bot_car("/dev/ttyUSB1", (char)0x12, 0.62);
    // int fd = cas::bot::initBotCar("/dev/ttyUSB1");

    unsigned char buffer[10] = {0};
    buffer[0] = 0xff;
    buffer[1] = 0xfe;
    buffer[2] = 0x00; // A电机速度
    buffer[3] = 0x00; // B电机速度
    buffer[4] = 0x00; // A电机方向
    buffer[5] = 0x00; // B电机方向
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    buffer[8] = 0x00;
    buffer[9] = 0x00;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    char ch;

    // thread bot_car_thread([&]() {
    //     unsigned char bot_car_buffer[10];
    //     while (true) {
    //         // unique_lock<mutex> lock(car_mutex);
    //         //清空缓冲区
    //         memset(bot_car_buffer, 0, sizeof(bot_car_buffer));
    //         bot_car.recvData(bot_car_buffer, 10);
    //         if (bot_car_buffer[2] != 0) {
    //             for (int i = 0; i < 10; i++) {
    //                 // 打印十六进制
    //                 cout << hex << (int)bot_car_buffer[i] << " ";
    //             }
    //             cout << endl;
    //         }
    //     }
    // });
    // W, A, S, D分别对应前进，左转，后退，右转
    while (true) {
        ch = getch(); // 获取按下的键值
        if (ch == 'w' || ch == 'W') {
            std::cout << "向前移动" << std::endl;
            bot_car.moveForwardDistance(80);
        } else if (ch == 'a' || ch == 'A') {
            std::cout << "向左转动" << std::endl;
            // bot_car.turnLeftAngle(45);
            bot_car.autoTurnByAngle(179);
        } else if (ch == 's' || ch == 'S') {
            std::cout << "向后移动" << std::endl;
            bot_car.moveBackwardDistance(10);
        } else if (ch == 'd' || ch == 'D') {
            std::cout << "向右转动" << std::endl;
            // bot_car.turnRightAngle(45);
            bot_car.autoTurnByAngle(-45);
        } else if (ch == 'x' || ch == 'X') {
            std::cout << "停止移动" << std::endl;
            bot_car.stopCar();
        } else {
            std::cout << "未知操作" << std::endl;
        }
    }
}
