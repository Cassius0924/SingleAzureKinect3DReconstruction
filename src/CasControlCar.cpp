// #include "CasAzureKinect.h"
#include "CasBot.h"
#include "CasSoundSourceLocalization.h"

#include <iostream>
#include <ncurses.h>
#include <string>

using namespace std;

int main() {
    // 控制小车移动
    cas::bot::BotCar bot_car("/dev/ttyUSB0");
    bot_car.setSpeed((char)0x12);
    int fd = 3;
    // int fd = cas::bot::initBotCar("/dev/ttyUSB1");

    unsigned char buffer[10] = {0};
    buffer[0] = 0xff;
    buffer[1] = 0xfe;
    buffer[2] = 0x00;   // A电机速度
    buffer[3] = 0x00;   // B电机速度
    buffer[4] = 0x00;   // A电机方向
    buffer[5] = 0x00;   // B电机方向
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    buffer[8] = 0x00;
    buffer[9] = 0x00;
    // W, A, S, D分别对应前进，左转，后退，右转
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    char ch;
    while (true) {
        ch = getch();// 获取按下的键值
        if (ch == 'w' || ch == 'W') {
            std::cout << "向前移动" << std::endl;
            buffer[2] = 0x12;
            buffer[3] = 0x12;
            buffer[4] = 0x01;
            buffer[5] = 0x01;
            write(fd, buffer, 10);
        } else if (ch == 'a' || ch == 'A') {
            std::cout << "向左转动" << std::endl;
            bot_car.turnLeftAngle(45);
        } else if (ch == 's' || ch == 'S') {
            std::cout << "向后移动" << std::endl;
            buffer[2] = 0x00;
            buffer[3] = 0x00;
            buffer[4] = 0x01;
            buffer[5] = 0x01;
            write(fd, buffer, 10);
        } else if (ch == 'd' || ch == 'D') {
            std::cout << "向右转动" << std::endl;
            bot_car.turnRightAngle(45);
        } else {
            std::cout << "未知操作" << std::endl;
        }
    }
}

