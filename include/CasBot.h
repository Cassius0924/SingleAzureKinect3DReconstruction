#ifndef _MYCOBOT_H_
#define _MYCOBOT_H_

#include "fcntl.h"
#include "pthread.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "termios.h"
#include "unistd.h"
#include <errno.h>
#include <string>
#include <time.h>

#include <sys/select.h>

using namespace std;

// 通信指令帧的定义
#define FLAG_START 0xfe
#define FLAG_END 0xfa

typedef unsigned char ComuType;
// 指令类型帧的定义
#define FREEMODE 0x13           // 自由模式
#define READANGLE 0x20          // 读取所有舵机的角度，不包括机械爪
#define POST_ALL_ANGLE 0x22     // 发送全部角度
#define READ_GRIPPER_ANGLE 0x65 // 读取夹爪角度
#define SET_GRIPPER_MODE 0x66   // 设置夹爪模式，张开和闭合两种模式
#define SET_GRIPPER_ANGLE 0x67  // 设置夹爪角度

#define READ_ALL_COORD 0x23     // 读取当前位置坐标
#define SET_ALL_COORD 0x25      // 发送位置坐标

namespace cas {
    namespace bot {
        // int initArm(string serial_port_name);
        // int resetCobot(int fd);
        // int initMotor(string serial_port_name);
        // int resetMotor(int fd);
        // bool rotateMotor(int fd, string direction);
        // bool forwardRotateMotor(int fd);
        // bool reverseRotateMotor(int fd);
        class BotArm {
        private:
            int fd;

        public:
            BotArm(string serial_port_name);
            bool reset();
            bool execute(unsigned char *data_buffer, int length);
        };

        class BotMotor{
        private:
            int fd;
            unsigned char buffer[9];

        public:
            BotMotor(string serial_port_name);
            bool rotate(string direction);
        };

        class BotCar {
        private:
            int fd;
            float speed; //
            float angle_speed;
            unsigned char buffer[10];

        public:
            BotCar(string serial_port_name);
            void setSpeed(const char speed_value, float scale = 1.0);
            bool moveForward();
            bool moveForwardTime(float time);
            bool moveForwardDistance(float distance);
            bool moveBackward();
            bool moveBackwardTime(float time);
            bool moveBackwardDistance(float distance);
            bool turnLeft();
            bool turnLeftTime(float time);
            bool turnLeftAngle(float angle);
            bool turnRight();
            bool turnRightTime(float time);
            bool turnRightAngle(float angle);
            bool turnAngle(float anlge);
            bool stopCar();
            bool executeMoveSequence(int *seq, int seq_length);
        };
    } // namespace bot
} // namespace cas

#endif //_MYCOBOT_MAIN_H_