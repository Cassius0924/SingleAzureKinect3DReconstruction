#define _MYCOBOT_MAIN_H_

#include "fcntl.h"
#include "pthread.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "termios.h"
#include "unistd.h"
#include <errno.h>
#include <time.h>

#include <sys/select.h>

/*通信指令帧的定义*/
#define FLAG_START 0xfe
#define FLAG_END 0xfa

typedef unsigned char ComuType;
/*指令类型帧的定义*/
#define FREEMODE 0x13           /*自由模式*/
#define READANGLE 0x20          /*读取所有舵机的角度，不包括机械爪*/
#define POST_ALL_ANGLE 0x22     /*发送全部角度*/
#define READ_Gripper_ANGLE 0x65 /*读取夹爪角度*/
#define SET_Gripper_Mode 0x66   /*设置夹爪模式，张开和闭合两种模式*/
#define SET_Gripper_ANGLE 0x67  /*设置夹爪角度*/


#define READ_ALL_COORD 0x23 /*读取当前位置坐标*/
#define SET_ALL_COORD 0x25  /*发送位置坐标*/


namespace cas {
    namespace bot {
        int armInit();
        int resetCobot(int fd); /*f复位机械臂*/
    }                           // namespace bot
}// namespace cas