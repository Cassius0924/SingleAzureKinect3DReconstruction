//#ifndef _MYCOBOT_MAIN_H_
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

// #include "mycobot_socket.h"
// #include "A.Struct.pb.h"

/* According to POSIX.1-2001 */
#include <sys/select.h>


/*拖拽功能结构体*/
typedef struct Sensor_DATA {
    tm *p;//当前时间

    int x;//点位  x
    int y;//点位  y

    int mod;//档位  1~5

    int temp_high;//温度
    int temp_low;
    int humi_high;//湿度
    int humi_low;
    int co;     //CO
    int ham_gas;//有害气体

} Sensor_DATA;


/*拖拽功能结构体*/
typedef struct DragAndDrop_DATA {
    char datafile[64]; /*机械臂数据文件名*/
    int fd;            /*机械臂串口文件描述符*/
    int fd_txt;        /*数据文件 文件描述符*/

} DragAndDrop_DATA;


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


//#define SERIAL_NAME "/dev/ttyUSB0" /*机械臂设备的串口号*/

//#define SENSOR_SERIAL_NAME "/dev/ttyUSB1" /*机械臂设备的串口号*/

int mysys_sensor_init();//机械臂设备的初始化，返回值：串口文件描述符；
int mysys_car_init();   //传感器设备的初始化，返回值：串口文件描述符；
int mysys_arm_init();
int mysys_network_init();//网络设备的初始化，返回值，网络文件描述符


//void sensor_data(int fd_sensor,int x,int y);

DragAndDrop_DATA sensor_data(int fd_sensor, int x, int y, int fd_txt);


// bool ProtobufEncode(std::string &strPb);
// void printSensor(DATA::Struct::Sensor &sensor);
// bool ProtobufDecode(std::string &strPb);

void forword_one_step(int fd_car, int fd_arm);

void usSleep(unsigned int nusecs);


/*

位移， 

jud = 1 前进一格
jud = 2 向右转
jud = 3 向左转

*/
int displacement(int jud, int fd_car);//


void drag_Up_DropTeach_Mode(int fd, int client_fd); /*网络控制模式*/
int str_to_int(char x);

// void aaa(int socket_fd, PDataPack_RecvQUEUE pdatapack_recvqueue);
void signal_Test_Mode(int robotfd, int client_fd);

int mysys_menu_(int fd_arm, int fd_car, int fd_sensor, int client_fd); /*系统菜单与导航*/


void mysys_code_mode(int fd_main, int argc, const char *args[]); /*程序调用模式*/
DragAndDrop_DATA dragAndDropTeach_Mode(int fd);                  /*本地拖拽伺教模式*/
int mysys_socketMain(int fd);                                    /*网络控制模式*/
void mysys_exit_(int fd_device);                                 /*关闭设备*/
int mysys_resetMycobot(int fd);                                  /*f复位机械臂*/

int mysys_PostAllAngle(int fd, unsigned char databuff[], unsigned char sp); /*发送全部角度给机械臂*/

void mysys_cleanhandle(void *argc); /*线程销毁处理函数*/

void *mysys_DumpSerial(void *p_draganddrop_data); /*拖拽伺教模式中的读取串口子线程*/

void *mysys_teachMode_Play(void *p_draganddrop_data); /*拖拽伺教模式中的机械臂执行子线程*/
void *mySockRecvStartThread(void *arg);               /*网络数据包接收 启动例程*/

int mysys_Serial_TX(int fd, ComuType comutype, unsigned char databuff[]); /*串口发送  ComuType为类型 databuff为数据帧*/
int mysys_Serial_RX(int cobot_fd, unsigned char *recvbuff);               /*机械臂的串口接收 recvbuff必须提供512大小，数据将存放在此*/
// void mysys_freeDataPack_recvQ(PDataPack_RecvQUEUE pdatapack_recvqueue);   /*释放数据包接收队列*/

/*左臂自适应，仿真左臂的动作，即在源数据的基础上，给1-5号舵机反向处理，在发送之前和接收时候调用，将所有的舵机角度数组给我即可
databuff中存储的六个舵机的数据值 共计12个字节
*/
void mysys_RightArmAutoRealy(unsigned char *databuff);
