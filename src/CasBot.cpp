#include "CasBot.h"

#include <iostream>

#define DEBUG /*调试模式*/

using namespace std;

void rightArmAutoRealy(unsigned char *databuff) {
    int i = 0;
    int new_angle = 0, temp = 0;//解析到原始的角度在取反后
    if (databuff == NULL) return;
    for (i = 1; i <= 5; i++) {//只处理1-5号机位
        temp = databuff[(i - 1) * 2] * 0x100 + databuff[(i - 1) * 2 + 1];//0x100 = 256
        new_angle = ((temp > 33000 ? (temp - 65536) : temp) / 100) * -1; //新角度值

        databuff[(i - 1) * 2] = (unsigned char) ((((new_angle * 100) + 65536) % 65536) / 0x100);    //新角度数据值的高字节
        databuff[(i - 1) * 2 + 1] = (unsigned char) ((((new_angle * 100) + 65536) % 65536) % 0x100);//新角度数据值的低字节
    }
    return;
}

/*串口发送  ComuType为类型  databuff中存储数据*/
int serialTX(int fd, ComuType comutype, unsigned char databuff[]) {
    int ret = 0, i;
    unsigned char writebuff[32] = {0};

    writebuff[0] = writebuff[1] = FLAG_START;
    switch (comutype) {
            /*设置指令帧*/
        case FREEMODE:
            /*自由模式*/
            writebuff[2] = 0x02; /*数据长度帧*/
            writebuff[3] = comutype;
            break;
        case READANGLE:
            /*读取所有机械臂舵机角度*/
            writebuff[2] = 0x02;
            writebuff[3] = comutype;
            break;
        case POST_ALL_ANGLE:
            /*发送全部角度*/
            writebuff[2] = 0x0f;
            writebuff[3] = comutype;
            rightArmAutoRealy(databuff); /*右臂角度自适应*/
            memcpy(writebuff + 4, databuff, 13);
            break;
        case READ_Gripper_ANGLE:
            /*读取夹爪角度*/
            writebuff[2] = 0x02; /*数据长度帧，按照机械臂的协议来*/
            writebuff[3] = comutype;
            break;
        case SET_Gripper_Mode:
            /*设置夹爪模式与设置夹爪角度相同*/
        case SET_Gripper_ANGLE:
            /*设置夹爪角度*/
            writebuff[2] = 0x04; /*数据长度帧*/
            writebuff[3] = comutype;
            memcpy(writebuff + 4, databuff, 2);
            break;


            /*读取当前位置坐标*/
        case READ_ALL_COORD:
            writebuff[2] = 0x02;
            writebuff[3] = comutype;
            break;
            /*设置位置坐标*/

        case SET_ALL_COORD:
            writebuff[2] = 0x10;
            writebuff[3] = comutype;
            memcpy(writebuff + 4, databuff, 14);
            break;
        default:
            printf("发送类型未定义！\n");
            return -1;
    }
    writebuff[2 + writebuff[2]] = FLAG_END; /*根据协议中的数据长度帧填充结束帧*/
    write(fd, writebuff, (int) (writebuff[2] + 3));


    printf("Jetson Serial Sent to Robot:");
    if (writebuff[3] != SET_ALL_COORD) {
        for (i = 0; i < (writebuff[2] + 3); i++)
            printf("%02X ", writebuff[i]);
        printf("\n");
    } else {
        for (i = 0; i < 16; i++)
            printf("%02X ", writebuff[i]);
        printf("\n");
    }
    return ret;
}

/*复位机械臂*/
int cas::bot::resetCobot(int fd) {
    unsigned char databuff[16] = {0};
    memset(databuff, 0, 16);
    databuff[12] = 0x1e; /*速度*/
    serialTX(fd, POST_ALL_ANGLE, databuff);
    return 0;
}

//对机械臂串口初始化
int cas::bot::armInit() {
    int fd_device;
    struct termios newtio;
    tcgetattr(fd_device, &newtio);
    newtio.c_cflag &= ~CSIZE;         /*数据位屏蔽 将c_cflag全部清零*/
    newtio.c_cflag = B115200;         /*set bound*/
    newtio.c_cflag |= CS8;            /*数据位8*/
    newtio.c_cflag |= CLOCAL | CREAD; /*使驱动程序启动接收字符装置，同时忽略串口信号线的状态*/

    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); /*禁用软件流控制*/

    newtio.c_oflag &= ~OPOST; /*使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。*/

    /*在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。要使串口设备工作在原始模式，需要关闭ICANON、ECHO、ECHOE和ISIG选项，其操作方法如下：*/
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。*/

    /*4）当VMIN = 0，VTIME = 0时 如果有数据可用，则read最多返回所要求的字节数，如果无数据可用，则read立即返回0。MIN > 0 , TIME =0 READ 会等待,直到MIN字元可读
	*/
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    fd_device = open("/dev/ttyUSB0", O_RDWR); /*读写方式打开串口*/
    //set UART
    if (fd_device < 0) {
        cerr << "机械臂设备连接失败！arm_fd = " << fd_device << endl;
        cerr << "串口打开失败！" << endl;
        return -1;
    }
    cout << "机械臂设备连接成功 arm_fd = " << fd_device << endl;

    if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0) {
        printf("设备连接初始化失败！\n");
        cerr << "串口初始化失败！" << endl;
        return -2;
    }
    cout << "机械臂设备初始化成功！" << endl;
    return fd_device; /*返回文件描述符*/
}
