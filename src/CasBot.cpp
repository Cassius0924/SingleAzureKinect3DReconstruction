#include "CasBot.h"

#include <iostream>

#define DEBUG /*调试模式*/

using namespace std;

void usSleep(unsigned int nusecs) {
    struct timeval tval;

    tval.tv_sec = nusecs / 1000000;
    tval.tv_usec = nusecs % 1000000;
    select(0, NULL, NULL, NULL, &tval);
}


void forword_one_step(int fd_car, int fd_arm) {

    unsigned char writebuff[18];

    int i;
    /*
	前进 5s 后停止

	*/

    displacement(1, fd_car);
    usSleep(6000000);
    displacement(4, fd_car);


    /*
		FE FE 0F 22 00 00 00 00 00 00 00 00 00 00 00 00 1E FA

		FE FE 0F 22 00 00 00 00 00 00 DC D8 00 00 00 00 1E FA

		FE FE 0F 22 00 00 00 00 DC D8 00 00 00 00 00 00 1E FA

		FE FE 0F 22 00 00 DC D8 00 00 23 28 00 00 00 00 1E FA

		FE FE 0F 22 00 00 DC D8 00 00 00 00 00 00 00 00 1E FA

	*/


    //采集温度、湿度、气体数据并发送


    writebuff[0] = 0xfe;
    writebuff[1] = 0xfe;
    writebuff[2] = 0x0f;
    writebuff[3] = 0x22;


    writebuff[16] = 0x1e;
    writebuff[17] = 0xfa;


    //第一档


    writebuff[4] = 0;//1
    writebuff[5] = 0;

    writebuff[6] = 0;//2
    writebuff[7] = 0;

    writebuff[8] = 0;//3
    writebuff[9] = 0;

    writebuff[10] = 0;//4
    writebuff[11] = 0;

    writebuff[12] = 0;//5
    writebuff[13] = 0;

    writebuff[14] = 0;//6
    writebuff[15] = 0;


    write(fd_arm, writebuff, 18);

    for (i = 0; i < 18; i++)
        printf("%02X ", writebuff[i]);
    printf("\n");

    printf("第一档数据包发送完毕！\n");
    usSleep(3000000);


    //第二档


    writebuff[4] = 0;//1
    writebuff[5] = 0;

    writebuff[6] = 0;//2
    writebuff[7] = 0;

    writebuff[8] = 0;//3
    writebuff[9] = 0;

    writebuff[10] = 0x23;//4
    writebuff[11] = 0x28;

    writebuff[12] = 0;//5
    writebuff[13] = 0;

    writebuff[14] = 0;//6
    writebuff[15] = 0;


    write(fd_arm, writebuff, 18);

    for (i = 0; i < 18; i++)
        printf("%02X ", writebuff[i]);
    printf("\n");

    printf("第二档数据包发送完毕！\n");
    usSleep(3000000);


    //第三档


    writebuff[4] = 0;//1
    writebuff[5] = 0;

    writebuff[6] = 0;//2
    writebuff[7] = 0;

    writebuff[8] = 0x23;//3
    writebuff[9] = 0x28;

    writebuff[10] = 0;//4
    writebuff[11] = 0;

    writebuff[12] = 0;//5
    writebuff[13] = 0;

    writebuff[14] = 0;//6
    writebuff[15] = 0;


    write(fd_arm, writebuff, 18);

    for (i = 0; i < 18; i++)
        printf("%02X ", writebuff[i]);
    printf("\n");

    printf("第三档数据包发送完毕！\n");
    usSleep(3000000);


    //第四档


    writebuff[4] = 0;//1
    writebuff[5] = 0;

    writebuff[6] = 0x23;//2
    writebuff[7] = 0x28;

    writebuff[8] = 0;//3
    writebuff[9] = 0;

    writebuff[10] = 0xdc;//4
    writebuff[11] = 0xd8;

    writebuff[12] = 0;//5
    writebuff[13] = 0;

    writebuff[14] = 0;//6
    writebuff[15] = 0;


    write(fd_arm, writebuff, 18);

    for (i = 0; i < 18; i++)
        printf("%02X ", writebuff[i]);
    printf("\n");

    printf("第四档数据包发送完毕！\n");
    usSleep(3000000);


    //第五档


    writebuff[4] = 0;//1
    writebuff[5] = 0;

    writebuff[6] = 0x23;//2
    writebuff[7] = 0x28;

    writebuff[8] = 0;//3
    writebuff[9] = 0;

    writebuff[10] = 0;//4
    writebuff[11] = 0;

    writebuff[12] = 0;//5
    writebuff[13] = 0;

    writebuff[14] = 0;//6
    writebuff[15] = 0;


    write(fd_arm, writebuff, 18);

    for (i = 0; i < 18; i++)
        printf("%02X ", writebuff[i]);
    printf("\n");

    printf("第五档数据包发送完毕！\n");
    usSleep(3000000);

    mysys_resetMycobot(fd_arm);
}


int displacement(int jud, int fd_car) {

    unsigned char writebuff[32] = {0};

    writebuff[0] = 0xff;
    writebuff[1] = 0xfe;

    int i = 0;


    switch (jud) {

        case 1:
            writebuff[2] = 0x17;
            writebuff[3] = 0x17;
            break;
        case 2:
            writebuff[2] = 0;
            writebuff[3] = 0x17;
            break;
        case 3:
            writebuff[2] = 0x17;
            writebuff[3] = 0;
            break;
        case 4:
            writebuff[2] = 0;
            writebuff[3] = 0;
            break;
        default:
            break;
    }


    writebuff[4] = 0;
    writebuff[5] = 0;

    writebuff[6] = 0;
    writebuff[7] = 0;
    writebuff[8] = 0;
    writebuff[9] = 0;


    write(fd_car, writebuff, 10);


    printf("Jetson Serial Sent to Robot:");

    for (i = 0; i < (10); i++)
        printf("%02X ", writebuff[i]);
    printf("\n");
}


int str_to_int(char x) {
    if (x >= 'A' && x <= 'F') {
        return (int) x - 55;
    } else {
        return (int) x - 48;
    }
}


/*
开始伺教
1、循环读角度，获取到数据文件fd_txt
2、将fd_txt发回上位机
3、发送结束信号
4、返回(void *) 0
*/
void drag_Up_DropTeach_Mode(int fd, int client_fd) {

    char *tmpbuf;

    char recvbuff[32], ch;
    DragAndDrop_DATA draganddrop_data; /*相关数据结构体*/


    /*开始信号：0xff 0xff 0x0a 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x0a 0xfa*/
    unsigned char *mode_1;
    mode_1 = (unsigned char *) "0xff 0xff 0x0a 0x0a 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xfa";


    //pthread_t ptid, ptid2;/*储存子线程的ID ，创建子线程读取串口 同时存到文件中*/
    mysys_resetMycobot(fd); /*初始化机械臂*/
    draganddrop_data.fd = fd;

    /*将读取到的数据放到fd_txt文件里*/
    int fd_txt = 0;
    //int fd_txt = mysys_DumpSerial(&draganddrop_data);

    printf("请输入xxx结束:");
    //scanf("%s",);
    while (1) {
        memset(recvbuff, 0, 32);
        fgets(recvbuff, 32, stdin);
        if (strstr(recvbuff, "xxx") != NULL)
            break;
    }
    printf("饲教结束...");

    usleep(1000 * 500);


    /*向上发送数据文件fd_txt*/
    memset(tmpbuf, 0, 1024 * 20);

    strcat(tmpbuf, "0xff 0xff 0x0a 0x0a 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xfa");
    printf("tmpbuf = : %s \n", tmpbuf);

    fd_txt = open(draganddrop_data.datafile, O_RDONLY);
    int retlen = read(fd_txt, tmpbuf + 23, 1024 * 20); /*六个舵机，12字节 假设一次性可以正确读取所有坐标 且没有出现丢失*/
    printf("读取到的字符串长度为：%d \n", retlen);
    printf("tmpbuf = : %s \n", tmpbuf);


    strcat(tmpbuf, "0xff 0xff 0x0a 0x0a 0x0b 0x0b 0x00 0x00 0x00 0x00 0x00 0xfa");
    printf("tmpbuf = : %s \n", tmpbuf);


    write(client_fd, tmpbuf, 1024 * 20); /*写到网络对端*/
    printf("机械臂执行完毕！");
    usleep(1000 * 500);

    return;
}

int post_All_Coord(int fd, unsigned char databuff[], unsigned char sp) {
    databuff[12] = 0x1e; /*速度*/
    if (sp != 0) {
        databuff[12] = sp; /*设置速度值*/
    }
    mysys_Serial_TX(fd, POST_ALL_ANGLE, databuff);
    return 0;
}


void mysys_RightArmAutoRealy(unsigned char *databuff) {
    int i = 0;
    int new_angle = 0, temp = 0;//解析到原始的角度在取反后
    if (databuff == NULL) return;
    for (i = 1; i <= 5; i++)//只处理1-5号机位
    {

        /*
		joint_no取值范围: 1~6
		angle_high：数据类型byte
		计算方式：角度值乘以100 先转换成int形式 再取十六进制的高字节
		angle_low：数据类型byte
		计算方式：角度值乘以100 先转换成int形式 再取十六进制的低字节

		说明：上述的计算方式，是顺时针角度（从舵机的视角来说，3号舵机反向）；
		若设置逆时针角度，角度值乘100，然后用65536减去。转换成int形式 再取十六进制的高字节、低字节。

		如何得出关节最大角度
		temp = angle1_high*256 + angle1_low
		Angle1=（temp \ 33000 ?(temp – 65536) : temp）/10
		计算方式：角度值低位 + 角度高位值乘以256
		先判断是否大于33000 如果大于33000就再减去65536 最后除以10 如果小于33000就直接除以10
		*/
        temp = databuff[(i - 1) * 2] * 0x100 + databuff[(i - 1) * 2 + 1];//0x100 = 256
        new_angle = ((temp > 33000 ? (temp - 65536) : temp) / 100) * -1; //新角度值

        databuff[(i - 1) * 2] = (unsigned char) ((((new_angle * 100) + 65536) % 65536) / 0x100);    //新角度数据值的高字节
        databuff[(i - 1) * 2 + 1] = (unsigned char) ((((new_angle * 100) + 65536) % 65536) % 0x100);//新角度数据值的低字节
    }
    return;
}

/*串口发送  ComuType为类型  databuff中存储数据*/
int mysys_Serial_TX(int fd, ComuType comutype, unsigned char databuff[]) {

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
            mysys_RightArmAutoRealy(databuff); /*右臂角度自适应*/
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


/*网络控制模式
fd：机械臂文件描述符
线程（Main）：
	1、初始化服务器Socket()；
	2、阻塞监听客户端连接
	3、创建子线程负责网络通信
	4、从链式队列摘除数据包，
	5、解析、执行数据包
	6、回传执行状态数据包

子线程：
	1、监听数据，将每个数据包验证后入队；
	2、入队后，回传所接收数据包的确认
*/

/*复位机械臂*/
int mysys_resetMycobot(int fd) {
    unsigned char databuff[16] = {0};
    memset(databuff, 0, 16);
    databuff[12] = 0x1e; /*速度*/
    mysys_Serial_TX(fd, POST_ALL_ANGLE, databuff);
#ifdef DEBUG
    printf("机械臂复位成功！\n");
#endif
    return 0;
}

/*发送全部角度给机械臂，不包括机械爪*/
int mysys_PostAllAngle(int fd, unsigned char databuff[], unsigned char sp) {
    databuff[12] = 0x1e; /*速度*/
    if (sp != 0) {
        databuff[12] = sp; /*设置速度值*/
    }
    mysys_Serial_TX(fd, POST_ALL_ANGLE, databuff);
    return 0;
}

int mysys_sensor_init() {
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
    fd_device = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY); /*读写方式打开串口*/


    //set UART
    if (fd_device < 0) {
        printf("设备连接失败！fd_sensor = %d\n", fd_device);
        perror("sensor Serial open failed...");
        return -1;
    }
    printf("sensor 设备连接成功 fd_sensor = %d\n", fd_device);

    if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0) {
        printf("设备连接初始化失败！\n");
        perror("sensor Serial init failed...");
        return -2;
    }

    printf("sensor 设备连接初始化成功！\n");
    return fd_device; /*返回文件描述符*/
}


//对机械臂串口初始化
int mysys_arm_init() {
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
        printf("设备连接失败！fd_robot_arm = %d\n", fd_device);
        perror("Serial open failed...");
        return -1;
    }
    printf("设备连接成功 fd_robot_arm = %d\n", fd_device);

    if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0) {
        printf("设备连接初始化失败！\n");
        perror("Serial init failed...");
        return -2;
    }
    printf("设备连接初始化成功！\n");
    return fd_device; /*返回文件描述符*/
}


//对底盘车串口初始化
int mysys_car_init() {
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
        printf("设备连接失败！fd_car = %d\n", fd_device);
        perror("Serial open failed...");
        return -1;
    }
    printf("设备连接成功 fd_car = %d\n", fd_device);

    if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0) {
        printf("设备连接初始化失败！\n");
        perror("Serial init failed...");
        return -2;
    }
    printf("设备连接初始化成功！\n");
    return fd_device; /*返回文件描述符*/
}


int mysys_network_init() {
    return -1;
}
