#include "CasBot.h"

#include <cmath>
#include <iostream>
#include <thread>

#define DEFAULT_SERIAL_PORT_NAME "/dev/ttyUSB0"
#define WHEEL_D 20 // 单位厘米（cm）
#define WHEEL_DISTANCE 53

using namespace std;

void rightArmAutoRealy(unsigned char *databuff) {
    int i = 0;
    int new_angle = 0, temp = 0; // 解析到原始的角度在取反后
    if (databuff == NULL) {
        return;
    }
    for (i = 1; i <= 5; i++) {                                                                  // 只处理1-5号机位
        temp = databuff[(i - 1) * 2] * 0x100 + databuff[(i - 1) * 2 + 1];                       // 0x100 = 256
        new_angle = ((temp > 33000 ? (temp - 65536) : temp) / 100) * -1;                        // 新角度值

        databuff[(i - 1) * 2] = (unsigned char)((((new_angle * 100) + 65536) % 65536) / 0x100); // 新角度数据值的高字节
        databuff[(i - 1) * 2 + 1] =
            (unsigned char)((((new_angle * 100) + 65536) % 65536) % 0x100); // 新角度数据值的低字节
    }
    return;
}

// 串口发送  ComuType为类型  databuff中存储数据
int serialTX(int fd, ComuType comutype, unsigned char databuff[]) {
    int ret = 0, i;
    unsigned char writebuff[32] = {0};

    writebuff[0] = writebuff[1] = FLAG_START;
    switch (comutype) {
            // 设置指令帧
        case FREEMODE:
            // 自由模式
            writebuff[2] = 0x02; // 数据长度帧
            writebuff[3] = comutype;
            break;
        case READANGLE:
            // 读取所有机械臂舵机角度
            writebuff[2] = 0x02;
            writebuff[3] = comutype;
            break;
        case POST_ALL_ANGLE:
            // 发送全部角度
            writebuff[2] = 0x0f;
            writebuff[3] = comutype;
            rightArmAutoRealy(databuff); // 右臂角度自适应
            memcpy(writebuff + 4, databuff, 13);
            break;
        case READ_GRIPPER_ANGLE:
            // 读取夹爪角度
            writebuff[2] = 0x02; // 数据长度帧，按照机械臂的协议来
            writebuff[3] = comutype;
            break;
        case SET_GRIPPER_MODE:
            // 设置夹爪模式与设置夹爪角度相同
        case SET_GRIPPER_ANGLE:
            // 设置夹爪角度
            writebuff[2] = 0x04; // 数据长度帧
            writebuff[3] = comutype;
            memcpy(writebuff + 4, databuff, 2);
            break;
            // 读取当前位置坐标
        case READ_ALL_COORD:
            writebuff[2] = 0x02;
            writebuff[3] = comutype;
            break;
            // 设置位置坐标
        case SET_ALL_COORD:
            writebuff[2] = 0x10;
            writebuff[3] = comutype;
            memcpy(writebuff + 4, databuff, 14);
            break;
        default:
            cerr << "发送类型未定义！" << endl;
            return -1;
    }
    writebuff[2 + writebuff[2]] = FLAG_END; // 根据协议中的数据长度帧填充结束帧
    write(fd, writebuff, (int)(writebuff[2] + 3));

    cout << "Jetson Serial Sent to Robot:";
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

cas::bot::BotArm::BotArm(string serial_port_name = DEFAULT_SERIAL_PORT_NAME) {
    struct termios newtio;
    tcgetattr(this->fd, &newtio);
    newtio.c_cflag &= ~CSIZE;         // 数据位屏蔽 将c_cflag全部清零
    newtio.c_cflag = B115200;         // set bound
    newtio.c_cflag |= CS8;            // 数据位8
    newtio.c_cflag |= CLOCAL | CREAD; // 使驱动程序启动接收字符装置，同时忽略串口信号线的状态
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
    newtio.c_oflag &= ~OPOST; // 使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。
    // 在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。要使串口设备工作在原始模式，需要关闭ICANON、ECHO、ECHOE和ISIG选项，其操作方法如下：
    newtio.c_lflag &=
        ~(ICANON | ECHO | ECHOE | ISIG); // 在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    this->fd = open(serial_port_name.c_str(), O_RDWR); // 读写方式打开串口
    if (this->fd < 0) {
        cerr << "机械臂设备连接失败！bot_arm_fd = " << this->fd << endl;
        cerr << "串口打开失败！" << endl;
        return;
    }
    cout << "机械臂设备连接成功！bot_arm_fd = " << this->fd << endl;

    if (tcsetattr(this->fd, TCSADRAIN, &newtio) != 0) {
        cerr << "串口初始化失败！" << endl;
        return;
    }
    cout << "机械臂设备初始化成功！" << endl;
}

bool cas::bot::BotArm::reset() {
    unsigned char databuff[16] = {0};
    memset(databuff, 0, 16);                      // 清空数组
    databuff[12] = 0x1e;                          // 速度
    serialTX(this->fd, POST_ALL_ANGLE, databuff); // 发送数据
    return true;
}

bool cas::bot::BotArm::execute(unsigned char *data_buffer, int length) {
    if (write(this->fd, data_buffer, length) < 0) {
        return false;
    }
    return true;
}

// 复位机械臂
// int cas::bot::resetCobot(int fd) {
//     unsigned char databuff[16] = {0};
//     memset(databuff, 0, 16);                // 清空数组
//     databuff[12] = 0x1e;                    // 速度
//     serialTX(fd, POST_ALL_ANGLE, databuff); // 发送数据
//     return 0;
// }

// 对机械臂串口初始化
// int cas::bot::initArm(string serial_port_name = DEFAULT_SERIAL_PORT_NAME) {
//     int fd_device;
//     struct termios newtio;
//     tcgetattr(fd_device, &newtio);
//     newtio.c_cflag &= ~CSIZE;         // 数据位屏蔽 将c_cflag全部清零
//     newtio.c_cflag = B115200;         // set bound
//     newtio.c_cflag |= CS8;            // 数据位8
//     newtio.c_cflag |= CLOCAL | CREAD; // 使驱动程序启动接收字符装置，同时忽略串口信号线的状态

//     newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制

//     newtio.c_oflag &= ~OPOST; // 使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。

//     //
//     在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。要使串口设备工作在原始模式，需要关闭ICANON、ECHO、ECHOE和ISIG选项，其操作方法如下：
//     newtio.c_lflag &=
//         ~(ICANON | ECHO | ECHOE | ISIG); //
//         在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。

//     // 4）当VMIN = 0，VTIME = 0时 如果有数据可用，则read最多返回所要求的字节数，如果无数据可用，则read立即返回0。MIN
//     > 0
//     // , TIME =0 READ 会等待,直到MIN字元可读

//     newtio.c_cc[VMIN] = 1;
//     newtio.c_cc[VTIME] = 0;
//     fd_device = open(serial_port_name.c_str(), O_RDWR); // 读写方式打开串口
//     // set UART
//     if (fd_device < 0) {
//         cerr << "机械臂设备连接失败！arm_fd = " << fd_device << endl;
//         cerr << "串口打开失败！" << endl;
//         return -1;
//     }
//     cout << "机械臂设备连接成功！arm_fd = " << fd_device << endl;

//     if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0) {
//         cerr << "串口初始化失败！" << endl;
//         return -2;
//     }
//     cout << "机械臂设备初始化成功！" << endl;
//     return fd_device; // 返回文件描述符
// }

cas::bot::BotMotor::BotMotor(string serial_port_name = DEFAULT_SERIAL_PORT_NAME) {
    struct termios newtio;
    tcgetattr(this->fd, &newtio);
    newtio.c_cflag &= ~CSIZE;         // 数据位屏蔽 将c_cflag全部清零
    newtio.c_cflag = B115200;         // set bound
    newtio.c_cflag |= CS8;            // 数据位8
    newtio.c_cflag |= CLOCAL | CREAD; // 使驱动程序启动接收字符装置，同时忽略串口信号线的状态
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
    newtio.c_oflag &= ~OPOST; // 使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。
    newtio.c_lflag &=
        ~(ICANON | ECHO | ECHOE | ISIG); // 在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    this->fd = open(serial_port_name.c_str(), O_RDWR); // 读写方式打开串口

    if (this->fd < 0) {
        cerr << "舵机设备连接失败！motor_fd = " << this->fd << endl;
        cerr << "串口打开失败！" << endl;
        return;
    }
    cout << "舵机设备连接成功！motor_fd = " << this->fd << endl;

    if (tcsetattr(this->fd, TCSADRAIN, &newtio) != 0) {
        cerr << "串口初始化失败！" << endl;
        return;
    }
    cout << "舵机设备初始化成功！" << endl;
    this->buffer[0] = '@';
    this->buffer[1] = 'X';
    this->buffer[2] = '0';
    this->buffer[3] = '0';
    this->buffer[4] = '0';
    this->buffer[5] = '0';
    this->buffer[6] = '0';
    this->buffer[7] = '\r';
    this->buffer[8] = '\n';
}

bool cas::bot::BotMotor::rotate(string direction) {
    if (direction == "F" || direction == "R") {
        this->buffer[1] = direction[0];
        this->buffer[2] = '0';
        this->buffer[3] = '8';
        this->buffer[4] = '5';
        this->buffer[5] = '5';
        this->buffer[6] = '0';
        if (write(this->fd, this->buffer, 9) < 0) {
            return false;
        }
    } else {
        return false;
    }
    return true;
}



// int cas::bot::initMotor(string serial_port_name = DEFAULT_SERIAL_PORT_NAME) {
//     int fd_device;
//     struct termios newtio;
//     tcgetattr(fd_device, &newtio);
//     newtio.c_cflag &= ~CSIZE;         // 数据位屏蔽 将c_cflag全部清零
//     newtio.c_cflag = B115200;         // set bound
//     newtio.c_cflag |= CS8;            // 数据位8
//     newtio.c_cflag |= CLOCAL | CREAD; // 使驱动程序启动接收字符装置，同时忽略串口信号线的状态
//     newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
//     newtio.c_oflag &= ~OPOST; // 使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。
//     newtio.c_lflag &=
//         ~(ICANON | ECHO | ECHOE | ISIG); //
//         在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。
//     newtio.c_cc[VMIN] = 1;
//     newtio.c_cc[VTIME] = 0;
//     fd_device = open(serial_port_name.c_str(), O_RDWR); // 读写方式打开串口

//     if (fd_device < 0) {
//         cerr << "舵机设备连接失败！motor_fd = " << fd_device << endl;
//         cerr << "串口打开失败！" << endl;
//         return -1;
//     }
//     cout << "舵机设备连接成功！motor_fd = " << fd_device << endl;

//     if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0) {
//         cerr << "串口初始化失败！" << endl;
//         return -2;
//     }
//     cout << "舵机设备初始化成功！" << endl;
//     return fd_device;
// }

// bool cas::bot::rotateMotor(int fd, string direction) {
//     if (direction == "F" || direction == "R") {
//         char buffer[9];
//         buffer[0] = '@';
//         buffer[1] = direction[0];
//         buffer[2] = '0';
//         buffer[3] = '8';
//         buffer[4] = '5';
//         buffer[5] = '5';
//         buffer[6] = '0';
//         buffer[7] = '\r';
//         buffer[8] = '\n';
//         if (write(fd, buffer, 9) < 0) {
//             return false;
//         }
//     } else {
//         return false;
//     }
//     return true;
// }

// bool cas::bot::forwardRotateMotor(int fd) {
//     char buffer[9];
//     buffer[0] = '@';
//     buffer[1] = 'F';
//     buffer[2] = '0';
//     buffer[3] = '8';
//     buffer[4] = '5';
//     buffer[5] = '5';
//     buffer[6] = '0';
//     buffer[7] = '\r';
//     buffer[8] = '\n';
//     if (write(fd, buffer, 9) < 0) {
//         return false;
//     }
//     return true;
// }

// bool cas::bot::reverseRotateMotor(int fd) {
//     char buffer[9];
//     buffer[0] = '@';
//     buffer[1] = 'R';
//     buffer[2] = '0';
//     buffer[3] = '8';
//     buffer[4] = '5';
//     buffer[5] = '5';
//     buffer[6] = '0';
//     buffer[7] = '\r';
//     buffer[8] = '\n';
//     if (write(fd, buffer, 9) < 0) {
//         return false;
//     }
//     return true;
// }

cas::bot::BotCar::BotCar(string serial_port_name = DEFAULT_SERIAL_PORT_NAME) {
    this->fd;
    struct termios newtio;
    tcgetattr(this->fd, &newtio);
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag = B115200;
    newtio.c_cflag |= CS8;
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
    newtio.c_oflag &= ~OPOST;
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    this->fd = open(serial_port_name.c_str(), O_RDWR); // 读写方式打开串口
    if (this->fd < 0) {
        cerr << "底盘车设备连接失败！bot_car_fd = " << this->fd << endl;
        cerr << "串口打开失败！" << endl;
        return;
    }
    cout << "底盘车设备连接成功！bot_car_fd = " << this->fd << endl;
    if (tcsetattr(this->fd, TCSADRAIN, &newtio) != 0) {
        cerr << "串口初始化失败！" << endl;
        return;
    }
    cout << "底盘车设备初始化成功！" << endl;
}

/**
 * 假如 A 电机的速度为 0x12，即十进制18。这样相当于控制A电机以18的速度运动。下面解释一下这个 18 的单位：
 * 10ms 转 18(0x12) 个脉冲，1s 转 1800 个脉冲。车轮转一圈，输出(编码器线数 500 * 减速比 28 * 倍频 4) = 56000 个脉冲,
 * 实际上因为光电编码器精度非常高，为了方便信号处理与传输，在代码里面已经除以了 8 的，所以相当于车轮转一圈输出 7000
 * 个脉冲。 也就是每秒转 1800/7000 圈，即 0.2571 圈，再结合轮胎的直径信息，就可以得到小车的运行速度。
 * 已知小车车轮的直径为 20cm，即周长为 62.8cm。
 * 小车的速度可以通过每秒转的圈数乘以轮胎的周长来计算，即 0.2571 圈/秒 * 62.8 厘米/圈 ≈ 16.12 厘米/秒。
 * 旋转的角速度可以通过两个轮子的速度差除以两个轮子的轴距来计算。
 * 已知小车轮距为 53cm，即旋转角速度 = abs(left_speed - right_speed) / 53。
 */
void cas::bot::BotCar::setSpeed(const char speed_value, float scale) {
    this->buffer[0] = 0xff;
    this->buffer[1] = 0xfe;
    this->buffer[2] = speed_value;
    this->buffer[3] = speed_value;
    this->buffer[4] = 0x01;
    this->buffer[5] = 0x01;
    this->buffer[6] = 0x00;
    this->buffer[7] = 0x00;
    this->buffer[8] = 0x00;
    this->buffer[9] = 0x00;

    // 计算小车移动线速度
    cout << ((int)speed_value) * 100 / 7000.0 << endl;
    this->speed = ((int)speed_value) * 100 / 7000.0 * WHEEL_D * M_PI * scale;
    cout << "小车移动线速度（cm/s）：" << this->speed << endl;
    // 计算小车旋转角速度
    this->angle_speed = 2 * (int)speed_value * 100 / 7000.0 * WHEEL_D * M_PI / WHEEL_DISTANCE * 180.0 / M_PI;
    cout << "小车旋转角速度（deg/s）：" << this->angle_speed << endl;
}

bool cas::bot::BotCar::moveForward() {
    this->buffer[4] = 0x01;
    this->buffer[5] = 0x01;
    if (write(this->fd, this->buffer, 10) < 0) {
        return false;
    }
    return true;
}
bool cas::bot::BotCar::moveForwardTime(float time_s) {
    cout << "time_s = " << time_s << endl;
    if (!moveForward()) {
        return false;
    }
    cout << (int)(time_s * 1000) << endl;
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
    cout << "stop" << endl;
    return stopCar();
}
bool cas::bot::BotCar::moveForwardDistance(float distance) {
    if (distance < 0) {
        return false;
    } else if (distance == 0) {
        return true;
    }
    return moveForwardTime(distance / this->speed);
}

bool cas::bot::BotCar::moveBackward() {
    unsigned char buffer1[10] = {0};
    buffer1[0] = 0xff;
    buffer1[1] = 0xfe;
    buffer1[2] = 0x12; // A电机速度
    buffer1[3] = 0x12; // B电机速度
    buffer1[4] = 0x01; // A电机方向
    buffer1[5] = 0x01; // B电机方向
    buffer1[6] = 0x00;
    buffer1[7] = 0x00;
    buffer1[8] = 0x00;
    buffer1[9] = 0x00;
    this->buffer[4] = 0x00;
    this->buffer[5] = 0x00;
    if (write(this->fd, buffer1, 10) < 0) {
        return false;
    }
    return true;
}
bool cas::bot::BotCar::moveBackwardTime(float time_s) {
    if (!moveBackward()) {
        return false;
    }
    sleep(time_s * 1000);
    return stopCar();
}
bool cas::bot::BotCar::moveBackwardDistance(float distance) {
    if (distance < 0) {
        return false;
    } else if (distance == 0) {
        return true;
    }
    return moveBackwardTime(distance / this->speed);
}

bool cas::bot::BotCar::turnLeft() {
    buffer[4] = 0x00;
    buffer[5] = 0x01;
    if (write(this->fd, this->buffer, 10) < 0) {
        return false;
    }
    return true;
}
bool cas::bot::BotCar::turnLeftTime(float time_s) {
    if (!turnLeft()) {
        return false;
    }
    sleep(time_s * 1000);
    return stopCar();
}
bool cas::bot::BotCar::turnLeftAngle(float angle) {
    if (angle <= 0) {
        return false;
    }
    return turnLeftTime(angle / this->angle_speed);
}

bool cas::bot::BotCar::turnRight() {
    buffer[4] = 0x01;
    buffer[5] = 0x00;
    if (write(this->fd, this->buffer, 10) < 0) {
        return false;
    }
    return true;
}
bool cas::bot::BotCar::turnRightTime(float time_s) {
    if (!turnRight()) {
        return false;
    }
    sleep(time_s * 1000);
    return stopCar();
}
bool cas::bot::BotCar::turnRightAngle(float angle) {
    if (angle <= 0) {
        return false;
    }
    return turnRightTime(angle / this->angle_speed);
}

bool cas::bot::BotCar::turnAngle(float angle) {
    if (angle < 0) {
        return turnLeftAngle(-angle);
    } else if (angle > 0) {
        return turnRightAngle(angle);
    }
    return true;
}

bool cas::bot::BotCar::stopCar() {
    buffer[4] = 0x00;
    buffer[5] = 0x00;
    if (write(this->fd, this->buffer, 10) < 0) {
        return false;
    }
    return true;
}

bool cas::bot::BotCar::executeMoveSequence(int *seq, int seq_length) {
    int flag = seq[0];
    if (flag == 1) { // 先移动
        // 奇数下标为移动，偶数下标为旋转
        for (int i = 1; i < seq_length; i++) {
            if (i % 2 == 1) { // 如果是奇数
                moveForwardDistance(seq[i]);
            } else {
                turnAngle(seq[i]);
            }
        }
    } else if (flag = 0) { // 先旋转
        // 奇数下标为旋转，偶数下标为移动
        for (int i = 1; i < seq_length; i++) {
            if (i % 2 == 1) { // 如果是奇数
                turnAngle(seq[i]);
            } else {
                moveForwardDistance(seq[i]);
            }
        }
    } else {
        cerr << "未知的底盘车移动序列" << endl;
    }
}
