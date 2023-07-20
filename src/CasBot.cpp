#include "CasBot.h"

#include <cmath>
#include <iostream>
#include <thread>

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
    this->command_buffer[0] = 0xFE;
    this->command_buffer[1] = 0xFE;
    this->gripper_buffer[0] = 0xFE;
    this->gripper_buffer[1] = 0xFE;
    this->gripper_buffer[2] = 0x04;
    this->gripper_buffer[3] = CommandSet::SEND_GRIPPER_ANGLE;
    this->gripper_buffer[5] = 0x32;
    this->gripper_buffer[6] = 0xFA;
}

bool cas::bot::BotArm::reset() {
    unsigned char databuff[16] = {0};
    memset(databuff, 0, 16);                      // 清空数组
    databuff[12] = 0x1e;                          // 速度
    serialTX(this->fd, POST_ALL_ANGLE, databuff); // 发送数据
    return true;
}

bool cas::bot::BotArm::execute(const char *data_buffer, int length) {
    if (write(this->fd, data_buffer, length) < 0) {
        return false;
    }
    return true;
}

int cas::bot::BotArm::recvData(unsigned char *recv_buffer, const int recv_length) {
    int len = -1;
    while ((len = read(this->fd, recv_buffer, recv_length)) < 0)
        ;
    switch (recv_buffer[3]) { // 数据帧
        case cas::bot::BotArm::DataSet::ALL_ANGLE: {
            // 计算方法：角度值低位 + 角度高位值乘以256 先判断是否大于33000，
            //         如果大于33000就再减去65536，最后除以100，如果小于33000就直接除以100
            int angles[6];
            for (int i = 0; i < 6; i++) {
                if (recv_buffer[5 + 2 * i] + recv_buffer[4 + 2 * i] * 256 > 33000) {
                    angles[i] = (recv_buffer[5 + 2 * i] + recv_buffer[4 + 2 * i] * 256 - 65536) / 100;
                } else {
                    angles[i] = (recv_buffer[5 + 2 * i] + recv_buffer[4 + 2 * i] * 256) / 100;
                }
            }
            break;
        }
        case cas::bot::BotArm::DataSet::ALL_COORD: {

            break;
        }
        default:
            cerr << "未知的机械臂数据类型！" << endl;
            break;
    }
    return len;
}

int getCommand(cas::bot::BotArm::CommandSet command_type, char *&command) {
    int cmd_len = 1;
    switch (command_type) {
        case cas::bot::BotArm::CommandSet::READ_ANGLE: {
            cmd_len = 2;
            command = new char[cmd_len];
            command[0] = cmd_len;
            command[1] = cas::bot::BotArm::CommandSet::READ_ANGLE;
            break;
        }
        case cas::bot::BotArm::CommandSet::READ_COORD: {
            cmd_len = 2;
            command = new char[cmd_len];
            command[0] = cmd_len;
            command[1] = cas::bot::BotArm::CommandSet::READ_COORD;
            break;
        }
        case cas::bot::BotArm::CommandSet::FREE_MODE: {
            cmd_len = 2;
            command = new char[cmd_len];
            command[0] = cmd_len;
            command[1] = cas::bot::BotArm::CommandSet::FREE_MODE;
            break;
        }
        default:
            cerr << "未定义的命令类型！" << endl;
            command = new char[1];
            command[0] = 0x01;
    }
    return cmd_len;
}

bool cas::bot::BotArm::sendCommand(cas::bot::BotArm::CommandSet command_type) {
    char *command;
    int command_length = getCommand(command_type, command);
    memcpy(this->command_buffer + 2, command, command_length);
    this->command_buffer[command_length + 2] = 0xFA;

    // 计算方法：角度值低位 + 角度高位值乘以256 先判断是否大于33000 如果大于33000就再减去65536 最后除以100
    // 如果小于33000就直接除以100
    //  0x01 0x20 0xFA
    return execute(this->command_buffer, command_length + 3);
}

bool cas::bot::BotArm::openGripper(const char value) {
    this->gripper_buffer[4] = value;
    return execute(this->gripper_buffer, 7);
}

bool cas::bot::BotArm::closeGripper() {
    this->gripper_buffer[4] = 0x00;
    return execute(this->gripper_buffer, 7);
}

cas::bot::STM32::STM32(string serial_port_name) {
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
}

bool cas::bot::STM32::sendData(unsigned char *send_buffer, const int send_length) {
    if (write(this->fd, send_buffer, send_length) < 0) {
        return false;
    }
    return true;
}

int cas::bot::STM32::recvData(unsigned char *recv_buffer, const int recv_length) {
    int len = -1;
    while ((len = read(this->fd, recv_buffer, recv_length)) < 0)
        ;
    return len;
}

cas::bot::BotMotor::BotMotor(string serial_port_name = DEFAULT_SERIAL_PORT_NAME) : STM32(serial_port_name) {
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
    this->buffer[1] = direction[0];
    if (direction == "F" ) {
        this->buffer[2] = '2';
        this->buffer[3] = '0';
        this->buffer[4] = '0';
        this->buffer[5] = '0';
        this->buffer[6] = '0';
        return STM32::sendData(this->buffer, 9);
    } else if(direction == "R") {
        this->buffer[2] = '2';
        this->buffer[3] = '5';
        this->buffer[4] = '0';
        this->buffer[5] = '0';
        this->buffer[6] = '0';
        return STM32::sendData(this->buffer, 9);
    }
    return false;
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

cas::bot::BotCar::BotCar(string serial_port_name, const char speed_value, float scale) : STM32(serial_port_name) {
    setSpeed(speed_value, scale);
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

    this->buffer2[0] = 0xff;
    this->buffer2[1] = 0xfe;
    this->buffer2[2] = speed_value; // A速度
    this->buffer2[3] = speed_value; // B速度
    this->buffer2[4] = 0x00;        // 方向，1左转，2右转
    this->buffer2[5] = 0x00;        // 角度
    this->buffer2[6] = 0x01;        // 模式：1角度旋转模式
    this->buffer2[7] = 0x00;
    this->buffer2[8] = 0x00;
    this->buffer2[9] = 0x00;

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
    return STM32::sendData(this->buffer, 10);
}
bool cas::bot::BotCar::moveForwardTime(float time_s) {
    // cout << "time_s = " << time_s << endl;
    if (!moveForward()) {
        return false;
    }
    cout << (int)(time_s * 1000) << endl;
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
    // cout << "stop" << endl;
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
    this->buffer[4] = 0x00;
    this->buffer[5] = 0x00;
    return STM32::sendData(this->buffer, 10);
}
bool cas::bot::BotCar::moveBackwardTime(float time_s) {
    if (!moveBackward()) {
        return false;
    }
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
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
    this->buffer[4] = 0x00;
    this->buffer[5] = 0x01;
    return STM32::sendData(this->buffer, 10);
}
bool cas::bot::BotCar::turnLeftTime(float time_s) {
    cout << "time_s = " << time_s << endl;
    if (!turnLeft()) {
        return false;
    }
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
    return stopCar();
}
bool cas::bot::BotCar::turnLeftAngle(float angle) {
    if (angle <= 0) {
        return false;
    }
    return turnLeftTime(angle / this->angle_speed);
}

bool cas::bot::BotCar::turnRight() {
    this->buffer[4] = 0x01;
    this->buffer[5] = 0x00;
    return STM32::sendData(this->buffer, 10);
}

bool cas::bot::BotCar::turnRightTime(float time_s) {
    if (!turnRight()) {
        return false;
    }
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
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

bool cas::bot::BotCar::autoTurnByAngle(float angle) {
    if (angle > 0) {
        this->buffer2[4] = 0x01; // 大于0左转，小于0右转
        this->buffer2[5] = (int)angle - 1;
    } else if (angle < 0) {
        this->buffer2[4] = 0x02; // 大于0左转，小于0右转
        this->buffer2[5] = -(int)angle - 1;
    } else {
        return false;
    }
    return STM32::sendData(this->buffer2, 10);
}

bool cas::bot::BotCar::autoTurnByAngleAndSpeed(float angle, char left_speed_value, char right_speed_value) {
    int left_temp = this->buffer2[2];
    int right_temp = this->buffer2[3];
    this->buffer2[2] = left_speed_value;
    this->buffer2[3] = right_speed_value;
    if (!autoTurnByAngle(angle)) {
        return false;
    }
    this->buffer2[2] = left_temp;
    this->buffer2[3] = right_temp;
    return true;
}

bool cas::bot::BotCar::stopCar() {
    int left_temp = this->buffer[2];
    int right_temp = this->buffer[3];
    this->buffer[2] = 0x00;
    this->buffer[3] = 0x00;
    if (!STM32::sendData(this->buffer, 10)) {
        return false;
    }
    this->buffer[2] = left_temp;
    this->buffer[3] = right_temp;
    return true;
}

bool cas::bot::BotCar::executeMoveSequence(float *seq, int seq_length) {
    float flag = seq[0];
    if (flag == 1) { // 先移动
        // 奇数下标为移动，偶数下标为旋转
        for (int i = 1; i < seq_length; i++) {
            cout << "执行：" << seq[i] << endl;
            if (i % 2 == 1) { // 如果是奇数
                moveForwardDistance(seq[i]);
                this_thread::sleep_for(chrono::milliseconds(500));
            } else {
                // turnAngle(-seq[i]);
                autoTurnByAngle(seq[i]);
                this_thread::sleep_for(chrono::milliseconds(8000));
            }
        }
    } else if (flag == 0) { // 先旋转
        // 奇数下标为旋转，偶数下标为移动
        for (int i = 1; i < seq_length; i++) {
            cout << "执行：" << seq[i] << endl;
            if (i % 2 == 1) { // 如果是奇数
                // turnAngle(-seq[i]);
                autoTurnByAngle(seq[i]);
                this_thread::sleep_for(chrono::milliseconds(8000));
            } else {
                moveForwardDistance(seq[i]);
                this_thread::sleep_for(chrono::milliseconds(500));
            }
        }
    } else {
        cerr << "未知的底盘车移动序列" << endl;
        return false;
    }
    return true;
}
