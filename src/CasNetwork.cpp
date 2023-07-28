#include "CasNetwork.h"

using namespace std;

/*
 * 获取本地192.168开头的本地IP并打印
 * 参数：IP地址的存储地址（char*类型）
 * 返回值：获取成功返回0
 */
bool getLocalIp(char *ip) {
    int fd, intrface, retn = 0;        // fd是用户程序打开设备时使用open函数返回的文件标示符
    struct ifreq buf[INET_ADDRSTRLEN]; // INET_ADDRSTRLEN 宏定义，16
    struct ifconf ifc;                 // ifconf > ifreq
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) >=
        0) // 创建套接字，存放AF_INET代表IPV4，SOCK_DGRAM代表创建的是数据报套接字/无连接的套接字，后面一般为0
    {
        // 套接字创建成功
        ifc.ifc_len = sizeof(buf); // 所有网口加一起的长度 ifc.ifc_len 应该是一个出入参数
        // caddr_t,linux内核源码里定义的：typedef void *caddr_t；一般是一个int
        ifc.ifc_buf = (caddr_t)buf;                // 开辟网口缓冲区
        if (!ioctl(fd, SIOCGIFCONF, (char *)&ifc)) // linux系统调用函数，SIOCGIFCONF   获取所有接口(网口)的清单
        {
            intrface = ifc.ifc_len / sizeof(struct ifreq);             // 获取到int值的网口总数
            while (intrface-- > 0) {
                if (!(ioctl(fd, SIOCGIFADDR, (char *)&buf[intrface]))) // SIOCGIFADDR 获取接口(网口)地址
                {
                    // inet_ntoa ()功能是将网络地址转换成“.”点隔的字符串格式。序列化
                    // 拿到该网口地址做对比
                    ip = (inet_ntoa(((struct sockaddr_in *)(&buf[intrface].ifr_addr))->sin_addr));
                    if (strstr(ip, "192.168.")) {
                        Debug::CoutInfo("服务器本地IP: {}", ip);
                        break;
                    }
                }
            }
        }
        close(fd);
        return true;
    }
    Debug::CoutError("获取本地IP地址失败");
    return false;
}

/**
 * 创建服务器，阻塞进程，等待服务器连接。
 */
cas::net::Client::Client(const int port, std::function<void()> onConnect) {

    // int cas::net::creatServerSocket(int port) {
    int server_socket_fd = -1;
    this->fd = -1;
    struct sockaddr_in *addr = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
    socklen_t addr_len = (socklen_t)sizeof(*addr);
    memset(addr, 0, sizeof(*addr));

    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port); // 端口号

    char ip_local[32 + 1] = {0};
    if (!getLocalIp(ip_local)) {
        Debug::CoutError("连接IP失败: {}", ip_local);
        exit(0);
    }
    inet_aton(ip_local, &sockaddr.sin_addr); // 将一个字符串IP地址转换为一个32位的网络序列IP地址

    server_socket_fd = socket(AF_INET, SOCK_STREAM, 0); // 创建套接字

    // setsockopt(server_socket_fd, IPPROTO_TCP, O_NDELAY, (char *)&flag, sizeof(int));
    // int on = 1;
    // int result = setsockopt(server_socket_fd, IPPROTO_TCP, TCP_NODELAY, (char *)&on, sizeof(int)); // 1 - on, 0 - off
    // if (result == -1) {
    //     Debug::CoutError("关闭 Nagle error");
    // }

    if (server_socket_fd < 0) {
        Debug::CoutError("Socket 创建失败");
        exit(0);
    }

    if (bind(server_socket_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) { // 绑定套接字
        Debug::CoutError("Socket 绑定失败");
        close(server_socket_fd);
        exit(0);
    }
    if (listen(server_socket_fd, 1) != 0) { // 监听套接字
        Debug::CoutError("Socket 监听失败");
        close(server_socket_fd);
        exit(0);
    }

    Debug::CoutDebug("等待客户端连接...");
    this->fd = accept(server_socket_fd, (struct sockaddr *)addr, &addr_len);

    if (this->fd < 0) {
        Debug::CoutError("Socket 接收失败");
        close(server_socket_fd);
        free(addr);
        exit(0);
    } else {
        Debug::CoutSuccess("客户端连接成功，客户端IP: {}:{}", inet_ntoa(addr->sin_addr), ntohs(addr->sin_port));
        if (onConnect != nullptr) {
            onConnect();
        }
    };
}

// 发送Protobuf消息到网络对端
bool cas::net::Client::sendMessage(google::protobuf::Message &message) {
    ostringstream output_stream(ios::binary);

    int message_size = message.ByteSizeLong();
    output_stream.write(reinterpret_cast<const char *>(&message_size), sizeof(message_size));

    if (!message.SerializeToOstream(&output_stream)) {
        Debug::CoutError("序列化消息失败");
        return false;
    }

    // 获取序列化后的数据并发送到网络对端
    string serialized_data = output_stream.str();
    if (write(this->fd, serialized_data.data(), serialized_data.size()) < 0) {
        Debug::CoutError("发送消息失败");
        return false;
    }
    return true;
}

// 发送exit_mesh
bool cas::net::Client::sendExitMeshMessage() {
    cas::proto::DataMessage message;
    message.set_type(cas::proto::DataMessage::EXIT_MESH);
    return sendMessage(message);
}

int cas::net::Client::recvData(unsigned char *recv_buffer, const int recv_length) {
    int len = -1;
    while ((len = recv(this->fd, recv_buffer, recv_length, 0)) < 0)
        ;
    return len;
}

bool cas::net::Client::recvMessage(cas::proto::DataMessage &message) {
    unsigned char recv_buffer[1024];
    int len = recvData(recv_buffer, 1024);
    if (!message.ParseFromArray(recv_buffer, len)) {
        return false;
    }
    return true;
}
