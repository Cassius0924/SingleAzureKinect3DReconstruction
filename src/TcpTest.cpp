#include "CasNetwork.h"
#include "CasBot.h"
#include <chrono>
#include <thread>

using namespace std;

int main() {
    int server_socket_fd = -1;
    int client_fd = -1;
    struct sockaddr_in *addr = (struct sockaddr_in *) malloc(sizeof(struct sockaddr_in));
    socklen_t addr_len = (socklen_t) sizeof(*addr);
    memset(addr, 0, sizeof(*addr));

    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(5001);


    char ip_local[32 + 1] = {0};
    if (!cas::net::getLocalIp(ip_local)) {
        cout << "连接IP失败: " << ip_local << endl;
        return -1;
    }
    inet_aton(ip_local, &sockaddr.sin_addr);

    server_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_fd < 0) {
        perror("Socket create failed!\n");
        return -1;
    }

    if (bind(server_socket_fd, (struct sockaddr *) &sockaddr, sizeof(sockaddr)) != 0) {
        perror("Socket bind failed!\n");
        close(server_socket_fd);
        return -1;
    }
    if (listen(server_socket_fd, 1) != 0) {
        perror("Socket listen failed!\n");
        close(server_socket_fd);
        return -1;
    }

    int fd_arm = cas::bot::initArm("/dev/ttyUSB0");
    //等待客户端连接
    cout << "等待客户端连接..." << endl;

    client_fd = accept(server_socket_fd, (struct sockaddr *) addr, &addr_len);

    if (client_fd < 0) {
        cerr << "Socket accept failed" << endl;
        close(server_socket_fd);
        free(addr);
        return -1;
    } else {
        cout << "IP地址: " << inet_ntoa(addr->sin_addr) << ":" << ntohs(addr->sin_port) << endl;
    };

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    string c = "EXIT";
    send(client_fd, c.c_str(), c.length(), 0);


    unsigned char tempbuff[1024]; /*临时缓存*/
    unsigned char databuff[18];

    int recv_long = 0;


    while (true) {
        // string c ;
        // cin >> c;
        // //发送字符串
        // send(client_fd, c.c_str(), c.length(), 0);


        //接收服务器消息：
        cout << "接收服务器消息：" << endl;
        memset(tempbuff, 0, sizeof(tempbuff));
        memset(databuff, 0, sizeof(databuff));
        recv_long = recv(client_fd, tempbuff, sizeof(tempbuff), 0);
        if (recv_long > 0) {
            memcpy(databuff, tempbuff, recv_long);
            cout << "接收到的数据长度：" << recv_long << endl;
            cout << "接收到的数据：" << databuff << endl;
            write(fd_arm, databuff, sizeof(databuff));
            //
        }
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }

    return 0;
}