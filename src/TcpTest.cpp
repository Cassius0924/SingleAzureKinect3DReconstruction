#include "CasIp.h"
#include "Mesh.pb.h"

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

    cas::CasIp cas_ip;
    char ip_local[32 + 1] = {0};
    if (!cas_ip.get_local_ip(ip_local)) {
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

    while (true) {
        char c = getchar();
        cout << "send" << endl;
        write(client_fd, &c, 1);
    }

    return 0;
}