#include <chrono>
#include <sstream>
#include <thread>

#include "CasBot.h"
#include "CasNetwork.h"
#include "DataMessage.pb.h"

using namespace std;

bool send_proto_message(int fd, google::protobuf::Message& message) {
    ostringstream output_stream(ios::binary);

    if (!message.SerializeToOstream(&output_stream)) {
        cerr << "序列化消息失败" << endl;
        return false;
    }
    cout << "序列化消息成功" << endl;

    // 获取序列化后的数据并发送到网络对端
    string serialized_data = output_stream.str();
    if (write(fd, serialized_data.data(), serialized_data.size()) < 0) {
        cerr << "发送消息失败" << endl;
        return false;
    }
    cout << "发送消息成功" << endl;
    return true;
}

int main() {
    int server_socket_fd = -1;
    int client_fd = -1;
    struct sockaddr_in* addr =
        (struct sockaddr_in*)malloc(sizeof(struct sockaddr_in));
    socklen_t addr_len = (socklen_t)sizeof(*addr);
    memset(addr, 0, sizeof(*addr));

    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(5001);

    char ip_local[32 + 1] = { 0 };
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

    if (bind(server_socket_fd, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) !=
        0) {
        perror("Socket bind failed!\n");
        close(server_socket_fd);
        return -1;
    }
    if (listen(server_socket_fd, 1) != 0) {
        perror("Socket listen failed!\n");
        close(server_socket_fd);
        return -1;
    }

    //   int fd_arm = cas::bot::initArm("/dev/ttyUSB0");
    // 等待客户端连接
    cout << "等待客户端连接..." << endl;

    client_fd = accept(server_socket_fd, (struct sockaddr*)addr, &addr_len);

    if (client_fd < 0) {
        cerr << "Socket accept failed" << endl;
        close(server_socket_fd);
        free(addr);
        return -1;
    }
    else {
        cout << "IP地址: " << inet_ntoa(addr->sin_addr) << ":"
            << ntohs(addr->sin_port) << endl;
    };

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    //   string c = "EXIT";
    //   send(client_fd, c.c_str(), c.length(), 0);

    unsigned char tempbuff[1024];
    unsigned char databuff[18];

    int recv_long = 0;

    while (true) {
        char c;
        cin >> c;
        if (c == 'a') {
            // 发送字符串
            cas::proto::DataMessage data_message;
            // 设置消息类型
            data_message.set_type(cas::proto::DataMessage::MESH);
            cas::proto::Mesh* mesh_message = data_message.mutable_mesh();
            for (int i = 0; i < 2; i++) {
                cas::proto::V1* v1 = mesh_message->add_v1();
                v1->set_x(i);
                v1->set_y(i);
                v1->set_z(i);

                cas::proto::V2* v2 = mesh_message->add_v2();
                v2->set_x(i);
                v2->set_y(i);
                v2->set_z(i);

                cas::proto::V3* v3 = mesh_message->add_v3();
                v3->set_x(i);
                v3->set_y(i);
                v3->set_z(i);

                mesh_message->add_r(i);
                mesh_message->add_g(i);
                mesh_message->add_b(i);
            }
            send_proto_message(client_fd, data_message);

        }
        else if (c == 'q') {
            cas::proto::DataMessage data_message;
            data_message.set_type(cas::proto::DataMessage_Type_EXIT_MESH);
            send_proto_message(client_fd, data_message);
        }
        else {
            cout << "NO!" << endl;
        }

        // if (c == 'a') {
        //   // 发送字符串
        //   cas::proto::Mesh mesh_message;
        //   // 设置消息类型
        //   for (int i = 0; i < 10; i++) {
        //     cas::proto::V1 *v1 = mesh_message.add_v1();
        //     v1->set_x(i);
        //     v1->set_y(i);
        //     v1->set_z(i);

        //     cas::proto::V2 *v2 = mesh_message.add_v2();
        //     v2->set_x(i);
        //     v2->set_y(i);
        //     v2->set_z(i);

        //     cas::proto::V3 *v3 = mesh_message.add_v3();
        //     v3->set_x(i);
        //     v3->set_y(i);
        //     v3->set_z(i);

        //     mesh_message.add_r(i);
        //     mesh_message.add_g(i);
        //     mesh_message.add_b(i);
        //   }
        //   ostringstream output_stream(ios::binary);

        //   // 将 pg 对象序列化到内存输出流中
        //   if (!mesh_message.SerializeToOstream(&output_stream)) {
        //     cerr << "序列化消息失败" << endl;
        //   }

        //   // 获取序列化后的数据并发送到网络对端
        //   string serialized_data = output_stream.str();

        // //   string serialized_data = mesh_message.SerializeAsString();
        //   cout << "序列化成功" << endl;

        //   if (write(client_fd, serialized_data.data(), serialized_data.size()) <
        //   0) {
        //     cerr << "发送场景模型消息失败" << endl;
        //   }
        //   cout << "发送成功" << endl;
        // } else {
        //   cout << "NO!" << endl;
        // }
        // 接收服务器消息：
        //  cout << "接收服务器消息：" << endl;
        //  memset(tempbuff, 0, sizeof(tempbuff));
        //  memset(databuff, 0, sizeof(databuff));
        //  recv_long = recv(client_fd, tempbuff, sizeof(tempbuff), 0);
        //  if (recv_long > 0) {
        //      memcpy(databuff, tempbuff, recv_long);
        //      cout << "接收到的数据长度：" << recv_long << endl;
        //      cout << "接收到的数据：" << databuff << endl;
        //      write(fd_arm, databuff, sizeof(databuff));
        //      //
        //  }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}