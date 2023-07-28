#ifndef CASNETWORK_H
#define CASNETWORK_H

#include "DataMessage.pb.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
#include <net/if.h>
#include <netinet/in.h>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <netinet/tcp.h> //TCP_NODELAY
#include <CasUtility.h>

using namespace cas::utility;

namespace cas {
    namespace net {
        // bool getLocalIp(char *ip);

        int creatServerSocket(int port);

        namespace proto {
            bool send_message(int fd, google::protobuf::Message &message);
            bool send_exit_mesh_message(int fd);
        } // namespace proto

        class Client {
        private:
            int fd;

        public:
            Client(const int port);
            // 反馈函数
            Client(const int port, std::function<void()> onConnect = nullptr);
            bool sendMessage(google::protobuf::Message &message);
            int recvData(unsigned char *recv_buffer, const int recv_length);
            bool recvMessage(cas::proto::DataMessage &message);
            bool sendExitMeshMessage();
        };

    }; // namespace net
} // namespace cas

#endif // CASNETWORK_H
