#ifndef CASNETWORK_H
#define CASNETWORK_H

#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
#include <net/if.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace cas {
    namespace net {
        bool getLocalIp(char *ip);

        int creatServerSocket(int port);
    };// namespace net
}// namespace cas

#endif//CASNETWORK_H
