#ifndef CASIP_H
#define CASIP_H

#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
#include <net/if.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace cas {
    class CasIp {
    public:
        bool get_local_ip(char *ip);
    };
}// namespace cas

#endif//CASIP_H
