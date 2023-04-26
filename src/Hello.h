#include <arpa/inet.h>
#include <fcntl.h> // for open
#include <net/if.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>// for close


class Hello {
public:
    int get_local_ip(char *ip);
};
