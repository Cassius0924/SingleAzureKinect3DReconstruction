#include "CasNetwork.h"
// #include "polygon.pb.h"

using namespace std;

/*
 * 获取本地192.168开头的本地IP并打印
 * 参数：IP地址的存储地址（char*类型）
 * 返回值：获取成功返回0
*/
bool cas::net::getLocalIp(char *ip) {
    int fd, intrface, retn = 0;                    //fd是用户程序打开设备时使用open函数返回的文件标示符
    struct ifreq buf[INET_ADDRSTRLEN];             //INET_ADDRSTRLEN 宏定义，16
    struct ifconf ifc;                             //ifconf > ifreq
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) >= 0)//创建套接字，存放AF_INET代表IPV4，SOCK_DGRAM代表创建的是数据报套接字/无连接的套接字，后面一般为0
    {
        //套接字创建成功
        ifc.ifc_len = sizeof(buf);                 //所有网口加一起的长度 ifc.ifc_len 应该是一个出入参数
                                                   //caddr_t,linux内核源码里定义的：typedef void *caddr_t；一般是一个int
        ifc.ifc_buf = (caddr_t) buf;               //开辟网口缓冲区
        if (!ioctl(fd, SIOCGIFCONF, (char *) &ifc))//linux系统调用函数，SIOCGIFCONF   获取所有接口(网口)的清单
        {
            intrface = ifc.ifc_len / sizeof(struct ifreq);//获取到int值的网口总数
            while (intrface-- > 0) {
                if (!(ioctl(fd, SIOCGIFADDR, (char *) &buf[intrface])))//SIOCGIFADDR 获取接口(网口)地址
                {
                    //inet_ntoa ()功能是将网络地址转换成“.”点隔的字符串格式。序列化
                    //拿到该网口地址做对比
                    ip = (inet_ntoa(((struct sockaddr_in *) (&buf[intrface].ifr_addr))->sin_addr));
                    if (strstr(ip, "192.168.")) {
                        cout << "服务器本地IP:" << ip << endl;
                        break;
                    }
                }
            }
        }
        close(fd);
        return true;
    }
    cout << "获取本地IP地址失败" << endl;
    return false;
}

int cas::net::creatServerSocket(int port) {
    int server_socket_fd = -1;
    int client_fd = -1;
    struct sockaddr_in *addr = (struct sockaddr_in *) malloc(sizeof(struct sockaddr_in));
    socklen_t addr_len = (socklen_t) sizeof(*addr);
    memset(addr, 0, sizeof(*addr));

    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);    //端口号

    char ip_local[32 + 1] = {0};
    if (!cas::net::getLocalIp(ip_local)) {
        cout << "连接IP失败: " << ip_local << endl;
        return -1;
    }
    inet_aton(ip_local, &sockaddr.sin_addr);//将一个字符串IP地址转换为一个32位的网络序列IP地址

    server_socket_fd = socket(AF_INET, SOCK_STREAM, 0);//创建套接字
    if (server_socket_fd < 0) {
        cerr << "Socket 创建失败" << endl;
        return -1;
    }

    if (bind(server_socket_fd, (struct sockaddr *) &sockaddr, sizeof(sockaddr)) != 0) {//绑定套接字
        cerr << "Socket 绑定失败" << endl;
        close(server_socket_fd);
        return -1;
    }
    if (listen(server_socket_fd, 1) != 0) {//监听套接字
        cerr << "Socket 监听失败" << endl;
        close(server_socket_fd);
        return -1;
    }

    cout << "等待客户端连接..." << endl;

    client_fd = accept(server_socket_fd, (struct sockaddr *) addr, &addr_len);
    cout << "客户端连接成功" << endl;

    if (client_fd < 0) {
        cerr << "Socket 接收失败" << endl;
        close(server_socket_fd);
        free(addr);
        return -1;
    } else {
        cout << "客户端IP: " << inet_ntoa(addr->sin_addr) << ":" << ntohs(addr->sin_port) << endl;
    };

    return client_fd;
}


/*


bool Hello::ProtobufEncode(std::string& strPb)
{
	//DATA::Struct::Sensor sensor;

	my_package::pg sensor;

	sensor.set_x(2.2);
	sensor.set_y(3.3);
	sensor.set_z(4.4);

	sensor.set_r(5.5);
	sensor.set_g(6.6);
	sensor.set_b(7.7);

	uint32_t pbSize = sensor.ByteSize();        // 获取序列化后的大小

	strPb.clear();
	strPb.resize(pbSize);
	uint8_t* szData = (uint8_t*)strPb.c_str();

	if (!sensor.SerializeToArray(szData, pbSize))   // 拷贝序列化后的数据
	{
		std::cout << "sensor pb msg SerializeToArray failed." << std::endl;
		return false;
	}
	return true;
}

void Hello::printSensor(my_package::pg& sensor)
{
	std::cout << "x:\t" << sensor.x() << std::endl;
	std::cout << "y:\t" << sensor.y() << std::endl;
	std::cout << "z:\t" << sensor.z() << std::endl;

	std::cout << "r:\t" << sensor.r() << std::endl;
	std::cout << "g:\t" << sensor.g() << std::endl;
	std::cout << "b:\t" << sensor.b() << std::endl;

}

bool Hello::ProtobufDecode(std::string& strPb)
{
	my_package::pg sensor;
	sensor.ParseFromArray(strPb.c_str(), strPb.size()); // 反序列化

	printSensor(sensor);

	return true;
}


*/