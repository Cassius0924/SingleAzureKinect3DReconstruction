//
// Create by HoChihchou on 2023/3/27
//

#ifndef SMALL_AZURE_KINECT_DK_3D_RECONSTRUCTION_CASWEBSOCKET_H
#define SMALL_AZURE_KINECT_DK_3D_RECONSTRUCTION_CASWEBSOCKET_H

//服务器地址：ws://175.178.56.40:8080/ws-test
//此文件用于将AcquiringPointCloud产生的点云数据发送至服务器
//应该在AcquiringPointCloud中调用此文件中的函数
//函数：sendPointCloudToServer，参数：点云数据

#include "string"
#include <CasUtility.h>

using namespace std;
using namespace cas::utility;

namespace cas {
    class CasWebSocket {
    public:
        CasWebSocket(const string &url, const string &port, const string &target);

        ~CasWebSocket();

        bool connect();

        void disconnect();

        bool isConnected() const;

        void sendPointCloud(string pointCloudData);

    private:
        struct Impl;
        Impl *m_impl;
    };
}

#endif //SMALL_AZURE_KINECT_DK_3D_RECONSTRUCTION_CASWEBSOCKET_H
