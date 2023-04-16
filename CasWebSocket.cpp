//
// Create by HoChihchou on 2023/3/27
//
//服务器地址：ws://175.178.56.40:8080/ws-test
//此文件用于将AcquiringPointCloud产生的点云数据发送至服务器

#include "CasWebSocket.h"
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <iostream>
#include <string>

using tcp = boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;

using namespace std;
using namespace cas;

struct WebSocket::Impl {
    net::io_context ioc;
    string url;
    string port;
    string target;
    tcp::resolver resolver;
    websocket::stream<tcp::socket> ws;

    Impl(const string &url, const string &port, const string &target) : url(url) ,port(port), target(target), resolver(ioc), ws(ioc) {};

    bool connect() {
        try {
            auto const results = resolver.resolve(url,port); //resolve需要两个参数，一个是url，一个是端口号
            net::connect(ws.next_layer(), results.begin(), results.end());
            ws.handshake(url, target);
            return true;
        } catch (const std::exception &e) {
            std::cerr << "Error connecting to WebSocket server: " << e.what() << std::endl;
            return false;
        }
    }

    void disconnect() {
        ws.close(websocket::close_code::normal);
    }

    bool isConnected() const {
        return ws.is_open();
    }

    void sendPointCloud(string pointCloudData) {
        if (pointCloudData.empty()) {
            std::cout << "Error: Empty point cloud data" << std::endl;
            return;
        }
        if (ws.is_open()) {
            cout << "Server open, sending point cloud data..." << endl;
//            ws.write(net::buffer(pointCloudData, std::strlen(pointCloudData)));
            ws.write(net::buffer(pointCloudData.c_str(), pointCloudData.length()));
            //判断是否发送成功
            if (ws.got_text()) {
                cout << "Send successfully." << endl;
            } else {
                cout << "Send failed." << endl;
            }
        } else {
            cout << "Server closed." << endl;
        }
    }
};

WebSocket::WebSocket(const string &url, const string &port, const string &target) : m_impl(new Impl(url, port, target)) {}

WebSocket::~WebSocket() {}

bool WebSocket::connect() {
    return m_impl->connect();
}

void WebSocket::disconnect() {
    m_impl->disconnect();
}

bool WebSocket::isConnected() const {
    return m_impl->isConnected();
}

void WebSocket::sendPointCloud(string pointCloudData) {
    m_impl->sendPointCloud(pointCloudData);
}


//int main()
//{
//    namespace beast = boost::beast;
//    namespace http = beast::http;
//    namespace websocket = beast::websocket;
//    namespace net = boost::asio;
//    using tcp = net::ip::tcp;
//
//    try
//    {
//        net::io_context ioc;
//
//        // 创建 TCP 侦听器并连接到 WebSocket 服务器
//        tcp::resolver resolver(ioc);
//        auto const results = resolver.resolve("echo.websocket.org", "80");
//        websocket::stream<tcp::socket> ws(ioc);
//        net::connect(ws.next_layer(), results.begin(), results.end());
//        ws.handshake("echo.websocket.org", "/");
//
//        // 发送 WebSocket 消息
//        ws.write(net::buffer(std::string("Hello, world!")));
//
//        // 接收并显示 WebSocket 响应
//        beast::flat_buffer buffer;
//        websocket::opcode op;
//        ws.read(op, buffer);
//        std::cout << beast::make_printable(buffer.data()) << std::endl;
//
//        // 关闭 WebSocket 连接
//        ws.close(websocket::close_code::normal);
//    }
//    catch (const std::exception& e)
//    {
//        std::cerr << "Error: " << e.what() << std::endl;
//        return EXIT_FAILURE;
//    }
//
//    return EXIT_SUCCESS;
//}

