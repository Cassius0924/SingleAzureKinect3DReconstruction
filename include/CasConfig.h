//
// Created by HoChihchou on 4/23/23.
//
// 读取配置文件

#ifndef CASCONFIG_H
#define CASCONFIG_H

#include <iostream>
#include <string>
#include <map>

using namespace std;

#define COMMENT_CHAR '#'

namespace cas {
    // 读取配置
    class CasConfig {
    private:
        map<string, string> config_map;

    public:
        CasConfig(const string config_path, const char comment_char = COMMENT_CHAR);

        string get(const string key);
    };
}// namespace cas


#endif
