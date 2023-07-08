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
        string config_path;
        map<string, string> config_map;

    public:
        CasConfig(const string config_path, const char comment_char = COMMENT_CHAR);

        string get(const string key);

        float get_float(const string key);

        int get_int(const string key);

        bool get_bool(const string key);

        bool set(const string key, const string value);
    };
}// namespace cas


#endif
