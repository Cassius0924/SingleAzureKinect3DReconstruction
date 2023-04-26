//
// Create by HoChihchou on 2023/3/27
//
// 读取配置文件

#include "CasConfig.h"

#include <fstream>

using namespace std;
using namespace cas;

void tirm(string &str) {
    str.erase(0, str.find_first_not_of(" "));
    str.erase(str.find_last_not_of(" ") + 1);
}

CasConfig::CasConfig(const string config_path, const char comment_char) {
    ifstream in(config_path);   
    if (!in) {
        cout << "配置文件不存在" << endl;
        return;
    }
    string line;
    while (getline(in, line)) {
        if (line.empty() || line[0] == comment_char) {  //空行或注释行
            continue;
        }
        int pos = line.find("=");   //找到等号
        if (pos == -1) {    
            continue;   //没有等号，跳过
        }
        string key = line.substr(0, pos); 
        string value = line.substr(pos + 1);
        tirm(key);
        tirm(value);
        this->config_map[key] = value;
    }
}

string CasConfig::get(const string key) {
    map<string, string>::iterator iter;
    iter = this->config_map.find(key);
    if (iter != this->config_map.end()) {   //找到了
        return iter->second;
    }
    return "";
}