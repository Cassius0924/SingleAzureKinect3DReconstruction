//
// Create by HoChihchou on 2023/3/27
//
// 读取配置文件

#include "CasConfig.h"

#include <fstream>
#include <sstream>

using namespace std;
using namespace cas;

void tirm(string& str) {
    str.erase(0, str.find_first_not_of(" "));
    str.erase(str.find_last_not_of(" ") + 1);
}

CasConfig::CasConfig(const string config_path, const char comment_char) {
    this->config_path = config_path;
    ifstream in(this->config_path);
    if (in) {
        cout << "打开配置文件成功" << endl;
    }
    else {
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
    cout << "找不到配置项: " << key << "，返回默认值 \"\"" << endl;
    return "";
}

float CasConfig::get_float(const string key) {
    map<string, string>::iterator iter;
    iter = this->config_map.find(key);
    if (iter != this->config_map.end()) {
        return stof(iter->second);
    }
    cout << "找不到配置项: " << key << "，返回默认值 0.0" << endl;
    return 0.0;
}

int CasConfig::get_int(const string key) {
    map<string, string>::iterator iter;
    iter = this->config_map.find(key);
    if (iter != this->config_map.end()) {
        return stoi(iter->second);
    }
    cout << "找不到配置项: " << key << "，返回默认值 0" << endl;
    return 0;
}

bool CasConfig::get_bool(const string key) {
    map<string, string>::iterator iter;
    iter = this->config_map.find(key);
    if (iter != this->config_map.end()) {
        return iter->second == "true";
    }
    cout << "找不到配置项: " << key << "，返回默认值 false" << endl;
    return false;
}

// 找到配置项，并修改，写入文件
bool CasConfig::set(const string key, const string value) {
    std::ifstream inputFile(this->config_path); // 打开配置文件进行读取
    std::string line;
    std::ostringstream modifiedContent;

    while (std::getline(inputFile, line)) {
        if (line.find(key) != std::string::npos) { // 如果找到配置项
            modifiedContent << key << "=" << value << std::endl; // 修改值
        }
        else {
            modifiedContent << line << std::endl; // 保持原样
        }
    }

    inputFile.close();
    std::ofstream outputFile(this->config_path); // 打开配置文件进行写入
    outputFile << modifiedContent.str(); // 将修改后的内容写入文件
    outputFile.close();
    return false;
}
