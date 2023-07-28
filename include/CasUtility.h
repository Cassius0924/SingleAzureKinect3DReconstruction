//
// Created by HoChihchou on 4/23/23.
//

#ifndef CASUTILITY_H
#define CASUTILITY_H

#include <cstdarg>
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>

using namespace std;

#define COMMENT_CHAR '#'

#define COUT_RESET "\033[0m"
#define COUT_RED "\033[31m"
#define COUT_GREEN "\033[32m"
#define COUT_YELLOW "\033[33m"
#define COUT_BLUE "\033[34m"

#define COUT_BOLD "\033[1m"
#define COUT_MSG_BOLD_ETRS COUT_BOLD << "[ETRS] " << COUT_RESET
#define COUT_MSG_BOLD_ETRS_ERROR COUT_RED << COUT_BOLD << "[ETRS ERROR] " << COUT_RESET
#define COUT_MSG_BOLD_ETRS_SUCCESS COUT_GREEN << COUT_BOLD << "[ETRS SUCCESS] " << COUT_RESET
#define COUT_MSG_BOLD_ETRS_WARNING COUT_YELLOW << COUT_BOLD << "[ETRS WARNING] " << COUT_RESET

namespace cas {
    namespace utility {
        // 读取配置
        class Config {
        private:
            string config_path;
            map<string, string> config_map;

        public:
            Config();

            Config(const string config_path, const char comment_char = COMMENT_CHAR);

            string get(const string key);

            float getFloat(const string key);

            int getInt(const string key);

            bool getBool(const string key);

            bool set(const string key, const string value);
        };

        class Debug {
        public:
            template <typename... Args> static void CoutInfo(const char *format, Args... args) {
                cout << COUT_MSG_BOLD_ETRS << fmt::format(format, args...) << COUT_RESET << endl;
            }

            template <typename... Args> static void CoutError(const char *format, Args... args) {
                cout << COUT_MSG_BOLD_ETRS_ERROR << fmt::format(format, args...) << COUT_RESET << endl;
            }

            template <typename... Args> static void CoutSuccess(const char *format, Args... args) {
                cout << COUT_MSG_BOLD_ETRS_SUCCESS << fmt::format(format, args...) << COUT_RESET << endl;
            }

            template <typename... Args> static void CoutWarning(const char *format, Args... args) {
                cout << COUT_MSG_BOLD_ETRS_WARNING << fmt::format(format, args...) << COUT_RESET << endl;
            }

            template <typename... Args> static void CoutDebug(const char *format, Args... args) {
                cout << "[ETRS DEBUG...] " << fmt::format(format, args...) << COUT_RESET << endl;
            }

            // 百分比变化，使用 std::flush;
            template <typename... Args> static void CoutFlush(const char *format, Args... args) {
                cout << '\r' << COUT_MSG_BOLD_ETRS << fmt::format(format, args...) << COUT_RESET << std::flush;
            }

            template <typename... Args>
            static void CoutSection(const string &sectionTitle, const char *format, Args... args) {
                const std::string line = std::string(10, '=');
                cout << line << endl;
                cout << sectionTitle << endl;
                cout << fmt::format(format, args...) << endl;
                cout << line << endl;
            }
        };

    } // namespace utility
} // namespace cas

#endif
