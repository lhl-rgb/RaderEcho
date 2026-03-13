#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <mutex>
#include <chrono>
#include <iomanip>

#define _CRT_SECURE_NO_WARNINGS

class Logger {
public:
    enum class Level {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        FATAL = 4
    };

    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void setLevel(Level level) { m_minLevel = level; }

    void setLogFile(const std::string& filename);
    void log(Level level, const std::string& message, const std::string& file, int line);
    std::string levelToString(Level level);

private:
    Logger() : m_minLevel(Level::INFO), m_logToFile(false) {}
    ~Logger() { if (m_logFile.is_open()) m_logFile.close(); }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    Level m_minLevel;
    std::ofstream m_logFile;
    bool m_logToFile;
    std::mutex m_mutex;
};

#define LOG_DEBUG(msg) Logger::getInstance().log(Logger::Level::DEBUG, msg, __FILE__, __LINE__)
#define LOG_INFO(msg)  Logger::getInstance().log(Logger::Level::INFO, msg, __FILE__, __LINE__)
#define LOG_WARN(msg)  Logger::getInstance().log(Logger::Level::WARN, msg, __FILE__, __LINE__)
#define LOG_ERROR(msg) Logger::getInstance().log(Logger::Level::ERROR, msg, __FILE__, __LINE__)
#define LOG_FATAL(msg) Logger::getInstance().log(Logger::Level::FATAL, msg, __FILE__, __LINE__)

#define LOG_DEBUG_SIMPLE(msg) Logger::getInstance().log(Logger::Level::DEBUG, msg, "", 0)
#define LOG_INFO_SIMPLE(msg)  Logger::getInstance().log(Logger::Level::INFO, msg, "", 0)
#define LOG_WARN_SIMPLE(msg)  Logger::getInstance().log(Logger::Level::WARN, msg, "", 0)
#define LOG_ERROR_SIMPLE(msg) Logger::getInstance().log(Logger::Level::ERROR, msg, "", 0)
#define LOG_FATAL_SIMPLE(msg) Logger::getInstance().log(Logger::Level::FATAL, msg, "", 0)
