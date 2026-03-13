#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <mutex>
#include <chrono>
#include <iomanip>

/**
 * @brief 简易日志系统
 * @details 支持多级别日志输出到控制台和文件
 */
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

    void setLogFile(const std::string& filename) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_logFile.is_open()) {
            m_logFile.close();
        }
        m_logFile.open(filename, std::ios::app);
        m_logToFile = m_logFile.is_open();
    }

    void log(Level level, const std::string& message, const std::string& file, int line) {
        if (level < m_minLevel) return;

        std::lock_guard<std::mutex> lock(m_mutex);

        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;

        std::ostringstream oss;
        oss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        oss << " [" << levelToString(level) << "] ";
        oss << message;
        oss << " (" << file << ":" << line << ")";

        std::string logLine = oss.str();

        // 输出到控制台
        std::cout << logLine << std::endl;

        // 输出到文件
        if (m_logToFile && m_logFile.is_open()) {
            m_logFile << logLine << std::endl;
        }
    }

    std::string levelToString(Level level) {
        switch (level) {
            case Level::DEBUG: return "DEBUG";
            case Level::INFO:  return "INFO";
            case Level::WARN:  return "WARN";
            case Level::ERROR: return "ERROR";
            case Level::FATAL: return "FATAL";
            default: return "UNKNOWN";
        }
    }

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

// 日志宏
#define LOG_DEBUG(msg) Logger::getInstance().log(Logger::Level::DEBUG, msg, __FILE__, __LINE__)
#define LOG_INFO(msg)  Logger::getInstance().log(Logger::Level::INFO, msg, __FILE__, __LINE__)
#define LOG_WARN(msg)  Logger::getInstance().log(Logger::Level::WARN, msg, __FILE__, __LINE__)
#define LOG_ERROR(msg) Logger::getInstance().log(Logger::Level::ERROR, msg, __FILE__, __LINE__)
#define LOG_FATAL(msg) Logger::getInstance().log(Logger::Level::FATAL, msg, __FILE__, __LINE__)

// 简化版本（不带文件行号）
#define LOG_DEBUG_SIMPLE(msg) Logger::getInstance().log(Logger::Level::DEBUG, msg, "", 0)
#define LOG_INFO_SIMPLE(msg)  Logger::getInstance().log(Logger::Level::INFO, msg, "", 0)
#define LOG_WARN_SIMPLE(msg)  Logger::getInstance().log(Logger::Level::WARN, msg, "", 0)
#define LOG_ERROR_SIMPLE(msg) Logger::getInstance().log(Logger::Level::ERROR, msg, "", 0)
#define LOG_FATAL_SIMPLE(msg) Logger::getInstance().log(Logger::Level::FATAL, msg, "", 0)
