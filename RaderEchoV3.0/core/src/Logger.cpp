#include "Logger.h"

void Logger::setLogFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_logFile.is_open()) {
        m_logFile.close();
    }
    m_logFile.open(filename, std::ios::app);
    m_logToFile = m_logFile.is_open();
}

void Logger::log(Level level, const std::string& message, const std::string& file, int line) {
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

    std::cout << logLine << std::endl;

    if (m_logToFile && m_logFile.is_open()) {
        m_logFile << logLine << std::endl;
    }
}

std::string Logger::levelToString(Level level) {
    switch (level) {
        case Level::DEBUG: return "DEBUG";
        case Level::INFO:  return "INFO";
        case Level::WARN:  return "WARN";
        case Level::ERROR: return "ERROR";
        case Level::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}
