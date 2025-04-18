#pragma once

#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

inline std::string getCurrentIsoTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    
    std::tm tm_buf;
#ifdef _WIN32
    gmtime_s(&tm_buf, &time_t_now);
    ss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
#else
    gmtime_r(&time_t_now, &tm_buf);
    ss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
#endif
    
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count() << "Z";
    
    return ss.str();
} 