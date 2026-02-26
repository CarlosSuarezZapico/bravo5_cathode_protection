/**
 *    @file  bravo_logger.h
 *    @brief methods, structures used....
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      24-Feb-2026
 *    Modification 24-Feb-2026
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#pragma once
#include <cstdint>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace bravo_utils {

    enum class LogLevel : uint8_t { Debug=0, Info=1, Warn=2, Error=3 };

    using LogCallback = std::function<void(LogLevel, std::string_view)>;

    inline const char* log_level_tag(LogLevel lvl) noexcept {
        switch (lvl) {
            case LogLevel::Debug: return "[DEBUG]";
            case LogLevel::Info:  return "[INFO ]";
            case LogLevel::Warn:  return "[WARN ]";
            case LogLevel::Error: return "[ERROR]";
        }
        return "[INFO ]";
    }

    inline void write_log_stderr(LogLevel lvl, std::string_view message) {
        static std::mutex output_mutex;
        std::lock_guard<std::mutex> lock(output_mutex);
        std::cerr << log_level_tag(lvl) << " " << message << '\n';
    }

    template <typename DashboardEnabledFn, typename DashboardPushFn>
    inline LogCallback make_dashboard_or_stderr_callback(
        DashboardEnabledFn&& dashboard_enabled,
        DashboardPushFn&& dashboard_push) {
        return [enabled = std::forward<DashboardEnabledFn>(dashboard_enabled),
                push = std::forward<DashboardPushFn>(dashboard_push)](LogLevel lvl, std::string_view message) {
            if (enabled()) {
                push(lvl, message);
                return;
            }
            write_log_stderr(lvl, message);
        };
    }

    class Logger {
        public:
            Logger() = default;
            explicit Logger(LogCallback cb) : cb_(std::move(cb)) {}

            void set_callback(LogCallback cb) { cb_ = std::move(cb); }

            void log(LogLevel lvl, std::string_view msg) const {
                if (cb_) cb_(lvl, msg);
            }

            template <class... Args>
            void log_stream(LogLevel lvl, Args&&... args) const {
                if (!cb_) return;
                std::ostringstream oss;
                (oss << ... << std::forward<Args>(args));
                const std::string s = oss.str();
                cb_(lvl, s);
            }

        private:
            LogCallback cb_;
    };

} // namespace bravo_utils

// Convenience macros (keep call-sites short)
#define BRAVO_LOG_DEBUG(logger, ...) (logger).log_stream(::bravo_utils::LogLevel::Debug, __VA_ARGS__)
#define BRAVO_LOG_INFO(logger, ...)  (logger).log_stream(::bravo_utils::LogLevel::Info,  __VA_ARGS__)
#define BRAVO_LOG_WARN(logger, ...)  (logger).log_stream(::bravo_utils::LogLevel::Warn,  __VA_ARGS__)
#define BRAVO_LOG_ERROR(logger, ...) (logger).log_stream(::bravo_utils::LogLevel::Error, __VA_ARGS__)
