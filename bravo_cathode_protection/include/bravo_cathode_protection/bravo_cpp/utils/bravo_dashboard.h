#pragma once

/**
 *    @file  bravo_dashboard.h
 *    @brief methods, structures used....
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      24-Feb-2024
 *    Modification 24-Feb-2024
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include "bravo_cathode_protection/bravo_cpp/utils/bravo_logger.h"

namespace bravo_utils
{
    struct LogLine {
    uint64_t seq;
    std::chrono::steady_clock::time_point t;
    LogLevel lvl;
    std::string text;
    };

    class TerminalDashboard {
        public:
        TerminalDashboard() : run_(false), seq_(0) {}

        void start() {
            run_.store(true);
            render_thread_ = std::thread([this]{ renderLoop(); });
        }

        void stop() {
            run_.store(false);
            if (render_thread_.joinable()) render_thread_.join();
        }

        void push(LogLevel lvl, std::string s) {
            LogLine line{++seq_, std::chrono::steady_clock::now(), lvl, std::move(s)};
            {
            std::lock_guard<std::mutex> lk(m_);
            // ring buffer
            if (lines_.size() >= max_lines_) lines_.pop_front();
            lines_.push_back(std::move(line));
            }
        }

        // update telemetry values from your node (call from wherever safe)
        void setTelemetry(double io_hz, double js_hz, double ft_hz, double manipulability) {
            std::lock_guard<std::mutex> lk(m_);
            io_hz_ = io_hz; js_hz_ = js_hz; ft_hz_ = ft_hz; manip_ = manipulability;
        }

        private:
        void renderLoop() {
            using namespace std::chrono_literals;
            while (run_.load()) {
            std::deque<LogLine> snapshot;
            double io_hz, js_hz, ft_hz, manip;

            {
                std::lock_guard<std::mutex> lk(m_);
                snapshot = lines_;
                io_hz = io_hz_; js_hz = js_hz_; ft_hz = ft_hz_; manip = manip_;
            }

            // Clear + redraw
            std::cout << "\x1b[2J\x1b[H"; // clear screen + cursor home

            std::cout << "=== BRAVO HANDLER DASHBOARD ===\n\n";
            std::cout << "Telemetry:\n";
            std::cout << "  bravo_io Hz:          " << io_hz << "\n";
            std::cout << "  joint_state Hz:       " << js_hz << "\n";
            std::cout << "  FT Hz:                " << ft_hz << "\n";
            std::cout << "  manipulability:       " << manip << "\n";
            std::cout << "\n";
            std::cout << "Recent logs (tail):\n";
            for (auto const& l : snapshot) {
                const char* tag = "[INFO ]";
                if (l.lvl == LogLevel::Error) {
                    tag = "[ERROR]";
                } else if (l.lvl == LogLevel::Warn) {
                    tag = "[WARN ]";
                } else if (l.lvl == LogLevel::Debug) {
                    tag = "[DEBUG]";
                }
                std::cout << tag << " " << l.text << "\n";
            }

            std::cout.flush();
            std::this_thread::sleep_for(100ms); // 10 Hz refresh
            }
        }

        std::atomic<bool> run_;
        std::thread render_thread_;

        std::mutex m_;
        std::deque<LogLine> lines_;
        const size_t max_lines_ = 60;

        std::atomic<uint64_t> seq_;
        double io_hz_ = 0.0, js_hz_ = 0.0, ft_hz_ = 0.0, manip_ = 0.0;
    };
}
