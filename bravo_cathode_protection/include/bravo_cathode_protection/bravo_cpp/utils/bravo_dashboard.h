#pragma once

/**
 *    @file  bravo_dashboard.h
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
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <algorithm>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <sstream>

#include "bravo_cathode_protection/bravo_cpp/utils/bravo_logger.h"

namespace bravo_utils
{
    struct LogLine {
    uint64_t seq;
    std::chrono::steady_clock::time_point t;
    LogLevel lvl;
    std::string text;
    uint32_t repeat_count;
    };

    class TerminalDashboard {
        public:
        TerminalDashboard() : run_(false), enabled_(shouldEnable()), seq_(0) {
            if (enabled_) {
                start();
            }
        }

        ~TerminalDashboard() {
            stop();
        }

        bool is_enabled() const {
            return enabled_;
        }

        void start() {
            if (!enabled_) {
                return;
            }
            bool expected = false;
            if (!run_.compare_exchange_strong(expected, true)) {
                return;
            }
            enterTerminalUiMode();
            render_thread_ = std::thread([this]{ renderLoop(); });
        }

        void stop() {
            bool expected = true;
            if (!run_.compare_exchange_strong(expected, false)) {
                return;
            }
            if (render_thread_.joinable()) render_thread_.join();
            leaveTerminalUiMode();
        }

        void push(LogLevel lvl, std::string s) {
            if (!enabled_) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            {
            std::lock_guard<std::mutex> lk(m_);
            if (!lines_.empty()) {
                LogLine& last = lines_.back();
                if (last.lvl == lvl && last.text == s) {
                    ++last.repeat_count;
                    last.t = now;
                    return;
                }
            }

            LogLine line{++seq_, now, lvl, std::move(s), 1};
            if (lines_.size() >= max_lines_) lines_.pop_front();
            lines_.push_back(std::move(line));
            }
        }

        // Update fixed status block shown in the top section.
        void setArmStatus(std::string state, double manipulability, bool udp_running, double udp_rx_hz = 0.0) {
            if (!enabled_) {
                return;
            }
            std::lock_guard<std::mutex> lk(m_);
            arm_state_ = std::move(state);
            manip_ = manipulability;
            udp_running_ = udp_running;
            udp_rx_hz_ = udp_rx_hz;
        }

        void setInteractionStatusX(double vel_ee_x, double desired_force_x, double exerted_force_x) {
            if (!enabled_) {
                return;
            }
            std::lock_guard<std::mutex> lk(m_);
            vel_ee_x_ = vel_ee_x;
            desired_force_x_ = desired_force_x;
            exerted_force_x_ = exerted_force_x;
        }

        private:
        static bool shouldEnable() {
            const bool has_tty = (::isatty(STDOUT_FILENO) == 1);
            const char* dashboard_env = std::getenv("BRAVO_DASHBOARD");
            const bool dashboard_requested =
                (dashboard_env == nullptr) || (std::string(dashboard_env) != "0");
            return has_tty && dashboard_requested;
        }

        static std::string sanitizeAndClamp(std::string s, size_t max_chars) {
            for (char& c : s) {
                if (c == '\n' || c == '\r' || c == '\t') {
                    c = ' ';
                }
            }
            if (s.size() <= max_chars) {
                return s;
            }
            if (max_chars <= 3) {
                return s.substr(0, max_chars);
            }
            return s.substr(0, max_chars - 3) + "...";
        }

        static std::string formatHz(double hz) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << hz;
            return oss.str();
        }

        void enterTerminalUiMode() {
            if (ui_mode_active_) {
                return;
            }
            // Alternate screen buffer + hide cursor for static dashboard effect.
            std::cout << "\x1b[?1049h\x1b[?25l\x1b[2J\x1b[H";
            std::cout.flush();
            ui_mode_active_ = true;
        }

        void leaveTerminalUiMode() {
            if (!ui_mode_active_) {
                return;
            }
            // Show cursor + leave alternate screen.
            std::cout << "\x1b[?25h\x1b[?1049l";
            std::cout.flush();
            ui_mode_active_ = false;
        }

        void renderLoop() {
            using namespace std::chrono_literals;
            while (run_.load()) {
            std::deque<LogLine> snapshot;
            std::string arm_state;
            double manip;
            bool udp_running;
            double udp_rx_hz;
            double vel_ee_x;
            double desired_force_x;
            double exerted_force_x;

            {
                std::lock_guard<std::mutex> lk(m_);
                snapshot = lines_;
                arm_state = arm_state_;
                manip = manip_;
                udp_running = udp_running_;
                udp_rx_hz = udp_rx_hz_;
                vel_ee_x = vel_ee_x_;
                desired_force_x = desired_force_x_;
                exerted_force_x = exerted_force_x_;
            }

            std::vector<std::string> log_rows;
            const size_t begin = snapshot.size() > visible_log_rows_ ? snapshot.size() - visible_log_rows_ : 0;
            log_rows.reserve(visible_log_rows_);
            for (size_t i = begin; i < snapshot.size(); ++i) {
                const auto& l = snapshot[i];
                const char* tag = "[INFO ]";
                if (l.lvl == LogLevel::Error) {
                    tag = "[ERROR]";
                } else if (l.lvl == LogLevel::Warn) {
                    tag = "[WARN ]";
                } else if (l.lvl == LogLevel::Debug) {
                    tag = "[DEBUG]";
                }
                std::string row = std::string(tag) + " " + sanitizeAndClamp(l.text, max_log_chars_);
                if (l.repeat_count > 1) {
                    row += " (x" + std::to_string(l.repeat_count) + ")";
                }
                log_rows.push_back(std::move(row));
            }

            // Cursor home and redraw fixed number of lines (no terminal scrolling).
            std::cout << "\x1b[H";
            auto print_line = [](const std::string& line) {
                std::cout << "\x1b[2K" << line << "\n";
            };

            print_line("\x1b[31m=== BRAVO5 CATHODE PROTECTION DASHBOARD ===\x1b[0m");
            print_line("");
            print_line("Arm Status:");
            print_line(" CP state:                " + sanitizeAndClamp(arm_state, 48));
            print_line("  manipulability:       " + std::to_string(manip));
            print_line(std::string("  udp running:          ") + (udp_running ? "YES" : "NO"));
            print_line("  Freq Feedback [Hz]:     " + formatHz(udp_rx_hz));
            print_line("  vel_ee.x [m/s]:         " + std::to_string(vel_ee_x));
            print_line("  desired_force.x [N]:    " + std::to_string(desired_force_x));
            print_line("  exerted_force.x [N]:    " + std::to_string(exerted_force_x));
            print_line("----------------------------------------");
            print_line("Recent logs (tail):");

            for (const auto& row : log_rows) {
                print_line(row);
            }
            for (size_t i = log_rows.size(); i < visible_log_rows_; ++i) {
                print_line("");
            }

            std::cout.flush();
            std::this_thread::sleep_for(100ms); // 10 Hz refresh
            }
        }

        std::atomic<bool> run_;
        std::thread render_thread_;
        bool enabled_;
        bool ui_mode_active_ = false;

        std::mutex m_;
        std::deque<LogLine> lines_;
        const size_t max_lines_ = 25;
        const size_t visible_log_rows_ = 18;
        const size_t max_log_chars_ = 110;

        std::atomic<uint64_t> seq_;
        std::string arm_state_ = "INIT";
        double manip_ = 0.0;
        bool udp_running_ = false;
        double udp_rx_hz_ = 0.0;
        double vel_ee_x_ = 0.0;
        double desired_force_x_ = 0.0;
        double exerted_force_x_ = 0.0;
    };
}
