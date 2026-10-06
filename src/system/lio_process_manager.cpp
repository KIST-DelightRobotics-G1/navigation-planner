#include "system/lio_process_manager.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace kist {

bool LioProcessManager::start() {
    if (!cfg_.manage) return false;       // nav does not manage LIO (run externally)
    if (running()) return true;           // already up (ours)

    // Build the same command run_lio_daemon uses: source the ROS2 overlay, then exec ros2 launch,
    // redirecting to the engine log. exec so the shell is replaced (clean group leader).
    const std::string cmd =
        "source '" + cfg_.ros_setup + "' 2>/dev/null; "
        "[ -f '" + cfg_.ws_setup + "' ] && source '" + cfg_.ws_setup + "'; "
        "exec ros2 launch '" + cfg_.launch_file + "' >> '" + cfg_.log_file + "' 2>&1 < /dev/null";

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "[LioProcessManager] fork failed: " << std::strerror(errno) << "\n";
        return false;
    }
    if (pid == 0) {
        // Child: become a new session leader so the whole launch tree is one killable group.
        ::setsid();
        ::execl("/bin/bash", "bash", "-lc", cmd.c_str(), (char*)nullptr);
        _exit(127);                        // exec failed
    }
    pgid_ = pid;                           // setsid makes the child its own pgid == pid
    std::cout << "[LioProcessManager] LIO engine started (pid " << pgid_
              << ", log: " << cfg_.log_file << ")\n";
    return true;
}

void LioProcessManager::stop() {
    if (pgid_ <= 0) return;
    std::cout << "[LioProcessManager] stopping LIO engine (pgid " << pgid_ << ")\n";
    ::killpg(pgid_, SIGINT);
    // give it a moment to shut down the driver + FAST-LIO cleanly, then force.
    for (int i = 0; i < 20; ++i) {         // up to ~2 s
        if (::waitpid(-pgid_, nullptr, WNOHANG) == -1 && errno == ECHILD) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ::killpg(pgid_, SIGKILL);
    while (::waitpid(-pgid_, nullptr, 0) > 0) {}   // reap the group
    pgid_ = -1;
}

bool LioProcessManager::running() const {
    if (pgid_ <= 0) return false;
    return ::kill(pgid_, 0) == 0;          // group leader alive
}

} // namespace kist
