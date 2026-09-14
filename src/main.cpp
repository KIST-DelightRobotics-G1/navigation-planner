// kist-navigation-planner — deployment entry. Runs perception + planning + rviz viz.
// This build does NOT command the robot (no velocity output); it maps, plans, and
// visualizes. Set a goal with rviz's "2D Goal Pose" tool.
//
//   ./build/kist-navigation-planner [config.yaml]
//
// All wiring lives in NavSystem; this binary only runs the lifecycle.

#include "system/nav_system.hpp"

#include <chrono>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    kist::NavSystem sys;
    sys.install_signal_handlers();

    if (!sys.start(config_path)) return 1;

    while (!sys.quit_requested())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    sys.stop();
    return 0;
}
