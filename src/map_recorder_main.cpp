// kist-map-recorder — deployment entry for building the prior PCD map from the LIO registered scans.
//
// All wiring lives in MapRecorderSystem (readers + the MapRecorder worker); this binary only runs
// the lifecycle and, on quit, saves. Output is ALWAYS maps/map.pcd (+ maps/map.uwb sidecar); if they
// exist you are asked to confirm the overwrite after recording, before anything is written.
//
//   ./build/kist-map-recorder [config=config/config.yaml]
//   -> drive the whole environment slowly, then Ctrl+C to save.
// The voxel leaf is fixed (0.05 m); to change it edit MapRecorderConfig in
// include/recorder/map_recorder.hpp.

#include "system/map_recorder.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>
#include <thread>

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    const std::string out_path    = "maps/map.pcd";              // fixed output (+ maps/map.uwb sidecar)

    kist::MapRecorderSystem sys;
    sys.install_signal_handlers();
    if (!sys.start(config_path)) return 1;

    auto last_print = std::chrono::steady_clock::now();
    while (!sys.quit_requested()) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_print >= std::chrono::milliseconds(1000)) {
            last_print = now;
            std::printf("[kist-map-recorder] scans=%llu raw=%lluk voxels=%zu%s\n",
                        (unsigned long long)sys.scans(), (unsigned long long)(sys.raw_points()/1000),
                        sys.voxels(), sys.has_origin() ? "  [origin captured]" : "");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    sys.stop();

    if (sys.voxels() == 0) {
        std::fprintf(stderr, "[kist-map-recorder] no points received — nothing saved.\n");
        return 1;
    }

    // Output paths (fixed): maps/map.pcd + maps/map.uwb. Ensure maps/ exists.
    std::error_code ec;
    std::filesystem::create_directories("maps", ec);
    std::string sidecar = out_path;                              // maps/map.pcd -> maps/map.uwb
    { const auto dot = sidecar.rfind(".pcd");
      if (dot != std::string::npos && dot == sidecar.size()-4) sidecar.replace(dot,4,".uwb"); else sidecar += ".uwb"; }

    // Overwrite guard: if either output exists, confirm before writing anything.
    if (std::filesystem::exists(out_path) || std::filesystem::exists(sidecar)) {
        std::printf("\n[kist-map-recorder] WARNING: %s and/or %s already exist and will be OVERWRITTEN.\n"
                    "Overwrite? [y/N]: ", out_path.c_str(), sidecar.c_str());
        std::fflush(stdout);
        std::string ans;
        std::getline(std::cin, ans);
        if (ans != "y" && ans != "Y" && ans != "yes") {
            std::printf("[kist-map-recorder] aborted — existing map kept, this recording discarded.\n");
            return 0;
        }
    }

    if (!sys.save(out_path)) {
        std::fprintf(stderr, "[kist-map-recorder] save failed (%s).\n", out_path.c_str());
        return 1;
    }
    return 0;
}
