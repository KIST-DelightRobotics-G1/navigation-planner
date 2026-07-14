// Deterministic probe of the pointcloud processor (no robot needed).
//
// Checks: passthrough when disabled, invert+offset transform, axis and
// radius filters, filter-sees-transformed-coordinates ordering, in-place
// compaction, metadata passthrough, option validation, and YAML parsing
// (full section, partial section, absent section).

#include "processing/pointcloud_processor.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <stdexcept>
#include <vector>

using namespace kist;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-46s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

static UnitreePointCloud frame_of(std::vector<float> xyz) {
    UnitreePointCloud f;
    f.stamp_ns = 42;
    f.frame_id = "utlidar_lidar";
    f.xyz = std::move(xyz);
    return f;
}

int main() {
    // ── passthrough ─────────────────────────────────────────────
    {
        PointCloudProcessor proc{{}};
        auto f = frame_of({1, 2, 3, -4, 5, -6});
        proc.process(f);
        check("all-disabled is a passthrough",
              f.xyz == std::vector<float>({1, 2, 3, -4, 5, -6}));
    }

    // ── transform ───────────────────────────────────────────────
    {
        PointCloudProcessorOptions opt;
        opt.transform.enabled  = true;
        opt.transform.invert_x = true;
        opt.transform.offset_x = 10.0f;
        opt.transform.offset_z = -1.0f;
        PointCloudProcessor proc{opt};

        auto f = frame_of({2, 3, 4});
        proc.process(f);
        check("transform: invert_x + offsets",
              f.xyz == std::vector<float>({8, 3, 3}));
    }

    // ── axis filter ─────────────────────────────────────────────
    {
        PointCloudProcessorOptions opt;
        opt.filter.enabled  = true;
        opt.filter.enable_z = true;
        opt.filter.min_z = 0.0f;
        opt.filter.max_z = 1.0f;
        PointCloudProcessor proc{opt};

        auto f = frame_of({1, 1, 0.5f,   2, 2, 2.0f,   3, 3, -0.1f,   4, 4, 1.0f});
        proc.process(f);
        check("z filter keeps in-range, compacts in order",
              f.xyz == std::vector<float>({1, 1, 0.5f, 4, 4, 1.0f}));
        check("metadata untouched by processing",
              f.stamp_ns == 42 && f.frame_id == "utlidar_lidar");
    }

    // ── radius filter ───────────────────────────────────────────
    {
        PointCloudProcessorOptions opt;
        opt.filter.enabled       = true;
        opt.filter.enable_radius = true;
        opt.filter.min_radius    = 1.0f;
        PointCloudProcessor proc{opt};

        // (0.5,0.5) inside r=1 -> dropped; (1,0) on the boundary -> kept;
        // z plays no part in the XY radius.
        auto f = frame_of({0.5f, 0.5f, 9,   1, 0, 9,   0, 0.99f, 9});
        proc.process(f);
        check("XY radius drops near-origin points",
              f.xyz == std::vector<float>({1, 0, 9}));
    }

    // ── filter runs on transformed coordinates ──────────────────
    {
        PointCloudProcessorOptions opt;
        opt.transform.enabled  = true;
        opt.transform.invert_x = true;
        opt.filter.enabled  = true;
        opt.filter.enable_x = true;
        opt.filter.min_x = -2.0f;
        opt.filter.max_x =  0.0f;
        PointCloudProcessor proc{opt};

        // raw x=1 -> transformed x=-1, inside [-2, 0] -> kept
        auto f = frame_of({1, 0, 0});
        proc.process(f);
        check("filter sees transformed coordinates",
              f.xyz.size() == 3 && f.xyz[0] == -1.0f);
    }

    // ── validation ──────────────────────────────────────────────
    {
        PointCloudProcessorOptions opt;
        opt.filter.enabled  = true;
        opt.filter.enable_y = true;
        opt.filter.min_y =  1.0f;
        opt.filter.max_y = -1.0f;
        bool threw = false;
        try {
            PointCloudProcessor proc{opt};
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        check("inverted y range rejected at construction", threw);
    }

    // ── YAML parsing ────────────────────────────────────────────
    {
        const auto node = YAML::Load(R"(
transform:
  enabled: true
  offset_z: 1.2
  invert_y: true
filter:
  enabled: true
  enable_radius: true
  min_radius: 0.5
)");
        const auto opt = pointcloud_processor_options_from_yaml(node);
        check("yaml: provided keys parsed",
              opt.transform.enabled && opt.transform.offset_z == 1.2f &&
              opt.transform.invert_y && opt.filter.enabled &&
              opt.filter.enable_radius && opt.filter.min_radius == 0.5f);
        check("yaml: missing keys keep defaults",
              opt.transform.offset_x == 0.0f && !opt.transform.invert_x &&
              !opt.filter.enable_x);
    }
    {
        const auto opt = pointcloud_processor_options_from_yaml(YAML::Node());
        check("yaml: absent section = all defaults (passthrough)",
              !opt.transform.enabled && !opt.filter.enabled);
    }

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
