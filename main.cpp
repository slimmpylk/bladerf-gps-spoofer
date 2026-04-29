// ─────────────────────────────────────────────────────────────────────────────
// main.cpp — bladerf-gps-spoofer entry point
//
// WIRES TOGETHER:
//   ca_code     → C/A Gold codes for each PRN
//   ephemeris   → satellite positions from RINEX
//   nav_message → navigation data bits
//   modulator   → per-channel I/Q samples
//   iq_stream   → multi-channel sum → SC16Q11
//   bladerf_tx  → stream to hardware at 1575.42 MHz
//
// USAGE:
//   ./gps-spoofer --rinex data/today.rnx \
//                 --lat 60.1699 --lon 24.9384 --alt 25 \
//                 --gain 20
//
// INSIDE RF-SHIELDED ENCLOSURE ONLY.
// ─────────────────────────────────────────────────────────────────────────────

#include "ca_code.hpp"
#include "ephemeris.hpp"
#include "nav_message.hpp"
#include "modulator.hpp"
#include "iq_stream.hpp"
#include "bladerf/include/bladerf_tx.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Signal handler for clean Ctrl+C shutdown
//
// WHY std::atomic<bool>?
//   Signal handlers run asynchronously — they can interrupt any line of
//   main(). A plain bool is not safe to read/write from two contexts
//   simultaneously. std::atomic<bool> guarantees safe concurrent access.
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};

static void signal_handler(int) {
    g_running = false;
    std::cerr << "\n[main] Shutting down...\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Simple CLI argument parser
// ─────────────────────────────────────────────────────────────────────────────
static std::string get_arg(int argc, char* argv[],
                            const std::string& flag,
                            const std::string& default_val = "") {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == flag) return argv[i + 1];
    }
    return default_val;
}

static bool has_flag(int argc, char* argv[], const std::string& flag) {
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == flag) return true;
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// is_satellite_visible()
//
// A satellite is "visible" if its elevation above the horizon is > 10°.
// Receivers generally don't track satellites below the horizon — the
// signal has to pass through too much atmosphere, degrading accuracy.
//
// We compute elevation from the angle between the receiver-to-satellite
// vector and the receiver's local horizontal plane.
// ─────────────────────────────────────────────────────────────────────────────
static bool is_satellite_visible(
    const gps::EcefPosition& rx,
    const gps::SatelliteState& sv,
    double min_elevation_deg = 10.0)
{
    // Vector from receiver to satellite
    const double dx = sv.x - rx.x;
    const double dy = sv.y - rx.y;
    const double dz = sv.z - rx.z;
    const double range = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Receiver unit normal vector (points "up" from Earth's surface at rx)
    const double rx_mag = std::sqrt(rx.x*rx.x + rx.y*rx.y + rx.z*rx.z);
    const double nx = rx.x / rx_mag;
    const double ny = rx.y / rx_mag;
    const double nz = rx.z / rx_mag;

    // Dot product of satellite direction with "up" gives sin(elevation)
    const double sin_el = (dx*nx + dy*ny + dz*nz) / range;
    const double elevation_deg = std::asin(sin_el) * 180.0 / std::numbers::pi;

    return elevation_deg >= min_elevation_deg;
}


// ─────────────────────────────────────────────────────────────────────────────
// main()
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {

    // ── Usage ─────────────────────────────────────────────────────────────
    if (has_flag(argc, argv, "--help") || argc < 2) {
        std::cout <<
            "bladerf-gps-spoofer — GPS L1 C/A signal generator\n"
            "RF-SHIELDED ENCLOSURE ONLY\n\n"
            "Usage:\n"
            "  ./gps-spoofer --rinex FILE [options]\n\n"
            "Options:\n"
            "  --rinex FILE    RINEX 3 navigation file (required)\n"
            "  --lat  DEG      Target latitude  (default: 60.1699, Helsinki)\n"
            "  --lon  DEG      Target longitude (default: 24.9384)\n"
            "  --alt  M        Target altitude in metres (default: 25)\n"
            "  --gain DB       TX gain in dB, 0–40 (default: 0)\n"
            "  --help          Show this message\n\n"
            "Example:\n"
            "  ./gps-spoofer --rinex data/brdc0010.25n --lat 40.7608 --lon -111.891 --alt 1288 --gain 20\n";
        return 0;
    }

    // ── Parse arguments ───────────────────────────────────────────────────
    const std::string rinex_file = get_arg(argc, argv, "--rinex");
    if (rinex_file.empty()) {
        std::cerr << "[main] Error: --rinex FILE is required\n";
        return 1;
    }

    gps::GeodeticPosition target_geo;
    target_geo.lat_deg = std::stod(get_arg(argc, argv, "--lat",  "60.1699"));
    target_geo.lon_deg = std::stod(get_arg(argc, argv, "--lon",  "24.9384"));
    target_geo.alt_m   = std::stod(get_arg(argc, argv, "--alt",  "25.0"));

    const int gain_db = std::stoi(get_arg(argc, argv, "--gain", "0"));

    std::cerr << "[main] Target: lat=" << target_geo.lat_deg
              << " lon=" << target_geo.lon_deg
              << " alt=" << target_geo.alt_m << "m\n";

    // ── Convert target position to ECEF ───────────────────────────────────
    const gps::EcefPosition rx_pos = gps::geodetic_to_ecef(target_geo);
    const gps::EcefVelocity rx_vel = {0.0, 0.0, 0.0};  // stationary receiver

    std::cerr << "[main] Receiver ECEF: X=" << rx_pos.x/1e3
              << " Y=" << rx_pos.y/1e3
              << " Z=" << rx_pos.z/1e3 << " km\n";

    // ── Parse RINEX ephemeris ─────────────────────────────────────────────
    std::cerr << "[main] Loading RINEX: " << rinex_file << "\n";
    const auto all_ephs = gps::parse_rinex_nav(rinex_file);
    if (all_ephs.empty()) {
        std::cerr << "[main] Error: no ephemeris found in " << rinex_file << "\n";
        return 1;
    }

    // ── Get current GPS time ──────────────────────────────────────────────
    const double gps_time = gps::current_gps_time();
    const double gps_tow  = std::fmod(gps_time, 604800.0);  // time of week
    std::cerr << "[main] GPS TOW: " << gps_tow << " s\n";

    // ── Find visible satellites and build channels ────────────────────────
    std::cerr << "[main] Building signal channels...\n";

    std::vector<gps::ChannelConfig>  channels;
    std::vector<gps::ModulatorState> states;

    for (int prn = 1; prn <= gps::kNumPRN; ++prn) {
        const auto eph_opt = gps::find_best_ephemeris(all_ephs, prn, gps_time);
        if (!eph_opt) continue;
        if (!eph_opt->valid) continue;

        const auto sat = gps::compute_satellite_state(*eph_opt, gps_time);

        if (!is_satellite_visible(rx_pos, sat)) continue;

        const auto nav = gps::encode_nav_message(*eph_opt, gps_tow);
        const auto cfg = gps::make_channel(prn, sat, rx_pos, rx_vel,
                                           gps_time, nav);

        gps::ModulatorState state;
        state.code_phase    = cfg.code_phase;
        state.carrier_phase = cfg.carrier_phase;

        channels.push_back(cfg);
        states.push_back(state);

        std::cerr << "  PRN " << prn
                  << "  Doppler=" << static_cast<int>(cfg.doppler_hz) << " Hz"
                  << "  CodePhase=" << static_cast<int>(cfg.code_phase) << " chips\n";
    }

    if (channels.empty()) {
        std::cerr << "[main] Error: no visible satellites found. "
                     "Check RINEX file is current.\n";
        return 1;
    }

    std::cerr << "[main] " << channels.size() << " satellites visible\n";

    // ── Open bladeRF ──────────────────────────────────────────────────────
    gps::BladeRFConfig rf_cfg;
    rf_cfg.tx_gain_db = gain_db;

    std::cerr << "[main] Opening bladeRF...\n";
    gps::BladeRFTX tx(rf_cfg);

    // ── Set up signal handler for clean Ctrl+C shutdown ───────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── Main transmit loop ────────────────────────────────────────────────
    std::cerr << "[main] Transmitting... (Ctrl+C to stop)\n";

    gps::IQStream iq_stream(static_cast<int>(channels.size()));

    // Geometry update interval: recompute satellite positions every second.
    // WHY every second and not every epoch?
    //   Satellite positions change ~3.9 km/s but geometry only changes
    //   meaningfully over seconds. Recomputing every 1ms would waste CPU.
    //   Every 1 second (1000 epochs) is precise enough for Doppler accuracy.
    constexpr int kGeometryUpdateEpochs = 1000;
    int epoch_count = 0;

    while (g_running) {

        // ── Update satellite geometry every second ─────────────────────
        if (epoch_count % kGeometryUpdateEpochs == 0) {
            const double t = gps::current_gps_time();
            const double tow = std::fmod(t, 604800.0);

            for (size_t ch = 0; ch < channels.size(); ++ch) {
                const int prn = channels[ch].prn;
                const auto eph_opt = gps::find_best_ephemeris(all_ephs, prn, t);
                if (!eph_opt) continue;

                const auto sat  = gps::compute_satellite_state(*eph_opt, t);
                const auto nav  = gps::encode_nav_message(*eph_opt, tow);
                channels[ch]    = gps::make_channel(prn, sat, rx_pos, rx_vel,
                                                    t, nav);
                // Preserve phase continuity — keep existing phase state
                // only update the Doppler/code-phase geometry
            }
        }

        // ── Generate one 1ms epoch per channel ────────────────────────
        iq_stream.reset();

        for (size_t ch = 0; ch < channels.size(); ++ch) {
            std::vector<std::complex<float>> epoch_samples;
            gps::generate_epoch(channels[ch], states[ch], epoch_samples);
            iq_stream.add_channel_epoch(epoch_samples);
        }

        // ── Convert to SC16Q11 and transmit ───────────────────────────
        const auto buffer = iq_stream.finalise();
        tx.transmit(buffer);

        ++epoch_count;
    }

    std::cerr << "[main] Transmitted " << epoch_count << " epochs ("
              << epoch_count << " ms)\n";
    // BladeRFTX destructor handles clean shutdown automatically
    return 0;
}