// ─────────────────────────────────────────────────────────────────────────────
// bladerf_tx.cpp — bladeRF 2.0 micro TX Implementation
//
// This file is intentionally short — the signal generation happens in
// gps_core. This file is only responsible for getting bytes into the
// bladeRF hardware correctly.
//
// THE LIBBLADERF TX FLOW:
//   1. bladerf_open()               — find and open the USB device
//   2. bladerf_set_frequency()      — tune to 1575.42 MHz
//   3. bladerf_set_sample_rate()    — 2.048 Msps
//   4. bladerf_set_bandwidth()      — 2.5 MHz
//   5. bladerf_set_gain()           — TX gain in dB
//   6. bladerf_sync_config()        — set up DMA ring buffers
//   7. bladerf_enable_module()      — start the TX chain
//   8. bladerf_sync_tx()            — feed samples repeatedly
//   9. bladerf_enable_module(false) — stop TX
//  10. bladerf_close()              — release USB device
// ─────────────────────────────────────────────────────────────────────────────

#include "../include/bladerf_tx.hpp"
#include <iostream>
#include <cstring>   // std::memcpy

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// check() — libbladeRF error handler
//
// libbladeRF functions return 0 on success, negative on error.
// bladerf_strerror() converts error codes to human-readable strings.
//
// WHY throw instead of returning bool?
//   Every libbladeRF call must succeed. Silently ignoring a failure
//   (e.g. failed to set frequency) would transmit on the wrong frequency
//   with no indication anything is wrong. Throwing forces the caller
//   to handle the error or let it propagate to main().
// ─────────────────────────────────────────────────────────────────────────────
void BladeRFTX::check(int status, const std::string& context) {
    if (status != 0) {
        throw std::runtime_error(
            "[bladeRF] " + context + ": " + bladerf_strerror(status)
        );
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Constructor — open device and configure everything
// ─────────────────────────────────────────────────────────────────────────────
BladeRFTX::BladeRFTX(const BladeRFConfig& config)
    : config_(config)
{
    int status;

    // ── Step 1: Open the bladeRF device ──────────────────────────────────
    // nullptr means "open the first bladeRF found on USB".
    // If you have multiple bladeRFs, pass a serial number string like
    // "bladerf:serial=4e6...abc" instead of nullptr.
    status = bladerf_open(&dev_, nullptr);
    check(status, "bladerf_open");

    // ── Log device info ───────────────────────────────────────────────────
    std::cerr << "[bladeRF] Opened device\n";
    std::cerr << "[bladeRF] Firmware: " << firmware_version() << "\n";
    std::cerr << "[bladeRF] FPGA:     " << fpga_version() << "\n";

    // ── Step 2: Set TX frequency ──────────────────────────────────────────
    // BLADERF_CHANNEL_TX(0) = TX1 = the first (and only) TX channel we use.
    // bladeRF 2.0 micro has TX1 and TX2 but we only need TX1 for GPS L1.
    status = bladerf_set_frequency(dev_,
                                   BLADERF_CHANNEL_TX(0),
                                   config_.frequency_hz);
    check(status, "bladerf_set_frequency");
    std::cerr << "[bladeRF] Frequency: " << config_.frequency_hz << " Hz\n";

    // ── Step 3: Set sample rate ───────────────────────────────────────────
    // bladerf_set_sample_rate() takes a requested rate and writes back
    // the actual rate the hardware achieved (PLL granularity).
    // At 2.048 Msps the bladeRF hits it exactly.
    uint32_t actual_rate;
    status = bladerf_set_sample_rate(dev_,
                                     BLADERF_CHANNEL_TX(0),
                                     config_.sample_rate,
                                     &actual_rate);
    check(status, "bladerf_set_sample_rate");
    std::cerr << "[bladeRF] Sample rate: " << actual_rate << " sps\n";

    // ── Step 4: Set RF bandwidth ──────────────────────────────────────────
    // The bandwidth filter removes out-of-band noise and adjacent channel
    // interference. 2.5 MHz covers the GPS C/A main lobe (±1.023 MHz)
    // with a small guard band. Going wider adds noise; narrower clips signal.
    uint32_t actual_bw;
    status = bladerf_set_bandwidth(dev_,
                                   BLADERF_CHANNEL_TX(0),
                                   config_.bandwidth_hz,
                                   &actual_bw);
    check(status, "bladerf_set_bandwidth");
    std::cerr << "[bladeRF] Bandwidth: " << actual_bw << " Hz\n";

    // ── Step 5: Set TX gain ───────────────────────────────────────────────
    // bladerf_set_gain() sets the overall TX gain in dB.
    // The bladeRF 2.0 micro AD9361 supports -23.5 to +66.5 dB TX overall.
    // Start at 0 dB and increase in 5 dB steps until the phone locks.
    // Inside a shielded enclosure you'll likely need 10–20 dB.
    status = bladerf_set_gain(dev_,
                              BLADERF_CHANNEL_TX(0),
                              config_.tx_gain_db);
    check(status, "bladerf_set_gain");
    std::cerr << "[bladeRF] TX gain: " << config_.tx_gain_db << " dB\n";

    // ── Step 6: Configure synchronous TX interface ────────────────────────
    // This sets up the DMA ring buffer for streaming.
    //
    // BLADERF_FORMAT_SC16_Q11: the SC16Q11 format our iq_stream produces.
    // num_buffers:   ring buffer depth (more = more latency tolerance)
    // buffer_size:   samples per buffer (must be multiple of 1024)
    // num_transfers: concurrent USB bulk transfers
    //
    // WHY synchronous instead of asynchronous?
    //   Async TX uses callbacks — harder to integrate with our sample
    //   generation loop. Sync TX blocks until the buffer is queued,
    //   which is simpler and sufficient for our ~2Msps throughput.
    status = bladerf_sync_config(dev_,
                                 BLADERF_CHANNEL_LAYOUT_TX_X1,
                                 BLADERF_FORMAT_SC16_Q11,
                                 config_.num_buffers,
                                 config_.buffer_size,
                                 config_.num_transfers,
                                 config_.timeout_ms);
    check(status, "bladerf_sync_config");

    // ── Step 7: Enable the TX module ─────────────────────────────────────
    // This powers up the TX chain in the AD9361. Without this, calling
    // bladerf_sync_tx() will queue samples but nothing is transmitted.
    status = bladerf_enable_module(dev_, BLADERF_CHANNEL_TX(0), true);
    check(status, "bladerf_enable_module");

    std::cerr << "[bladeRF] TX enabled — transmitting on 1575.42 MHz\n";
}


// ─────────────────────────────────────────────────────────────────────────────
// Destructor — clean shutdown
//
// noexcept: destructors must never throw. If bladerf_close() fails,
// we log the error but continue — there's nothing else we can do.
// ─────────────────────────────────────────────────────────────────────────────
BladeRFTX::~BladeRFTX() noexcept {
    if (dev_ == nullptr) return;

    // Disable TX module (stops the AD9361 TX chain)
    int status = bladerf_enable_module(dev_, BLADERF_CHANNEL_TX(0), false);
    if (status != 0) {
        std::cerr << "[bladeRF] Warning: failed to disable TX: "
                  << bladerf_strerror(status) << "\n";
    }

    // Close the device handle and release USB resources
    bladerf_close(dev_);
    dev_ = nullptr;

    std::cerr << "[bladeRF] Device closed cleanly\n";
}


// ─────────────────────────────────────────────────────────────────────────────
// Move constructor and assignment
//
// WHY implement move? So BladeRFTX can be stored in a std::vector or
// returned from a factory function without copying (which is deleted).
// Move transfers ownership of dev_ to the new object and nulls the old one
// so its destructor doesn't close the device.
// ─────────────────────────────────────────────────────────────────────────────
BladeRFTX::BladeRFTX(BladeRFTX&& other) noexcept
    : dev_(other.dev_), config_(other.config_)
{
    other.dev_ = nullptr;   // prevent double-close
}

BladeRFTX& BladeRFTX::operator=(BladeRFTX&& other) noexcept {
    if (this != &other) {
        // Close our current device first (if any)
        if (dev_) {
            bladerf_enable_module(dev_, BLADERF_CHANNEL_TX(0), false);
            bladerf_close(dev_);
        }
        dev_    = other.dev_;
        config_ = other.config_;
        other.dev_ = nullptr;
    }
    return *this;
}


// ─────────────────────────────────────────────────────────────────────────────
// transmit() — send one buffer of SC16Q11 samples to the bladeRF
//
// bladerf_sync_tx() takes:
//   - A pointer to the sample data (int16_t pairs)
//   - Number of samples to transmit
//   - Metadata pointer (nullptr = no timestamps, just stream)
//   - Timeout in milliseconds
//
// It blocks until the samples are accepted into the DMA ring.
// The hardware transmits them from the ring at exactly 2.048 Msps.
// ─────────────────────────────────────────────────────────────────────────────
void BladeRFTX::transmit(const SC16Q11Buffer& buffer) {
    if (!dev_) {
        throw std::runtime_error("[bladeRF] transmit() called on closed device");
    }

    // SC16Q11Sample is {int16_t i, int16_t q} — same memory layout as
    // what libbladeRF expects. We can cast the pointer directly.
    // static_assert verifies the layout assumption at compile time.
    static_assert(sizeof(SC16Q11Sample) == 4,
        "SC16Q11Sample must be exactly 4 bytes (2×int16_t)");
    static_assert(offsetof(SC16Q11Sample, i) == 0,
        "I must be first in SC16Q11Sample");
    static_assert(offsetof(SC16Q11Sample, q) == 2,
        "Q must be second in SC16Q11Sample");

    const int status = bladerf_sync_tx(
        dev_,
        reinterpret_cast<const void*>(buffer.data()),
        static_cast<unsigned int>(buffer.size()),
        nullptr,              // no metadata — continuous stream
        config_.timeout_ms
    );
    check(status, "bladerf_sync_tx");
}


// ─────────────────────────────────────────────────────────────────────────────
// Utility functions
// ─────────────────────────────────────────────────────────────────────────────

std::string BladeRFTX::firmware_version() const {
    if (!dev_) return "device not open";
    struct bladerf_version ver{};
    bladerf_fw_version(dev_, &ver);
    return std::string(ver.describe);
}

std::string BladeRFTX::fpga_version() const {
    if (!dev_) return "device not open";
    struct bladerf_version ver{};
    bladerf_fpga_version(dev_, &ver);
    return std::string(ver.describe);
}

void BladeRFTX::set_gain(int gain_db) {
    config_.tx_gain_db = gain_db;
    const int status = bladerf_set_gain(dev_,
                                        BLADERF_CHANNEL_TX(0),
                                        gain_db);
    check(status, "bladerf_set_gain");
    std::cerr << "[bladeRF] TX gain updated: " << gain_db << " dB\n";
}

} // namespace gps