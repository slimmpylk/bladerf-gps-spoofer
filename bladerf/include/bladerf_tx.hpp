#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// bladerf_tx.hpp — bladeRF 2.0 micro TX Interface
//
// WHAT THIS DOES:
//   Wraps the libbladeRF C API in a clean C++ RAII class.
//   Opens the bladeRF, configures it for GPS L1, and streams
//   SC16Q11 sample buffers to the TX1 antenna.
//
// RAII = Resource Acquisition Is Initialization.
//   WHY RAII? The bladeRF must be properly closed and the TX module
//   disabled even if an exception occurs. With RAII, the destructor
//   handles cleanup automatically — no need to remember to call close().
//   Constructor opens → destructor closes. Always. No leaks.
//
// HARDWARE: bladeRF 2.0 micro (AD9361 transceiver)
//   TX port:     TX1 (J3 SMA, labeled TX on the board)
//   Frequency:   1575.42 MHz (GPS L1)
//   Sample rate: 2.048 Msps
//   Format:      SC16Q11 — int16_t I then int16_t Q, interleaved
//
// REFERENCE: https://github.com/Nuand/bladeRF/wiki
//            libbladeRF API: https://nuand.com/libbladeRF-doc/
// ─────────────────────────────────────────────────────────────────────────────

// libbladeRF is a C library. We include it inside extern "C" to tell the
// C++ compiler not to mangle the function names — C and C++ use different
// name mangling schemes and mixing them without extern "C" causes linker errors.
extern "C" {
#include <libbladeRF.h>
}

#include "../../core/include/iq_stream.hpp"
#include <cstdint>
#include <stdexcept>
#include <string>

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// BladeRFConfig — all tunable parameters in one place
//
// WHY a config struct?
//   main.cpp can build one of these from command-line args and pass it
//   to BladeRFTX. Avoids a constructor with 8 positional parameters
//   which are impossible to read at the call site:
//     BladeRFTX tx(1575420000, 2048000, 2500000, 0, 16, 16, 8192, 5000);
//   vs:
//     BladeRFConfig cfg; cfg.tx_gain_db = 20;
//     BladeRFTX tx(cfg);
// ─────────────────────────────────────────────────────────────────────────────
struct BladeRFConfig {
    // RF parameters
    uint64_t frequency_hz  = 1575420000ULL;  // GPS L1 carrier
    uint32_t sample_rate   = 2048000;         // 2.048 Msps
    uint32_t bandwidth_hz  = 2500000;         // 2.5 MHz — covers C/A main lobe

    // TX gain in dB
    // bladeRF 2.0 micro TX gain range: -23.5 to +66.5 dB (overall)
    // Start at 0 dB and increase until the phone locks.
    // NEVER go above +20 dB indoors — unnecessary and risks interference.
    int tx_gain_db = 0;

    // Streaming buffer parameters
    // num_buffers:    number of DMA buffers in the ring
    // buffer_size:    samples per buffer (must be multiple of 1024)
    // num_transfers:  concurrent USB transfers in flight
    // timeout_ms:     TX timeout in milliseconds
    //
    // WHY 16 buffers of 8192 samples?
    //   8192 samples = 4ms of GPS signal at 2.048 Msps.
    //   16 buffers × 4ms = 64ms of pipeline depth.
    //   Enough to absorb OS scheduling jitter without underruns.
    unsigned int num_buffers   = 16;
    unsigned int buffer_size   = 8192;
    unsigned int num_transfers = 8;
    unsigned int timeout_ms    = 5000;
};


// ─────────────────────────────────────────────────────────────────────────────
// BladeRFTX — RAII wrapper around the bladeRF TX interface
// ─────────────────────────────────────────────────────────────────────────────
class BladeRFTX {
public:
    // Constructor: open device, configure, enable TX.
    // Throws std::runtime_error if any libbladeRF call fails.
    explicit BladeRFTX(const BladeRFConfig& config = BladeRFConfig{});

    // Destructor: disable TX, close device.
    // noexcept — destructors must never throw.
    ~BladeRFTX() noexcept;

    // Non-copyable — only one object should own the hardware handle.
    // WHY delete copy? If you copied BladeRFTX, both objects would try
    // to close the same device handle in their destructors — double-free.
    BladeRFTX(const BladeRFTX&)            = delete;
    BladeRFTX& operator=(const BladeRFTX&) = delete;

    // Movable — ownership can be transferred (e.g. into a vector).
    BladeRFTX(BladeRFTX&&) noexcept;
    BladeRFTX& operator=(BladeRFTX&&) noexcept;

    // Transmit one buffer of SC16Q11 samples.
    // Blocks until the samples are queued in the DMA ring.
    // Throws std::runtime_error on TX error or timeout.
    void transmit(const SC16Q11Buffer& buffer);

    // Get the bladeRF firmware version string (useful for logging)
    [[nodiscard]] std::string firmware_version() const;

    // Get the FPGA version string
    [[nodiscard]] std::string fpga_version() const;

    // Set TX gain on the fly (e.g. CLI adjustment while running)
    void set_gain(int gain_db);

    // True if the device is open and ready
    [[nodiscard]] bool is_open() const { return dev_ != nullptr; }

private:
    // Raw libbladeRF device handle.
    // WHY a raw pointer and not unique_ptr?
    //   bladerf_device* is a C opaque type. unique_ptr needs a custom
    //   deleter. It's cleaner to manage it manually in constructor/destructor
    //   since we already have RAII through the class itself.
    struct bladerf* dev_ = nullptr;

    BladeRFConfig config_;

    // Helper: check a libbladeRF return code.
    // WHY a member function instead of a free function?
    //   It can access config_ for error context.
    //   Throws std::runtime_error with the libbladeRF error string.
    static void check(int status, const std::string& context);
};

} // namespace gps