# bladerf-gps-spoofer

```
 ██████╗ ██████╗ ███████╗    ███████╗██████╗  ██████╗  ██████╗ ███████╗███████╗██████╗
██╔════╝ ██╔══██╗██╔════╝    ██╔════╝██╔══██╗██╔═══██╗██╔═══██╗██╔════╝██╔════╝██╔══██╗
██║  ███╗██████╔╝███████╗    ███████╗██████╔╝██║   ██║██║   ██║█████╗  █████╗  ██████╔╝
██║   ██║██╔═══╝ ╚════██║    ╚════██║██╔═══╝ ██║   ██║██║   ██║██╔══╝  ██╔══╝  ██╔══██╗
╚██████╔╝██║     ███████║    ███████║██║     ╚██████╔╝╚██████╔╝██║     ███████╗██║  ██║
 ╚═════╝ ╚═╝     ╚══════╝    ╚══════╝╚═╝      ╚═════╝  ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═╝
```

> A full-stack GPS L1 C/A signal generator for the bladeRF SDR platform — from raw Keplerian
> ephemeris to I/Q baseband samples, with a React coordinate-picker UI.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![Platform: Linux](https://img.shields.io/badge/Platform-Linux-lightgrey.svg)](https://www.kernel.org/)
[![Hardware: bladeRF](https://img.shields.io/badge/Hardware-bladeRF%202.0-green.svg)](https://www.nuand.com/bladerf-2-0-micro/)
[![Theory Docs](https://img.shields.io/badge/Theory-Math%20Reference-purple.svg)](https://slimmpylk.github.io/bladerf-gps-spoofer/)
[![Beginner Guide](https://img.shields.io/badge/Guide-Beginner%20Friendly-green.svg)](https://slimmpylk.github.io/bladerf-gps-spoofer/learn.html)
---

## ⚠️ Legal Notice

**This tool is for use exclusively inside RF-shielded enclosures or an authorised spectrum test facility.**

Transmitting on GPS L1 (1575.42 MHz) without authorisation is illegal in virtually every jurisdiction
(Finland/EU: Radio Equipment Directive 2014/53/EU, Traficom regulations; US: 47 CFR §333).
GPS spoofing signals propagate beyond your immediate environment and can affect aviation, maritime,
emergency services, and other safety-critical receivers.

**By using this software you confirm you are operating inside a properly attenuated RF environment
with appropriate regulatory authorisation.**

---

## Overview

`bladerf-gps-spoofer` generates GPS L1 C/A signals from first principles:

- Reads fresh RINEX 3 navigation files (ephemeris) from the IGS network
- Computes satellite ECEF positions via the full IS-GPS-200 Keplerian algorithm
- Synthesises C/A Gold codes (IS-GPS-200 Table 3-Ia) for all visible PRNs
- Encodes navigation message subframes 1–3 with correct Hamming parity
- Modulates to BPSK baseband, sums all channels, and streams SC16Q11 samples to bladeRF TX
- Exposes a REST API consumed by a React/TypeScript coordinate-picker UI

📖 **[Full mathematical theory & derivations →](https://slimmpylk.github.io/bladerf-gps-spoofer/)**

---

## Signal Chain

```
RINEX ephemeris (IGS)
        │
        ▼
┌───────────────────┐     ┌──────────────────┐
│  Satellite pos.   │     │  C/A code gen.   │
│  Keplerian→ECEF   │     │  G1⊕G2 Gold code │
└────────┬──────────┘     └────────┬─────────┘
         │                         │
         ▼                         ▼
┌────────────────────────────────────────────┐
│              Per-channel modulator         │
│   pseudorange  →  code phase offset        │
│   satellite vel → Doppler (±5 kHz)         │
│   D(t) · CA(t) · cos(2πf_D·t + φ)         │
└───────────────────────┬────────────────────┘
                        │  sum N channels
                        ▼
              ┌──────────────────┐
              │  I/Q baseband    │
              │  SC16Q11 @ Msps  │
              └────────┬─────────┘
                       │  libbladeRF stream
                       ▼
              bladeRF 2.0 micro
              TX @ 1575.42 MHz
```

---

## Repository Structure

```
bladerf-gps-spoofer/
├── core/                         # C++20 signal generation library
│   ├── include/
│   │   ├── ca_code.hpp           # C/A Gold code generator (G1, G2 LFSRs)
│   │   ├── ephemeris.hpp         # RINEX parser + Keplerian→ECEF solver
│   │   ├── nav_message.hpp       # Navigation message encoder (SF1–3)
│   │   ├── modulator.hpp         # Per-channel BPSK modulator
│   │   └── iq_stream.hpp         # Multi-channel I/Q combiner + scaler
│   └── src/
│       ├── ca_code.cpp
│       ├── ephemeris.cpp
│       ├── nav_message.cpp
│       ├── modulator.cpp
│       └── iq_stream.cpp
├── bladerf/                      # libbladeRF TX interface
│   ├── include/bladerf_tx.hpp
│   └── src/bladerf_tx.cpp
├── server/                       # Lightweight REST API (cpp-httplib)
│   ├── include/api_server.hpp
│   └── src/api_server.cpp
├── ui/                           # React/TypeScript frontend
│   ├── src/
│   │   ├── components/
│   │   │   ├── MapPicker.tsx     # Leaflet.js coordinate picker
│   │   │   ├── SkyPlot.tsx       # Real-time satellite sky plot
│   │   │   └── TxControl.tsx     # Start/stop, gain, sample rate
│   │   └── App.tsx
│   └── package.json
├── tools/
│   └── rinex_fetch.py            # Auto-download fresh ephemeris from IGS
├── docs/                         # GitHub Pages theory reference
│   ├── index.html                # Interactive math documentation
│   └── references.md             # Full citation list
├── tests/
│   ├── test_ca_code.cpp          # C/A code correctness vs known sequences
│   ├── test_ephemeris.cpp        # Keplerian solver vs reference positions
│   └── test_parity.cpp           # Navigation message parity check
├── CMakeLists.txt
└── README.md
```

---

## Dependencies

| Library | Purpose | Install |
|---|---|---|
| [libbladeRF](https://github.com/Nuand/bladeRF) | bladeRF hardware API | `apt install libbladerf-dev` |
| [cpp-httplib](https://github.com/yhirose/cpp-httplib) | Header-only REST server | vendored |
| [nlohmann/json](https://github.com/nlohmann/json) | JSON serialisation | `apt install nlohmann-json3-dev` |
| [GoogleTest](https://github.com/google/googletest) | Unit tests | CMake FetchContent |

**Frontend:**
```bash
cd ui && npm install
```

---

## Building

```bash
git clone https://github.com/yourusername/bladerf-gps-spoofer
cd bladerf-gps-spoofer
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

---

## Quick Start

### 1 — Download fresh ephemeris
```bash
python3 tools/rinex_fetch.py --output data/nav.rnx
```

### 2 — Start the backend
```bash
./build/gps-spoofer \
    --rinex data/nav.rnx \
    --lat 60.1699 --lon 24.9384 --alt 25.0 \
    --samplerate 2600000 \
    --port 8080
```

### 3 — Open the UI
```bash
cd ui && npm run dev
# → http://localhost:5173
```
Click anywhere on the map. The spoofer updates all satellite geometry in real-time.
The bladeRF begins transmitting immediately when you click **ARM**.

### 4 — Moving path mode (GPX)
```bash
./build/gps-spoofer --rinex data/nav.rnx --gpx tracks/helsinki_run.gpx --speed 3.5
```

---

## How It Works

See the **[interactive theory reference](https://slimmpylk.github.io/bladerf-gps-spoofer/)** for the full mathematical derivation of every component. The short version:

1. **C/A code generation** — Two 10-stage LFSRs (G1, G2) with polynomials from IS-GPS-200 Table 3-Ia produce a unique 1023-chip Gold code per PRN. The code repeats every 1 ms at 1.023 Mcps.

2. **Satellite position** — RINEX navigation files contain Keplerian orbital elements. Kepler's equation is solved iteratively (|ΔE| < 10⁻¹²) then rotated into ECEF using WGS-84 constants.

3. **Navigation message** — 50 bps BPSK, 1500-bit frames. Subframes 1–3 carry the satellite clock and full ephemeris. Hamming parity is applied per IS-GPS-200 §20.3.5.2.

4. **I/Q synthesis** — Each channel is modulated independently: `I = A·D·CA·cos(2πf_D·t)`, `Q = A·D·CA·sin(2πf_D·t)`. All N channels are summed and scaled to SC16Q11 format for bladeRF streaming.

---

## Verification

Before transmitting, verify your generated I/Q against a software receiver:

```bash
# Write ~10s of samples to file
./build/gps-spoofer --rinex data/nav.rnx --lat 60.1699 --lon 24.9384 \
    --output gpssim.bin --duration 10

# Decode with GNSS-SDR (in a separate environment)
gnss-sdr --config_file conf/gps_l1_ca_file.conf
```

A correct signal will lock on ≥4 satellites and report the spoofed position.

---

## References

A full annotated reference list with links is in [`docs/references.md`](docs/references.md).

Key sources:

- **IS-GPS-200** — Interface Specification: GPS SPS Signal Specification. [navipedia.net](https://www.navipedia.net/index.php/GPS_Signal_Plan)
- **Tsui (2000)** — *Fundamentals of Global Positioning System Receivers*, Wiley
- **Misra & Enge (2006)** — *Global Positioning System: Signals, Measurements and Performance*, 2nd ed.
- **gps-sdr-sim** — Takuji Ebinuma, [github.com/osqzss/gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim) — reference implementation this project builds upon
- **bladeRF wiki** — Nuand, [github.com/Nuand/bladeRF/wiki](https://github.com/Nuand/bladeRF/wiki)

---

## Contributing

Pull requests are welcome. For major changes, open an issue first. All signal-chain code must include unit tests that validate against known-good sequences from IS-GPS-200 appendices.

```
tests/test_ca_code.cpp   — PRN codes validated against IS-GPS-200 Table 3-Ia
tests/test_ephemeris.cpp — satellite positions validated against IGS precise orbits
tests/test_parity.cpp    — parity words validated against IS-GPS-200 §20.3.5.2
```

---

## License

MIT — see [LICENSE](LICENSE).

> This project is for educational and authorised research purposes only.
> The authors accept no liability for misuse.
