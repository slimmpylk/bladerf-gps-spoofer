#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// ephemeris.hpp — RINEX Navigation File Parser + Satellite Position Solver
//
// WHAT THIS DOES:
//   1. Reads a RINEX 3 navigation file (.rnx) from IGS
//   2. Parses the Keplerian orbital elements for each satellite
//   3. Computes the satellite's ECEF (X,Y,Z) position at any GPS time
//   4. Computes the satellite's ECEF velocity (needed for Doppler)
//
// REFERENCE: IS-GPS-200N §20.3.3.4.3 — Table of algorithm steps
//            RINEX 3.05 format specification (IGS, 2021)
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <cmath>       // std::sin, std::cos, std::sqrt, std::atan2, std::abs
#include <cstdint>
#include <optional>    // C++17: std::optional<T> — a value that may or may not exist
#include <string>
#include <unordered_map>  // hash map: PRN → Ephemeris
#include <vector>

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// WGS-84 Physical Constants
//
// WHY constexpr double and not float?
//   Satellite positions must be accurate to centimetres from 20,000 km away.
//   float has ~7 decimal digits of precision. double has ~15.
//   At 26,560,000 m orbital radius, float precision is ±2,656 m — totally
//   wrong. double gives ±0.003 m — fine for our purpose.
//
// WHY not just use M_PI?
//   M_PI is a POSIX extension, not standard C++. It may not exist on all
//   compilers. std::numbers::pi (C++20) is the correct portable way.
//   We define our own here so the code is self-documenting.
// ─────────────────────────────────────────────────────────────────────────────

namespace wgs84 {
    // Earth's gravitational constant (μ = GM)
    // Value: IS-GPS-200N §20.3.3.4.3 Table 20-IV
    constexpr double kMu         = 3.986005e14;    // m³/s²

    // Earth's rotation rate (Ω̇ₑ)
    // Value: IS-GPS-200N §20.3.3.4.3 Table 20-IV
    constexpr double kOmegaDotE  = 7.2921151467e-5; // rad/s

    // Speed of light
    constexpr double kSpeedOfLight = 2.99792458e8;  // m/s

    // Semi-major axis of WGS-84 ellipsoid (used for geodetic conversions)
    constexpr double kA          = 6378137.0;        // m
    constexpr double kF          = 1.0 / 298.257223563; // flattening
    constexpr double kE2         = 2*kF - kF*kF;    // first eccentricity squared
} // namespace wgs84


// ─────────────────────────────────────────────────────────────────────────────
// Ephemeris — one satellite's complete orbital state from RINEX
//
// WHY a struct and not a class?
//   In C++, struct and class are nearly identical — the only difference is
//   default member access (struct = public, class = private).
//   We use struct for plain data containers with no hidden state.
//   We use class when we want to enforce invariants through private members.
//   Ephemeris is just a bag of numbers read from a file — struct is right.
//
// WHY double for everything?
//   Orbital mechanics. See WGS-84 constants discussion above.
// ─────────────────────────────────────────────────────────────────────────────

struct Ephemeris {
    // ── Identity ──────────────────────────────────────────────────────────
    int    prn       = 0;     // Satellite PRN number (1–32)
    int    gps_week  = 0;     // GPS week number

    // ── Reference time ────────────────────────────────────────────────────
    // t_oe: time of ephemeris (seconds into GPS week)
    // This is the epoch at which the Keplerian elements are defined.
    // The algorithm computes t_k = t - t_oe (time since epoch).
    double t_oe      = 0.0;

    // ── Keplerian orbital elements ─────────────────────────────────────────
    // These 6 numbers completely define the shape and orientation of the orbit.

    double sqrt_A    = 0.0;   // √(semi-major axis) in m^(1/2)
                               // WHY sqrt? RINEX stores √A, not A directly.
                               // We compute A = sqrt_A² in the solver.
    double e         = 0.0;   // Eccentricity (0 = circular, GPS ≈ 0.01)
    double i0        = 0.0;   // Inclination at t_oe (rad) — GPS ≈ 55°
    double Omega0    = 0.0;   // RAAN at weekly epoch (rad)
    double omega     = 0.0;   // Argument of perigee (rad)
    double M0        = 0.0;   // Mean anomaly at t_oe (rad)

    // ── Rate corrections ──────────────────────────────────────────────────
    // These account for slow changes in the orbit over time.
    double delta_n   = 0.0;   // Mean motion correction (rad/s)
    double i_dot     = 0.0;   // Rate of inclination change (rad/s)
    double Omega_dot = 0.0;   // Rate of RAAN change (rad/s)

    // ── Harmonic correction terms ─────────────────────────────────────────
    // Second-order corrections for gravitational perturbations
    // (Moon, Sun, Earth's equatorial bulge). Without these, position
    // errors would be hundreds of metres.
    // C_uc, C_us: argument of latitude corrections (rad)
    // C_rc, C_rs: orbit radius corrections (m)
    // C_ic, C_is: inclination corrections (rad)
    double C_uc = 0.0, C_us = 0.0;
    double C_rc = 0.0, C_rs = 0.0;
    double C_ic = 0.0, C_is = 0.0;

    // ── Clock corrections ─────────────────────────────────────────────────
    // The satellite's onboard atomic clock isn't perfect. These terms
    // correct for its drift: Δt_sv = a_f0 + a_f1(t-t_oc) + a_f2(t-t_oc)²
    double t_oc  = 0.0;    // clock reference time (s)
    double a_f0  = 0.0;    // clock bias (s)
    double a_f1  = 0.0;    // clock drift (s/s)
    double a_f2  = 0.0;    // clock drift rate (s/s²)
    double T_GD  = 0.0;    // group delay (s) — L1 only correction

    // ── Validity flag ─────────────────────────────────────────────────────
    // Set to true only after successful parsing. Prevents using
    // uninitialised ephemeris data.
    bool   valid = false;
};


// ─────────────────────────────────────────────────────────────────────────────
// SatelliteState — computed position and velocity at a specific GPS time
//
// This is the OUTPUT of the Kepler solver. It tells us:
//   - Where the satellite is (ECEF X, Y, Z in metres)
//   - How fast it's moving (ECEF Vx, Vy, Vz in m/s)
//   - What its clock error is (seconds)
//
// WHY a separate struct instead of returning a tuple?
//   std::tuple<double,double,double,double,double,double,double> is
//   impossible to read. Named struct members are self-documenting.
//   "state.x" is clear. "std::get<0>(result)" is not.
// ─────────────────────────────────────────────────────────────────────────────

struct SatelliteState {
    // Position in ECEF (Earth-Centered Earth-Fixed) coordinates
    // Origin at Earth's centre of mass, X toward Greenwich meridian,
    // Z toward North Pole, Y completing the right-hand system.
    double x = 0.0;    // metres
    double y = 0.0;    // metres
    double z = 0.0;    // metres

    // Velocity in ECEF (needed to compute Doppler shift)
    double vx = 0.0;   // m/s
    double vy = 0.0;   // m/s
    double vz = 0.0;   // m/s

    // Satellite clock error at this time (seconds)
    // Used to correct pseudorange: ρ_corrected = ρ - c·Δt_sv
    double clock_error = 0.0;
};


// ─────────────────────────────────────────────────────────────────────────────
// GeodeticPosition — latitude/longitude/altitude (what humans give us)
//
// The UI accepts lat/lon/alt. The Kepler solver works in ECEF.
// We need to convert between them.
// ─────────────────────────────────────────────────────────────────────────────

struct GeodeticPosition {
    double lat_deg = 0.0;   // latitude in degrees (positive = North)
    double lon_deg = 0.0;   // longitude in degrees (positive = East)
    double alt_m   = 0.0;   // altitude above WGS-84 ellipsoid (metres)
};

struct EcefPosition {
    double x = 0.0;   // metres
    double y = 0.0;   // metres
    double z = 0.0;   // metres
};

struct EcefVelocity {
    double vx = 0.0;  // m/s
    double vy = 0.0;  // m/s
    double vz = 0.0;  // m/s
};


// ─────────────────────────────────────────────────────────────────────────────
// Function declarations
// ─────────────────────────────────────────────────────────────────────────────

// Parse a RINEX 3 navigation file and return all valid ephemerides.
//
// WHY return std::unordered_map<int, std::vector<Ephemeris>>?
//   - The outer map key is PRN number. O(1) lookup: "give me all
//     ephemerides for PRN 5" is map[5].
//   - Each PRN can have multiple ephemerides (one per 2-hour validity
//     window). We keep all of them and pick the freshest one.
//   - WHY unordered_map and not map?
//     unordered_map uses a hash table: O(1) average lookup.
//     map uses a red-black tree: O(log n) lookup.
//     For PRN lookups (keys 1–32), unordered_map is faster.
//
// WHY [[nodiscard]]?
//   Parsing a file and throwing away the result is almost always a bug.
[[nodiscard]] std::unordered_map<int, std::vector<Ephemeris>>
parse_rinex_nav(const std::string& filename);

// Find the best (most recent valid) ephemeris for a PRN at a given GPS time.
//
// WHY std::optional<Ephemeris>?
//   The satellite might not have a valid ephemeris in the file.
//   We could return a bool + output parameter:
//       bool find_best_ephemeris(int prn, double gps_time, Ephemeris& out);
//   But std::optional is cleaner — it wraps "might not have a value" into
//   the type itself. The caller does:
//       if (auto eph = find_best_ephemeris(...)) { use *eph; }
//   instead of checking a bool separately.
[[nodiscard]] std::optional<Ephemeris>
find_best_ephemeris(
    const std::unordered_map<int, std::vector<Ephemeris>>& all_ephs,
    int prn,
    double gps_time_s   // seconds since GPS epoch (Jan 6, 1980)
);

// Compute satellite ECEF position and velocity from ephemeris at a given time.
// This is the IS-GPS-200N §20.3.3.4.3 algorithm.
//
// gps_time_s: GPS time in seconds since GPS epoch
// Returns SatelliteState with position, velocity, and clock error.
[[nodiscard]] SatelliteState
compute_satellite_state(const Ephemeris& eph, double gps_time_s);

// Solve Kepler's equation iteratively: E = M + e*sin(E)
// Returns eccentric anomaly E in radians.
//
// WHY expose this? Primarily for unit testing — we can verify the
// solver converges correctly for known inputs.
[[nodiscard]] double
solve_kepler(double M, double e, double tolerance = 1e-12, int max_iter = 50);

// Convert geodetic (lat/lon/alt) to ECEF (X/Y/Z)
// Uses WGS-84 ellipsoid. This is how we convert the user's target
// location into the coordinate system the Kepler solver uses.
[[nodiscard]] EcefPosition
geodetic_to_ecef(const GeodeticPosition& pos);

// Convert ECEF back to geodetic (for display/verification)
[[nodiscard]] GeodeticPosition
ecef_to_geodetic(const EcefPosition& pos);

// Compute current GPS time in seconds since GPS epoch (Jan 6, 1980 00:00:00 UTC)
// GPS time does NOT include leap seconds. As of 2024 it is 18 seconds ahead of UTC.
[[nodiscard]] double current_gps_time();

} // namespace gps
