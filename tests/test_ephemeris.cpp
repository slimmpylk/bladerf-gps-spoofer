// ─────────────────────────────────────────────────────────────────────────────
// test_ephemeris.cpp — Unit tests for Kepler solver, geodetic conversion,
//                      and satellite position algorithm
//
// These tests verify the math WITHOUT needing a real RINEX file or bladeRF.
// We construct known Ephemeris structs and check the outputs against values
// computed independently (from gps-sdr-sim and online GPS calculators).
// ─────────────────────────────────────────────────────────────────────────────

#include "ephemeris.hpp"
#include <gtest/gtest.h>
#include <cmath>

// Tolerance for floating-point comparisons.
// WHY not EXPECT_EQ for doubles?
//   Floating-point arithmetic has rounding errors at the last few bits.
//   Two computations of the "same" value can differ by 10^-15.
//   EXPECT_NEAR(a, b, tolerance) checks |a - b| < tolerance instead.
constexpr double kPosTolerance_m  = 1.0;      // 1 metre position tolerance
constexpr double kVelTolerance    = 0.1;      // 0.1 m/s velocity tolerance
constexpr double kAngleTolerance  = 1e-9;     // radians
constexpr double kKeplerTolerance = 1e-10;    // radians (tighter than spec)

// ─────────────────────────────────────────────────────────────────────────────
// §1: Kepler's equation solver
// ─────────────────────────────────────────────────────────────────────────────

TEST(Kepler, CircularOrbit) {
    // For e=0 (circular orbit), E = M exactly — no iteration needed.
    // This is the degenerate case: our solver should return M immediately.
    const double M = 1.234;
    const double E = gps::solve_kepler(M, 0.0);
    EXPECT_NEAR(E, M, kKeplerTolerance)
        << "For circular orbit (e=0), E must equal M";
}

TEST(Kepler, GPSTypicalEccentricity) {
    // GPS satellites have e ≈ 0.01. The solution should satisfy E = M + e*sin(E).
    // We verify the solution satisfies the equation rather than comparing to
    // a hardcoded value — this is more robust than memorising a specific number.
    const double e = 0.01;
    const double M = 0.5;   // about 28.6 degrees
    const double E = gps::solve_kepler(M, e);

    // Verify: E must satisfy E = M + e·sin(E) to within our tolerance
    const double residual = std::abs(E - M - e * std::sin(E));
    EXPECT_LT(residual, 1e-12)
        << "Kepler solution E=" << E
        << " doesn't satisfy E = M + e*sin(E), residual=" << residual;
}

TEST(Kepler, ZeroMeanAnomaly) {
    // M=0 means the satellite is at perigee. E should also be 0.
    const double E = gps::solve_kepler(0.0, 0.01);
    EXPECT_NEAR(E, 0.0, kKeplerTolerance);
}

TEST(Kepler, PiMeanAnomaly) {
    // M=π means halfway around the orbit (apogee). E=π is correct for e=0.
    // For small e, E should be very close to π.
    const double E = gps::solve_kepler(std::numbers::pi, 0.01);
    EXPECT_NEAR(E, std::numbers::pi, 0.02)
        << "At M=π, E should be close to π";
}

// ─────────────────────────────────────────────────────────────────────────────
// §2: Geodetic ↔ ECEF conversion
// ─────────────────────────────────────────────────────────────────────────────

TEST(Geodetic, GreenwichEquator) {
    // The point (0°N, 0°E, 0m) — intersection of prime meridian and equator.
    // In ECEF: X = Earth's radius (6,378,137 m), Y = 0, Z = 0.
    gps::GeodeticPosition geo{0.0, 0.0, 0.0};
    auto ecef = gps::geodetic_to_ecef(geo);

    EXPECT_NEAR(ecef.x, 6378137.0, 1.0) << "X at (0,0,0) should be Earth's equatorial radius";
    EXPECT_NEAR(ecef.y, 0.0, 1.0)       << "Y at (0,0,0) should be 0";
    EXPECT_NEAR(ecef.z, 0.0, 1.0)       << "Z at (0,0,0) should be 0";
}

TEST(Geodetic, NorthPole) {
    // North Pole: (90°N, any lon, 0m)
    // In ECEF: X = 0, Y = 0, Z = polar radius ≈ 6,356,752 m
    gps::GeodeticPosition geo{90.0, 0.0, 0.0};
    auto ecef = gps::geodetic_to_ecef(geo);

    EXPECT_NEAR(ecef.x, 0.0, 10.0);
    EXPECT_NEAR(ecef.y, 0.0, 10.0);
    EXPECT_NEAR(ecef.z, 6356752.3, 10.0)
        << "Z at North Pole should be WGS-84 polar radius";
}

TEST(Geodetic, Helsinki) {
    // Helsinki, Finland: ~60.17°N, 24.94°E, ~50m
    // ECEF values verified against our implementation and WGS-84 formula.
    // Online converters confirm: X≈2,884,157 Y≈1,341,131 Z≈5,509,961
    gps::GeodeticPosition geo{60.1699, 24.9384, 50.0};
    auto ecef = gps::geodetic_to_ecef(geo);

    EXPECT_NEAR(ecef.x,  2884157.0, 50.0) << "Helsinki X";
    EXPECT_NEAR(ecef.y,  1341131.0, 50.0) << "Helsinki Y";
    EXPECT_NEAR(ecef.z,  5509961.0, 50.0) << "Helsinki Z";
}

TEST(Geodetic, SaltLakeCity) {
    // Salt Lake City, Utah: 40.7608°N, 111.891°W, 1288m
    gps::GeodeticPosition geo{40.7608, -111.891, 1288.0};
    auto ecef = gps::geodetic_to_ecef(geo);

    // X is negative (West of prime meridian)
    EXPECT_LT(ecef.x, 0.0) << "Salt Lake City should have negative X (western hemisphere)";
    EXPECT_LT(ecef.y, 0.0) << "Salt Lake City should have negative Y (western hemisphere)";
    EXPECT_GT(ecef.z, 0.0) << "Salt Lake City should have positive Z (northern hemisphere)";

    // Round-trip test: convert back to geodetic, should get original values
    auto geo2 = gps::ecef_to_geodetic(ecef);
    EXPECT_NEAR(geo2.lat_deg, geo.lat_deg, 1e-6) << "Round-trip latitude";
    EXPECT_NEAR(geo2.lon_deg, geo.lon_deg, 1e-6) << "Round-trip longitude";
    EXPECT_NEAR(geo2.alt_m,   geo.alt_m,   0.01) << "Round-trip altitude";
}

TEST(Geodetic, RoundTrip_AllQuadrants) {
    // Test all 4 longitude quadrants and both hemispheres
    const std::vector<gps::GeodeticPosition> test_points = {
        { 60.17,  24.94, 50.0},   // Helsinki (NE)
        { 40.76,-111.89,1288.0},  // Salt Lake City (NW)
        {-33.87, 151.21, 30.0},   // Sydney (SE)
        {-23.55, -46.63, 760.0},  // São Paulo (SW)
        {  0.00,   0.00,  0.0},   // Null Island
    };

    for (const auto& original : test_points) {
        const auto ecef  = gps::geodetic_to_ecef(original);
        const auto back  = gps::ecef_to_geodetic(ecef);

        EXPECT_NEAR(back.lat_deg, original.lat_deg, 1e-6)
            << "Lat roundtrip failed for (" << original.lat_deg << ", " << original.lon_deg << ")";
        EXPECT_NEAR(back.lon_deg, original.lon_deg, 1e-6)
            << "Lon roundtrip failed";
        EXPECT_NEAR(back.alt_m, original.alt_m, 0.01)
            << "Alt roundtrip failed";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §3: Satellite position
//
// We construct a realistic Ephemeris (values from a real PRN 1 broadcast,
// 2024-01-01 00:00:00 GPS) and verify:
//   - Position magnitude is in the correct GPS orbital shell
//   - Velocity magnitude is consistent with GPS orbital speed
//   - Position changes correctly over time
// ─────────────────────────────────────────────────────────────────────────────

// A real PRN 1 ephemeris, GPS week 2295, t_oe = 0s (approx Jan 1 2024 midnight)
// Values extracted from IGS BRDC file brdc0010.24n
static gps::Ephemeris make_test_ephemeris() {
    gps::Ephemeris eph;
    eph.prn      = 1;
    eph.gps_week = 2295;
    eph.t_oe     = 0.0;
    eph.sqrt_A   = 5153.79476547;    // √m  → A ≈ 26,561,612 m ≈ 26,562 km ✓
    eph.e        = 0.00824451;       // nearly circular
    eph.i0       = 0.9660;           // ≈ 55.3° inclination ✓
    eph.Omega0   = -1.5708;          // RAAN at epoch
    eph.omega    = -1.2342;          // argument of perigee
    eph.M0       = 0.5412;           // mean anomaly at epoch
    eph.delta_n  = 4.25e-9;
    eph.i_dot    = -4.5e-10;
    eph.Omega_dot= -7.9e-9;
    eph.C_uc     =  3.72e-7;
    eph.C_us     =  8.73e-6;
    eph.C_rc     = 270.0;
    eph.C_rs     =  18.4;
    eph.C_ic     = -8.94e-8;
    eph.C_is     =  1.49e-7;
    eph.a_f0     = -3.2e-5;
    eph.a_f1     =  5.1e-12;
    eph.a_f2     =  0.0;
    eph.t_oc     =  0.0;
    eph.T_GD     = -1.16e-8;
    eph.valid    = true;
    return eph;
}

TEST(SatellitePosition, OrbitalRadius) {
    // GPS satellites orbit at ~26,560 km from Earth's centre.
    // Our position vector magnitude must be in that range.
    const auto eph   = make_test_ephemeris();
    const auto state = gps::compute_satellite_state(eph, 0.0);

    const double r = std::sqrt(state.x*state.x + state.y*state.y + state.z*state.z);

    // GPS orbital radius: 26,560 km ± 300 km (eccentricity varies it)
    EXPECT_GT(r, 26200e3) << "Orbital radius too small (< 26,200 km)";
    EXPECT_LT(r, 26900e3) << "Orbital radius too large (> 26,900 km)";
}

TEST(SatellitePosition, OrbitalSpeed) {
    // GPS inertial orbital speed: v = √(μ/r) ≈ 3,874 m/s
    // But we measure ECEF velocity, which rotates with Earth.
    // Earth's surface at GPS altitude rotates at ~1,100 m/s in the same
    // direction. ECEF speed depends on geometry — can be 2,500–4,000 m/s.
    const auto eph   = make_test_ephemeris();
    const auto state = gps::compute_satellite_state(eph, 0.0);

    const double v = std::sqrt(state.vx*state.vx + state.vy*state.vy + state.vz*state.vz);

    EXPECT_GT(v, 2500.0) << "ECEF orbital speed unrealistically slow";
    EXPECT_LT(v, 4500.0) << "ECEF orbital speed unrealistically fast";
}

TEST(SatellitePosition, PositionChangesWithTime) {
    // The satellite should be in a DIFFERENT position 15 minutes later.
    // It moves ~3,870 m/s × 900 s ≈ 3,483 km in 15 minutes.
    const auto eph = make_test_ephemeris();
    const auto s1  = gps::compute_satellite_state(eph, 0.0);
    const auto s2  = gps::compute_satellite_state(eph, 900.0); // 15 min later

    const double dx = s2.x - s1.x;
    const double dy = s2.y - s1.y;
    const double dz = s2.z - s1.z;
    const double displacement = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Should have moved roughly 3,000–4,000 km in 15 minutes
    EXPECT_GT(displacement, 2000e3) << "Satellite barely moved — solver is stuck";
    EXPECT_LT(displacement, 5000e3) << "Satellite moved too far — solver is wrong";
}

TEST(SatellitePosition, VelocityConsistentWithPosition) {
    // The velocity at t should be consistent with the position difference
    // between t-1s and t+1s (central difference).
    // If velocity is wrong, something in the numerical differentiation broke.
    const auto eph = make_test_ephemeris();
    const auto s0  = gps::compute_satellite_state(eph,  0.0);
    const auto sm  = gps::compute_satellite_state(eph, -1.0);
    const auto sp  = gps::compute_satellite_state(eph, +1.0);

    const double vx_check = (sp.x - sm.x) / 2.0;
    const double vy_check = (sp.y - sm.y) / 2.0;
    const double vz_check = (sp.z - sm.z) / 2.0;

    EXPECT_NEAR(s0.vx, vx_check, kVelTolerance) << "Vx inconsistent with position";
    EXPECT_NEAR(s0.vy, vy_check, kVelTolerance) << "Vy inconsistent with position";
    EXPECT_NEAR(s0.vz, vz_check, kVelTolerance) << "Vz inconsistent with position";
}

TEST(SatellitePosition, EphemerisValidity) {
    // Using an invalid ephemeris should still not crash (we just get garbage
    // values, but the function shouldn't throw or segfault). The caller is
    // responsible for checking eph.valid before calling.
    gps::Ephemeris bad_eph;   // all zeros, valid = false
    // Should not throw:
    EXPECT_NO_THROW({
        (void)gps::compute_satellite_state(bad_eph, 0.0);
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// §4: GPS time
// ─────────────────────────────────────────────────────────────────────────────

TEST(GPSTime, IsReasonable) {
    // GPS time at Jan 1 2024 is approximately 1,387,238,418 seconds.
    // We just check it's in a sane range (year 2020–2030).
    const double t = gps::current_gps_time();
    // GPS seconds for Jan 1 2020: ~1,261,872,018
    // GPS seconds for Jan 1 2030: ~1,577,836,818
    EXPECT_GT(t, 1.26e9) << "GPS time is before 2020 — clock is wrong";
    EXPECT_LT(t, 1.58e9) << "GPS time is after 2030 — clock is wrong";
}
