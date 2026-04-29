#include "ephemeris.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace gps {

static constexpr double deg_to_rad(double deg) noexcept {
    return deg * std::numbers::pi / 180.0;
}

    double solve_kepler(double M, double e, double tolerance, int max_iter) {
    double E = M;

    for (int i = 0; i < max_iter; ++i) {
        double E_new = M + e * std::sin(E);
        double delta = std::abs(E_new - E);
        E = E_new;

        if (delta < tolerance) return E;
    }
    // ← loop finished without converging (only reaches here if max_iter exhausted)

    if (e > 0.001) {
        std::cerr << "[ephemeris] Warning: Kepler solver did not converge\n";
    }
    return E;
}

SatelliteState compute_satellite_state(const Ephemeris& eph, double gps_time_s) {
    const double A  = eph.sqrt_A * eph.sqrt_A;
    const double n0 = std::sqrt(wgs84::kMu / (A * A * A));
    const double n  = n0 + eph.delta_n;

    double t_k = gps_time_s - eph.t_oe;
    if (t_k >  302400.0) t_k -= 604800.0;
    if (t_k < -302400.0) t_k += 604800.0;

    const double M       = eph.M0 + n * t_k;
    const double E       = solve_kepler(M, eph.e);
    const double sin_E   = std::sin(E);
    const double cos_E   = std::cos(E);
    const double sqrt_1e2= std::sqrt(1.0 - eph.e * eph.e);
    const double nu      = std::atan2(sqrt_1e2 * sin_E, cos_E - eph.e);
    const double Phi     = nu + eph.omega;

    const double sin2Phi = std::sin(2.0 * Phi);
    const double cos2Phi = std::cos(2.0 * Phi);
    const double delta_u = eph.C_us * sin2Phi + eph.C_uc * cos2Phi;
    const double delta_r = eph.C_rs * sin2Phi + eph.C_rc * cos2Phi;
    const double delta_i = eph.C_is * sin2Phi + eph.C_ic * cos2Phi;

    const double u     = Phi + delta_u;
    const double r     = A * (1.0 - eph.e * cos_E) + delta_r;
    const double i     = eph.i0 + delta_i + eph.i_dot * t_k;
    const double x_orb = r * std::cos(u);
    const double y_orb = r * std::sin(u);

    const double Omega = eph.Omega0
        + (eph.Omega_dot - wgs84::kOmegaDotE) * t_k
        - wgs84::kOmegaDotE * eph.t_oe;

    const double cos_Omega = std::cos(Omega);
    const double sin_Omega = std::sin(Omega);
    const double cos_i     = std::cos(i);

    SatelliteState state;
    state.x = x_orb * cos_Omega - y_orb * cos_i * sin_Omega;
    state.y = x_orb * sin_Omega + y_orb * cos_i * cos_Omega;
    state.z = y_orb * std::sin(i);

    // Numerical velocity via central difference (avoids recursive velocity calls)
    // We compute position only (no velocity) for the helper calls
    auto pos_only = [&](double t) -> std::array<double,3> {
        double tk2 = t - eph.t_oe;
        if (tk2 >  302400.0) tk2 -= 604800.0;
        if (tk2 < -302400.0) tk2 += 604800.0;
        const double M2   = eph.M0 + n * tk2;
        const double E2   = solve_kepler(M2, eph.e);
        const double nu2  = std::atan2(sqrt_1e2*std::sin(E2), std::cos(E2)-eph.e);
        const double Ph2  = nu2 + eph.omega;
        const double s2   = std::sin(2*Ph2), c2 = std::cos(2*Ph2);
        const double u2   = Ph2 + eph.C_us*s2 + eph.C_uc*c2;
        const double r2   = A*(1-eph.e*std::cos(E2)) + eph.C_rs*s2 + eph.C_rc*c2;
        const double i2   = eph.i0 + eph.C_is*s2 + eph.C_ic*c2 + eph.i_dot*tk2;
        const double xo   = r2*std::cos(u2), yo = r2*std::sin(u2);
        const double Om2  = eph.Omega0 + (eph.Omega_dot-wgs84::kOmegaDotE)*tk2
                            - wgs84::kOmegaDotE*eph.t_oe;
        const double cO   = std::cos(Om2), sO = std::sin(Om2), ci = std::cos(i2);
        return { xo*cO - yo*ci*sO, xo*sO + yo*ci*cO, yo*std::sin(i2) };
    };

    const double dt = 0.5;
    auto p_fwd = pos_only(gps_time_s + dt);
    auto p_bwd = pos_only(gps_time_s - dt);
    state.vx = (p_fwd[0] - p_bwd[0]) / (2.0 * dt);
    state.vy = (p_fwd[1] - p_bwd[1]) / (2.0 * dt);
    state.vz = (p_fwd[2] - p_bwd[2]) / (2.0 * dt);

    const double dt_sv = gps_time_s - eph.t_oc;
    state.clock_error  = eph.a_f0 + eph.a_f1*dt_sv + eph.a_f2*dt_sv*dt_sv - eph.T_GD;

    return state;
}

EcefPosition geodetic_to_ecef(const GeodeticPosition& pos) {
    const double lat     = deg_to_rad(pos.lat_deg);
    const double lon     = deg_to_rad(pos.lon_deg);
    const double h       = pos.alt_m;
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double N       = wgs84::kA / std::sqrt(1.0 - wgs84::kE2*sin_lat*sin_lat);
    return { (N+h)*cos_lat*std::cos(lon),
             (N+h)*cos_lat*std::sin(lon),
             (N*(1.0-wgs84::kE2)+h)*sin_lat };
}

GeodeticPosition ecef_to_geodetic(const EcefPosition& pos) {
    const double p   = std::sqrt(pos.x*pos.x + pos.y*pos.y);
    const double lon = std::atan2(pos.y, pos.x);
    double lat = std::atan2(pos.z, p*(1.0-wgs84::kE2));
    for (int i = 0; i < 5; ++i) {
        const double sl = std::sin(lat);
        const double N  = wgs84::kA / std::sqrt(1.0-wgs84::kE2*sl*sl);
        lat = std::atan2(pos.z + wgs84::kE2*N*sl, p);
    }
    const double sl  = std::sin(lat);
    const double N   = wgs84::kA / std::sqrt(1.0-wgs84::kE2*sl*sl);
    const double h   = p/std::cos(lat) - N;
    return { lat*180.0/std::numbers::pi, lon*180.0/std::numbers::pi, h };
}

double current_gps_time() {
    constexpr double kGpsEpochOffset = 315964800.0;
    constexpr double kLeapSeconds    = 18.0;
    const auto now      = std::chrono::system_clock::now();
    const double unix_t = std::chrono::duration<double>(now.time_since_epoch()).count();
    return unix_t - kGpsEpochOffset + kLeapSeconds;
}

static double parse_rinex_double(const std::string& s) {
    std::string f = s;
    for (char& c : f) if (c=='D'||c=='d') c='E';
    try { return std::stod(f); } catch (...) { return 0.0; }
}

static std::array<double,4> parse_orbit_line(const std::string& line) {
    std::array<double,4> v{};
    constexpr int off[] = {4,23,42,61};
    for (int i=0;i<4;++i)
        if ((int)line.size()>=off[i]+19)
            v[i] = parse_rinex_double(line.substr(off[i],19));
    return v;
}

std::unordered_map<int, std::vector<Ephemeris>>
parse_rinex_nav(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open())
        throw std::runtime_error("Cannot open RINEX file: " + filename);

    std::unordered_map<int, std::vector<Ephemeris>> result;
    std::string line;
    bool in_header = true;

    while (std::getline(file, line)) {
        if (line.find("END OF HEADER") != std::string::npos) { in_header=false; break; }
    }
    if (in_header) throw std::runtime_error("RINEX missing END OF HEADER: "+filename);

    while (std::getline(file, line)) {
        if (line.empty() || line[0]!='G') continue;
        Ephemeris eph;
        try { eph.prn = std::stoi(line.substr(1,2)); } catch(...) { continue; }
        if ((int)line.size()>=80) {
            eph.a_f0 = parse_rinex_double(line.substr(23,19));
            eph.a_f1 = parse_rinex_double(line.substr(42,19));
            eph.a_f2 = parse_rinex_double(line.substr(61,19));
        }
        try {
            eph.t_oc = std::stoi(line.substr(15,2))*3600.0
                     + std::stoi(line.substr(18,2))*60.0
                     + std::stoi(line.substr(21,2));
        } catch(...) {}

        std::array<std::array<double,4>,7> orbits{};
        for (int l=0;l<7;++l) {
            if (!std::getline(file,line)) break;
            orbits[l] = parse_orbit_line(line);
        }
        eph.C_rs=orbits[0][1]; eph.delta_n=orbits[0][2]; eph.M0=orbits[0][3];
        eph.C_uc=orbits[1][0]; eph.e=orbits[1][1]; eph.C_us=orbits[1][2]; eph.sqrt_A=orbits[1][3];
        eph.t_oe=orbits[2][0]; eph.C_ic=orbits[2][1]; eph.Omega0=orbits[2][2]; eph.C_is=orbits[2][3];
        eph.i0=orbits[3][0];   eph.C_rc=orbits[3][1]; eph.omega=orbits[3][2]; eph.Omega_dot=orbits[3][3];
        eph.i_dot=orbits[4][0]; eph.gps_week=(int)orbits[4][2];
        eph.T_GD=orbits[5][2];
        eph.valid=true;
        result[eph.prn].push_back(eph);
    }
    std::cerr << "[ephemeris] Parsed " << result.size() << " PRNs from " << filename << "\n";
    return result;
}

std::optional<Ephemeris>
find_best_ephemeris(
    const std::unordered_map<int,std::vector<Ephemeris>>& all_ephs,
    int prn, double gps_time_s)
{
    const auto it = all_ephs.find(prn);
    if (it==all_ephs.end() || it->second.empty()) return std::nullopt;
    const auto& ephs = it->second;
    const auto best = std::min_element(ephs.begin(), ephs.end(),
        [&](const Ephemeris& a, const Ephemeris& b){
            return std::abs(gps_time_s-a.t_oe) < std::abs(gps_time_s-b.t_oe);
        });
    return *best;
}

} // namespace gps
