#pragma once
#include <string>
#include <unordered_map>

// AircraftParams mirrors matlab/aircraft_params.m field-for-field. It is
// loaded from verification/aircraft_params.csv, which MATLAB exports
// directly from the SAME struct used for trim/linearization/LQR design
// (export_params_csv.m) -- so there is exactly one source of truth for
// every physical constant, never two hand-maintained copies that could
// silently drift apart between the MATLAB and C++ implementations.
struct AircraftParams {
    double m, g, S, b, c, Jx, Jy, Jz, Jxz;
    double Gamma1, Gamma2, Gamma3, Gamma4, Gamma5, Gamma6, Gamma7, Gamma8;
    double rho, T_max;
    double CL0, CL_alpha, CL_de, CD0, CD_alpha;
    double Cm0, Cm_alpha, Cm_q, Cm_de;
    double CY_beta, CY_dr, Cl_beta, Cl_p, Cl_r, Cl_da;
    double Cn_beta, Cn_p, Cn_r, Cn_da, Cn_dr;
    double de_max, da_max, dr_max;
};

// Parses a "name,value" CSV (lines starting with '#' are comments) into a
// map, then fills every field of AircraftParams by name -- throws
// std::runtime_error naming the missing key if the CSV is incomplete,
// rather than silently leaving a field default-initialized to zero.
AircraftParams load_aircraft_params(const std::string& csv_path);
