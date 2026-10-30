#include "params.hpp"
#include "csv_utils.hpp"
#include <stdexcept>

static double get_or_throw(const std::unordered_map<std::string, double>& m, const std::string& key) {
    auto it = m.find(key);
    if (it == m.end()) throw std::runtime_error("load_aircraft_params: missing field '" + key + "'");
    return it->second;
}

AircraftParams load_aircraft_params(const std::string& csv_path) {
    auto m = read_name_value_csv(csv_path);
    AircraftParams p{};
#define GET(field) p.field = get_or_throw(m, #field)
    GET(m); GET(g); GET(S); GET(b); GET(c);
    GET(Jx); GET(Jy); GET(Jz); GET(Jxz);
    GET(Gamma1); GET(Gamma2); GET(Gamma3); GET(Gamma4);
    GET(Gamma5); GET(Gamma6); GET(Gamma7); GET(Gamma8);
    GET(rho); GET(T_max);
    GET(CL0); GET(CL_alpha); GET(CL_de); GET(CD0); GET(CD_alpha);
    GET(Cm0); GET(Cm_alpha); GET(Cm_q); GET(Cm_de);
    GET(CY_beta); GET(CY_dr); GET(Cl_beta); GET(Cl_p); GET(Cl_r); GET(Cl_da);
    GET(Cn_beta); GET(Cn_p); GET(Cn_r); GET(Cn_da); GET(Cn_dr);
    GET(de_max); GET(da_max); GET(dr_max);
#undef GET
    return p;
}
