#include "dynamics.hpp"
#include <cmath>
#include <algorithm>

State dynamics_six_dof(const State& x, const Control& delta, const AircraftParams& p) {
    const double u = x[3], v = x[4], w = x[5];
    const double phi = x[6], theta = x[7], psi = x[8];
    const double pp = x[9], q = x[10], r = x[11];

    const double de = delta[0], da = delta[1], dr = delta[2], dt = delta[3];

    const double cphi = std::cos(phi), sphi = std::sin(phi);
    const double cth  = std::cos(theta), sth = std::sin(theta);
    const double cpsi = std::cos(psi), spsi = std::sin(psi);

    // R_b_v * [u;v;w]  (body -> NED)
    const double pn_dot = cth*cpsi*u + (sphi*sth*cpsi - cphi*spsi)*v + (cphi*sth*cpsi + sphi*spsi)*w;
    const double pe_dot = cth*spsi*u + (sphi*sth*spsi + cphi*cpsi)*v + (cphi*sth*spsi - sphi*cpsi)*w;
    const double pd_dot = -sth*u + sphi*cth*v + cphi*cth*w;

    // Euler angle rates from body rates (gimbal lock not guarded here,
    // matching the MATLAB reference's explicit error-on-lock contract --
    // callers must stay out of that regime, same as the MATLAB side).
    const double phi_dot   = pp + q*sphi*std::tan(theta) + r*cphi*std::tan(theta);
    const double theta_dot = q*cphi - r*sphi;
    const double psi_dot   = q*sphi/cth + r*cphi/cth;

    const double Va = std::sqrt(u*u + v*v + w*w);
    const double alpha = std::atan2(w, u);
    double beta = 0.0;
    if (Va > 1e-6) {
        double s = v / Va;
        s = std::max(-1.0, std::min(1.0, s));
        beta = std::asin(s);
    }
    const double qbar = 0.5 * p.rho * Va * Va;

    const double CL = p.CL0 + p.CL_alpha*alpha + p.CL_de*de;
    const double CD = p.CD0 + p.CD_alpha*alpha*alpha;
    const double CY = p.CY_beta*beta + p.CY_dr*dr;

    double p_hat = 0.0, q_hat = 0.0, r_hat = 0.0;
    if (Va > 1e-6) {
        p_hat = pp * p.b / (2.0*Va);
        q_hat = q  * p.c / (2.0*Va);
        r_hat = r  * p.b / (2.0*Va);
    }

    const double Cl_m = p.Cl_beta*beta + p.Cl_p*p_hat + p.Cl_r*r_hat + p.Cl_da*da;
    const double Cm_m = p.Cm0 + p.Cm_alpha*alpha + p.Cm_q*q_hat + p.Cm_de*de;
    const double Cn_m = p.Cn_beta*beta + p.Cn_p*p_hat + p.Cn_r*r_hat + p.Cn_da*da + p.Cn_dr*dr;

    const double L = qbar * p.S * CL;
    const double D = qbar * p.S * CD;
    const double Y = qbar * p.S * CY;

    const double ca = std::cos(alpha), sa = std::sin(alpha);
    const double fx_aero = -D*ca + L*sa;
    const double fz_aero = -D*sa - L*ca;
    const double fy_aero =  Y;

    const double fx_grav = -p.m * p.g * sth;
    const double fy_grav =  p.m * p.g * cth * sphi;
    const double fz_grav =  p.m * p.g * cth * cphi;

    const double throttle = std::max(0.0, std::min(1.0, dt));
    const double fx_thrust = p.T_max * throttle;

    const double fx = fx_aero + fx_grav + fx_thrust;
    const double fy = fy_aero + fy_grav;
    const double fz = fz_aero + fz_grav;

    const double l_m = qbar * p.S * p.b * Cl_m;
    const double m_m = qbar * p.S * p.c * Cm_m;
    const double n_m = qbar * p.S * p.b * Cn_m;

    const double u_dot = r*v - q*w + fx/p.m;
    const double v_dot = pp*w - r*u + fy/p.m;
    const double w_dot = q*u - pp*v + fz/p.m;

    const double p_dot = p.Gamma1*pp*q - p.Gamma2*q*r + p.Gamma3*l_m + p.Gamma4*n_m;
    const double q_dot = p.Gamma5*pp*r - p.Gamma6*(pp*pp - r*r) + m_m/p.Jy;
    const double r_dot = p.Gamma7*pp*q - p.Gamma1*q*r + p.Gamma4*l_m + p.Gamma8*n_m;

    State xdot;
    xdot[0] = pn_dot; xdot[1] = pe_dot; xdot[2] = pd_dot;
    xdot[3] = u_dot;  xdot[4] = v_dot;  xdot[5] = w_dot;
    xdot[6] = phi_dot; xdot[7] = theta_dot; xdot[8] = psi_dot;
    xdot[9] = p_dot;  xdot[10] = q_dot; xdot[11] = r_dot;
    return xdot;
}

static State add_scaled(const State& a, const State& b, double s) {
    State out;
    for (size_t i = 0; i < 12; ++i) out[i] = a[i] + s*b[i];
    return out;
}

State rk4_step(const State& x, const Control& delta, const AircraftParams& p, double dt) {
    State k1 = dynamics_six_dof(x, delta, p);
    State k2 = dynamics_six_dof(add_scaled(x, k1, dt/2.0), delta, p);
    State k3 = dynamics_six_dof(add_scaled(x, k2, dt/2.0), delta, p);
    State k4 = dynamics_six_dof(add_scaled(x, k3, dt), delta, p);

    State x_next;
    for (size_t i = 0; i < 12; ++i) {
        x_next[i] = x[i] + (dt/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    }
    return x_next;
}
