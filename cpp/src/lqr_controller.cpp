#include "lqr_controller.hpp"
#include "csv_utils.hpp"
#include <algorithm>

Control LqrController::operator()(const State& x) const {
    Control delta_cmd{};
    for (int row = 0; row < 4; ++row) {
        double acc = 0.0;
        for (int j = 0; j < 12; ++j) acc += K[row][j] * (x[j] - x_trim[j]);
        delta_cmd[row] = delta_trim[row] - acc;
    }

    Control delta;
    delta[0] = std::max(-params.de_max, std::min(params.de_max, delta_cmd[0]));
    delta[1] = std::max(-params.da_max, std::min(params.da_max, delta_cmd[1]));
    delta[2] = std::max(-params.dr_max, std::min(params.dr_max, delta_cmd[2]));
    delta[3] = std::max(0.0,            std::min(1.0,            delta_cmd[3]));
    return delta;
}

LqrController load_lqr_controller(const std::string& setup_csv, const AircraftParams& params) {
    LqrController c;
    c.params = params;

    auto x_trim_v = read_labeled_vector_csv(setup_csv, "x_trim");
    auto delta_trim_v = read_labeled_vector_csv(setup_csv, "delta_trim");
    for (int i = 0; i < 12; ++i) c.x_trim[i] = x_trim_v[i];
    for (int i = 0; i < 4; ++i) c.delta_trim[i] = delta_trim_v[i];

    for (int row = 0; row < 4; ++row) {
        auto row_v = read_labeled_vector_csv(setup_csv, "K_row" + std::to_string(row+1));
        for (int j = 0; j < 12; ++j) c.K[row][j] = row_v[j];
    }
    return c;
}
