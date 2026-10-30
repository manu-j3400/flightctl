#include "nn_controller.hpp"
#include "csv_utils.hpp"
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <fstream>

static int count_params(const std::array<int,4>& sizes) {
    int n = 0;
    for (size_t i = 0; i + 1 < sizes.size(); ++i) n += sizes[i]*sizes[i+1] + sizes[i+1];
    return n;
}

Control NnController::operator()(const State& x) const {
    static constexpr int idx_reduced[8] = {3, 4, 5, 6, 7, 9, 10, 11}; // 0-indexed equivalent of MATLAB's [4 5 6 7 8 10 11 12]

    std::array<double, 8> e{};
    for (int i = 0; i < 8; ++i) {
        e[i] = (x[idx_reduced[i]] - x_trim[idx_reduced[i]]) / scale[i];
    }

    const auto& sz = kLayerSizes;
    size_t idx = 0;

    // NOTE on indexing: MATLAB's nn_policy.m builds each weight matrix with
    // Ws{i} = reshape(theta(a:b), nout, nin) -- Octave/MATLAB reshape fills
    // COLUMN-MAJOR, so element (o, i) (0-indexed: output row o, input col i)
    // sits at flattened offset  i*nout + o , NOT o*nin + i (which would be
    // the row-major layout a C++ programmer reaches for by habit). Getting
    // this backwards is exactly the kind of silent bug the MATLAB/C++
    // parity check in verification/compare_trajectories.m exists to catch
    // -- it did, on the first attempt here (case2 failed with huge errors
    // until this offset was corrected to match MATLAB's storage order).

    // Layer 1: 8 -> 24, tanh
    std::array<double, 24> h1{};
    for (int o = 0; o < sz[1]; ++o) {
        double acc = 0.0;
        for (int i = 0; i < sz[0]; ++i) acc += theta[idx + i*sz[1] + o] * e[i];
        h1[o] = acc;
    }
    idx += sz[0]*sz[1];
    for (int o = 0; o < sz[1]; ++o) { h1[o] += theta[idx + o]; h1[o] = std::tanh(h1[o]); }
    idx += sz[1];

    // Layer 2: 24 -> 24, tanh
    std::array<double, 24> h2{};
    for (int o = 0; o < sz[2]; ++o) {
        double acc = 0.0;
        for (int i = 0; i < sz[1]; ++i) acc += theta[idx + i*sz[2] + o] * h1[i];
        h2[o] = acc;
    }
    idx += sz[1]*sz[2];
    for (int o = 0; o < sz[2]; ++o) { h2[o] += theta[idx + o]; h2[o] = std::tanh(h2[o]); }
    idx += sz[2];

    // Layer 3 (output): 24 -> 4, linear (saturation mapping applied below)
    std::array<double, 4> z{};
    for (int o = 0; o < sz[3]; ++o) {
        double acc = 0.0;
        for (int i = 0; i < sz[2]; ++i) acc += theta[idx + i*sz[3] + o] * h2[i];
        z[o] = acc;
    }
    idx += sz[2]*sz[3];
    for (int o = 0; o < sz[3]; ++o) z[o] += theta[idx + o];
    idx += sz[3];

    Control delta;
    delta[0] = params.de_max * std::tanh(z[0]);
    delta[1] = params.da_max * std::tanh(z[1]);
    delta[2] = params.dr_max * std::tanh(z[2]);
    delta[3] = 0.5 * (1.0 + std::tanh(z[3]));
    return delta;
}

NnController load_nn_controller(const std::string& weights_csv, const State& x_trim, const AircraftParams& params) {
    NnController c;
    c.x_trim = x_trim;
    c.params = params;

    std::ifstream f(weights_csv);
    if (!f.is_open()) throw std::runtime_error("load_nn_controller: could not open " + weights_csv);

    std::string line;
    std::vector<int> layer_sizes;
    bool got_layers = false, got_scale = false, got_theta = false;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] != '#') continue;
        auto fields = split_csv_line(line.substr(2)); // drop "# "
        if (fields.empty()) continue;
        if (fields[0] == "layer_sizes") {
            for (size_t i = 1; i < fields.size(); ++i) layer_sizes.push_back(std::stoi(fields[i]));
            got_layers = true;
        } else if (fields[0] == "scale") {
            for (size_t i = 1; i < fields.size() && i <= 8; ++i) c.scale[i-1] = std::stod(fields[i]);
            got_scale = true;
        } else if (fields[0] == "theta") {
            for (size_t i = 1; i < fields.size(); ++i) c.theta.push_back(std::stod(fields[i]));
            got_theta = true;
        }
    }
    if (!got_layers || !got_scale || !got_theta) {
        throw std::runtime_error("load_nn_controller: " + weights_csv + " missing layer_sizes/scale/theta header lines");
    }
    for (size_t i = 0; i < layer_sizes.size(); ++i) {
        if (layer_sizes[i] != NnController::kLayerSizes[i]) {
            throw std::runtime_error("load_nn_controller: layer size mismatch at index " + std::to_string(i) +
                " (file says " + std::to_string(layer_sizes[i]) + ", compiled-in architecture says " +
                std::to_string(NnController::kLayerSizes[i]) + ")");
        }
    }
    int expected = count_params(NnController::kLayerSizes);
    if ((int)c.theta.size() != expected) {
        throw std::runtime_error("load_nn_controller: expected " + std::to_string(expected) +
            " params, file has " + std::to_string(c.theta.size()));
    }
    return c;
}
