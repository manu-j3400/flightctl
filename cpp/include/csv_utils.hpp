#pragma once
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

// Minimal CSV helpers shared by the whole C++ side. Deliberately
// dependency-free (no external CSV/JSON library) -- the file formats
// written by the MATLAB export scripts are simple enough that a few
// dozen lines of istringstream code is the honest amount of machinery,
// not an excuse to add a library dependency.

inline std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) out.push_back(field);
    return out;
}

// Reads a "name,value" CSV (one scalar per line, '#'-prefixed comment
// lines skipped) into a name->value map.
inline std::unordered_map<std::string, double> read_name_value_csv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("read_name_value_csv: could not open " + path);
    }
    std::unordered_map<std::string, double> out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto fields = split_csv_line(line);
        if (fields.size() < 2) continue;
        out[fields[0]] = std::stod(fields[1]);
    }
    return out;
}

// Reads a vector given as a comma-separated list on one line, e.g.
// "x_trim,1.0,2.0,...". Looks up by the row label in column 0.
inline std::vector<double> read_labeled_vector_csv(const std::string& path, const std::string& label) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("read_labeled_vector_csv: could not open " + path);
    }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto fields = split_csv_line(line);
        if (fields.empty()) continue;
        if (fields[0] == label) {
            std::vector<double> v;
            for (size_t i = 1; i < fields.size(); ++i) v.push_back(std::stod(fields[i]));
            return v;
        }
    }
    throw std::runtime_error("read_labeled_vector_csv: label '" + label + "' not found in " + path);
}

inline double read_labeled_scalar_csv(const std::string& path, const std::string& label) {
    auto v = read_labeled_vector_csv(path, label);
    if (v.empty()) throw std::runtime_error("read_labeled_scalar_csv: empty vector for '" + label + "'");
    return v[0];
}
