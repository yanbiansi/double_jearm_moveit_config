#include "smoothness_metrics_utils.h"

#include <Eigen/Geometry>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
std::vector<std::string> split_csv_simple(const std::string &line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    tokens.push_back(item);
  }
  return tokens;
}

std::vector<Eigen::Vector3d> read_points_csv(const std::string &path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("failed to open points csv: " + path);
  }

  std::vector<Eigen::Vector3d> points;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto tokens = split_csv_simple(line);
    if (tokens.size() < 3) {
      throw std::runtime_error("points csv must have at least 3 columns");
    }
    const double x = std::stod(tokens[0]);
    const double y = std::stod(tokens[1]);
    const double z = std::stod(tokens[2]);
    points.emplace_back(x, y, z);
  }
  return points;
}

void write_kappa_csv(const std::string &path, const std::vector<double> &kappa)
{
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    throw std::runtime_error("failed to open output csv: " + path);
  }
  for (size_t i = 0; i < kappa.size(); ++i) {
    ofs << kappa[i] << "\n";
  }
}

}  // namespace

int main(int argc, char **argv)
{
  std::string p_path = "p.csv";
  std::string out_csv = "kappa_cpp.csv";

  if (argc >= 2) {
    p_path = argv[1];
  }
  if (argc >= 3) {
    out_csv = argv[2];
  }

  try {
    const std::vector<Eigen::Vector3d> points = read_points_csv(p_path);

    const double curvature_s = 0.003;
    const bool curvature_plot = false;
    const double fs = 30.0;
    const double fc_ratio = 0.15;
    const double amp_th = 0.0;
    const bool smoothness_plot = false;

    const CurvatureResult curv = compute_discrete_curvature_3d(points, curvature_s, curvature_plot);
    const SmoothnessFrequencyMetrics sm = compute_smoothness_frequency_metrics(
      curv.kappa, fs, fc_ratio, amp_th, smoothness_plot);

    write_kappa_csv(out_csv, curv.kappa);
    std::cout << "hf_energy_n: " << sm.hf_energy_n << "\n";
    std::cout << "hf_strong_energy_n: " << sm.hf_strong_energy_n << "\n";
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
