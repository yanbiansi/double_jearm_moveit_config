#pragma once

#include <Eigen/Geometry>

#include <vector>

struct CurvatureResult
{
  std::vector<double> s;      // 弧长（对应原始点）
  std::vector<double> kappa;  // 曲率（对应原始点）
};

std::vector<double> compute_arc_length(const std::vector<Eigen::Vector3d> &points);

CurvatureResult compute_discrete_curvature_3d(const std::vector<Eigen::Vector3d> &points, double s, bool do_plot);

struct SmoothnessFrequencyMetrics
{
  double hf_energy_n = 0.0;
  double hf_strong_energy_n = 0.0;
};

struct SmoothnessScoreResult
{
  std::vector<double> final_scores;
  std::vector<double> norm_C;
  std::vector<double> norm_A;
};

SmoothnessFrequencyMetrics compute_smoothness_frequency_metrics(
  const std::vector<double> &v,
  double fs,
  double fc_ratio,
  double amp_th,
  bool do_plot);

SmoothnessScoreResult calc_smoothness_score(
  const std::vector<double> &C,
  const std::vector<double> &A,
  double w_c);
