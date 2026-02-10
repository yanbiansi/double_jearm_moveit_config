#include "smoothness_metrics_utils.h"

#include <algorithm>
#include <complex>
#include <cmath>
#include <limits>
#include <stdexcept>

std::vector<double> compute_arc_length(const std::vector<Eigen::Vector3d> &points)
{
  const size_t n = points.size();
  std::vector<double> s(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    s[i] = s[i - 1] + (points[i] - points[i - 1]).norm();
  }
  return s;
}

CurvatureResult compute_discrete_curvature_3d(const std::vector<Eigen::Vector3d> &points, double s, bool do_plot)
{
  (void)do_plot;
  if (points.size() < 3) {
    throw std::runtime_error("need at least 3 points");
  }
  if (s < 0.0) {
    throw std::runtime_error("s must be non-negative");
  }

  constexpr int kStep = 1; // 步长，写死为1，相邻点
  const double eps = std::numeric_limits<double>::epsilon();  // 机器精度，用来避免除 0 或接近 0 的问题。

  const std::vector<double> s_orig = compute_arc_length(points);

  std::vector<size_t> idx_keep; // 用来记录 被保留点 在原始点列中的索引。
  std::vector<Eigen::Vector3d> p_res; // 存储 重采样后的点列。

  if (s == 0.0) {
    idx_keep.resize(points.size());   // 让 idx_keep 的长度和原始点数一致。
    for (size_t i = 0; i < points.size(); ++i) {
      idx_keep[i] = i;  // 把所有索引都填进去，表示“每个点都被保留”。
    }
    p_res = points;
  } else {
    // 最小间距重采样
    idx_keep.push_back(0);
    size_t last_kept = 0;
    for (size_t i = 1; i + 1 < points.size(); ++i) {
      if ((points[i] - points[last_kept]).norm() >= s) {
        idx_keep.push_back(i);
        last_kept = i;
      }
    }
    // 确保最后一个点一定保留
    if (idx_keep.back() != points.size() - 1) {
      idx_keep.push_back(points.size() - 1);
    }
    // 根据 idx_keep 把重采样点填到 p_res
    p_res.reserve(idx_keep.size());
    for (const size_t idx : idx_keep) {
      p_res.push_back(points[idx]);
    }
    // 曲率至少需要 3 个点，如果重采样后不足 3 个就报错
    if (p_res.size() < 3) {
      throw std::runtime_error("resampled points < 3");
    }
  }
  // 不同步长下需要的点数检查
  const size_t n_res = p_res.size();
  if (n_res < static_cast<size_t>(2 * kStep + 1)) {
    throw std::runtime_error("not enough points for curvature (need >= 3)");
  }
  // 减去头尾步长
  const size_t m_pts = n_res - 2 * kStep;
  // 存放每个可计算点的曲率值
  std::vector<double> kappa_res(m_pts, 0.0);
  // 存放这些曲率点在原始弧长坐标系里的位置（用于后续插值回原始点）
  std::vector<double> s_mid_orig(m_pts, 0.0);

  for (size_t j = kStep; j + kStep < n_res; ++j) {
    const size_t idx = j - kStep;
    const Eigen::Vector3d v1 = p_res[j] - p_res[j - kStep];
    const Eigen::Vector3d v2 = p_res[j + kStep] - p_res[j];
    const double len1 = v1.norm();
    const double len2 = v2.norm();

    const size_t orig_idx_j = idx_keep[j];
    s_mid_orig[idx] = s_orig[orig_idx_j];

    const double ds = 0.5 * (len1 + len2);
    if (len1 < eps || len2 < eps) {
      kappa_res[idx] = 0.0;
      continue;
    }

    const Eigen::Vector3d t1 = v1 / len1;
    const Eigen::Vector3d t2 = v2 / len2;
    const Eigen::Vector3d c = t1.cross(t2);
    const double theta = std::atan2(c.norm(), t1.dot(t2));
    kappa_res[idx] = theta / std::max(ds, eps);
  }

  for (double &k : kappa_res) {
    if (!std::isfinite(k)) {
      k = 0.0;
    }
  }

  std::vector<double> s_known;
  std::vector<double> k_known;
  s_known.reserve(m_pts + 2);
  k_known.reserve(m_pts + 2);
  s_known.push_back(0.0);
  k_known.push_back(0.0);
  for (size_t i = 0; i < m_pts; ++i) {
    s_known.push_back(s_mid_orig[i]);
    k_known.push_back(kappa_res[i]);
  }
  s_known.push_back(s_orig.back());
  k_known.push_back(0.0);

  std::vector<double> kappa_full(points.size(), 0.0);
  size_t seg = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const double x = s_orig[i];
    while (seg + 1 < s_known.size() && x > s_known[seg + 1]) {
      ++seg;
    }
    if (seg + 1 >= s_known.size()) {
      kappa_full[i] = k_known.back();
    } else {
      const double x0 = s_known[seg];
      const double x1 = s_known[seg + 1];
      const double y0 = k_known[seg];
      const double y1 = k_known[seg + 1];
      const double t = (x1 > x0) ? (x - x0) / (x1 - x0) : 0.0;
      kappa_full[i] = y0 + t * (y1 - y0);
    }
  }

  for (size_t j = kStep; j + kStep < n_res; ++j) {
    const size_t orig_idx = idx_keep[j];
    const size_t idx_kappa = j - kStep;
    kappa_full[orig_idx] = kappa_res[idx_kappa];
  }

  return {s_orig, kappa_full};
}

SmoothnessFrequencyMetrics compute_smoothness_frequency_metrics(
  const std::vector<double> &v,
  double fs,
  double fc_ratio,
  double amp_th,
  bool do_plot)
{
  (void)do_plot;
  SmoothnessFrequencyMetrics out;

  if (v.empty()) {
    return out;
  }
  if (fs <= 0.0) {
    throw std::runtime_error("fs must be positive");
  }
  if (fc_ratio <= 0.0) {
    fc_ratio = 0.25;
  }
  if (amp_th < 0.0) {
    amp_th = 0.0;
  }

  const size_t N = v.size();
  double mean = 0.0;
  for (double x : v) {
    mean += x;
  }
  mean /= static_cast<double>(N);

  std::vector<double> v0(N, 0.0);
  double energy_time = 0.0;
  for (size_t i = 0; i < N; ++i) {
    v0[i] = v[i] - mean;
    energy_time += v0[i] * v0[i];
  }

  if (energy_time < std::numeric_limits<double>::epsilon()) {
    return out;
  }

  std::vector<std::complex<double>> V(N);
  const double two_pi_over_n = 2.0 * M_PI / static_cast<double>(N);
  for (size_t k = 0; k < N; ++k) {
    std::complex<double> sum(0.0, 0.0);
    for (size_t n = 0; n < N; ++n) {
      const double angle = -two_pi_over_n * static_cast<double>(k * n);
      sum += std::complex<double>(std::cos(angle), std::sin(angle)) * v0[n];
    }
    V[k] = sum;
  }

  std::vector<double> P2(N, 0.0);
  for (size_t k = 0; k < N; ++k) {
    P2[k] = std::norm(V[k]) / static_cast<double>(N);
  }

  const size_t half_n = N / 2;
  const size_t P_size = half_n + 1;
  std::vector<double> P(P_size, 0.0);
  std::copy(P2.begin(), P2.begin() + static_cast<long>(P_size), P.begin());
  if (P.size() > 2) {
    for (size_t i = 1; i + 1 < P.size(); ++i) {
      P[i] *= 2.0;
    }
  }

  std::vector<double> amp(P.size(), 0.0);
  for (size_t i = 0; i < P.size(); ++i) {
    amp[i] = std::sqrt(P[i]);
  }

  const double fc = fc_ratio * (fs / 2.0);

  double hf_energy = 0.0;
  double hf_strong_energy = 0.0;

  for (size_t i = 0; i < P.size(); ++i) {
    const double f = static_cast<double>(i) * (fs / static_cast<double>(N));
    const bool is_hf = f > fc;
    if (is_hf) {
      hf_energy += P[i];
      const bool strong = (amp_th <= 0.0) ? true : (amp[i] > amp_th);
      if (strong) {
        hf_strong_energy += P[i];
      }
    }
  }

  out.hf_energy_n = hf_energy / static_cast<double>(N);
  out.hf_strong_energy_n = hf_strong_energy / static_cast<double>(N);

  return out;
}

SmoothnessScoreResult calc_smoothness_score(
  const std::vector<double> &C,
  const std::vector<double> &A,
  double w_c)
{
  SmoothnessScoreResult out;
  if (C.size() != A.size()) {
    throw std::runtime_error("C and A must have the same length");
  }
  const size_t n = C.size();
  out.final_scores.resize(n, 0.0);
  out.norm_C.resize(n, 0.0);
  out.norm_A.resize(n, 0.0);
  if (n == 0) {
    return out;
  }

  double max_C = *std::max_element(C.begin(), C.end());
  double max_A = *std::max_element(A.begin(), A.end());
  if (max_C == 0.0) {
    max_C = 1.0;
  }
  if (max_A == 0.0) {
    max_A = 1.0;
  }

  const double w_a = 1.0 - w_c;
  for (size_t i = 0; i < n; ++i) {
    out.norm_C[i] = C[i] / max_C;
    out.norm_A[i] = A[i] / max_A;
    out.final_scores[i] = (w_c * out.norm_C[i]) + (w_a * out.norm_A[i]);
  }

  return out;
}
