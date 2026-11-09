#include "arm_hardware/static_tendon_compensation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace arm_hardware
{
namespace
{

constexpr auto kActiveCount = kTendonAxes.size();
using Active = std::array<double, kActiveCount>;
using ActiveMatrix = std::array<Active, kActiveCount>;

TendonVector vector(const double * values)
{
  return {values[0], values[1], values[2]};
}

TendonVector add(const TendonVector & a, const TendonVector & b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

TendonVector subtract(const TendonVector & a, const TendonVector & b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

TendonVector scale(double s, const TendonVector & a)
{
  return {s * a[0], s * a[1], s * a[2]};
}

TendonVector cross(const TendonVector & a, const TendonVector & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]};
}

double dot(const TendonVector & a, const TendonVector & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

TendonVector rotate(const TendonRotation & rotation, const TendonVector & value)
{
  return {dot(rotation[0], value), dot(rotation[1], value), dot(rotation[2], value)};
}

TendonRotation multiply(const TendonRotation & a, const TendonRotation & b)
{
  TendonRotation product{};
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      product[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
    }
  }
  return product;
}

TendonRotation rotationAbout(const TendonVector & axis, double angle)
{
  const auto [x, y, z] = axis;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const double t = 1.0 - c;
  return {{{c + x * x * t, x * y * t - z * s, x * z * t + y * s},
    {y * x * t + z * s, c + y * y * t, y * z * t - x * s},
    {z * x * t - y * s, z * y * t + x * s, c + z * z * t}}};
}

template<typename Range>
void requireFinite(const Range & values, const char * field)
{
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(field);
    }
  }
}

double maxAbs(const Active & values)
{
  double result = 0.0;
  for (double value : values) {
    result = std::max(result, std::abs(value));
  }
  return result;
}

// Cholesky factorization checks the positive stiffness of the static equilibrium.
// A non-positive pivot is rejected rather than following an unstable branch.
Active solvePositiveDefinite(const ActiveMatrix & matrix, Active rhs)
{
  ActiveMatrix lower{};
  for (std::size_t i = 0; i < kActiveCount; ++i) {
    for (std::size_t j = 0; j <= i; ++j) {
      double pivot = (matrix[i][j] + matrix[j][i]) * 0.5;
      for (std::size_t k = 0; k < j; ++k) {
        pivot -= lower[i][k] * lower[j][k];
      }
      if (i == j) {
        if (!std::isfinite(pivot) || pivot <= 1e-10) {
          throw std::runtime_error("static feedback Jacobian is singular or unstable");
        }
        lower[i][j] = std::sqrt(pivot);
      } else {
        lower[i][j] = pivot / lower[j][j];
      }
    }
  }
  for (std::size_t i = 0; i < kActiveCount; ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      rhs[i] -= lower[i][j] * rhs[j];
    }
    rhs[i] /= lower[i][i];
  }
  for (std::size_t i = kActiveCount; i-- > 0;) {
    for (std::size_t j = i + 1; j < kActiveCount; ++j) {
      rhs[i] -= lower[j][i] * rhs[j];
    }
    rhs[i] /= lower[i][i];
  }
  return rhs;
}

}  // namespace

StaticTendonModel::StaticTendonModel(const TendonConfig & config)
: config_(config)
{
  requireFinite(config_.gravity_m_s2, "gravity must be finite");
  requireFinite(config_.payload_com_m, "payload COM must be finite");
  if (!std::isfinite(config_.payload_mass_kg) || config_.payload_mass_kg < 0.0) {
    throw std::invalid_argument("payload mass must be finite and nonnegative");
  }
  for (std::size_t i = 0; i < origin_rotation_.size(); ++i) {
    requireFinite(config_.origin_xyz_m[i], "joint origins must be finite");
    requireFinite(config_.origin_rpy_rad[i], "joint RPY must be finite");
    const auto [roll, pitch, yaw] = vector(config_.origin_rpy_rad[i]);
    origin_rotation_[i] = multiply(
      multiply(rotationAbout({0., 0., 1.}, yaw), rotationAbout({0., 1., 0.}, pitch)),
      rotationAbout({1., 0., 0.}, roll));
  }
  for (std::size_t i = 0; i < axis_.size(); ++i) {
    requireFinite(config_.com_m[i], "link COM must be finite");
    const auto axis = vector(config_.axis[i]);
    const double length = std::hypot(axis[0], axis[1], axis[2]);
    if (!std::isfinite(length) || length <= 0.0 ||
      !std::isfinite(config_.lower_rad[i]) || !std::isfinite(config_.upper_rad[i]) ||
      config_.lower_rad[i] >= config_.upper_rad[i])
    {
      throw std::invalid_argument("invalid revolute joint axis or limits");
    }
    axis_[i] = {axis[0] / length, axis[1] / length, axis[2] / length};
    if (!std::isfinite(config_.mass_kg[i]) || config_.mass_kg[i] < 0.0) {
      throw std::invalid_argument("link masses must be finite and nonnegative");
    }
    const double radius_m = config_.radius_mm[i] / 1000.0;
    const double rope_stiffness_n_per_m = config_.rope_stiffness_n_per_mm[i] * 1000.0;
    stiffness_nm_per_rad_[i] = 2.0 * rope_stiffness_n_per_m * radius_m * radius_m;
    if (radius_m <= 0.0 || !std::isfinite(stiffness_nm_per_rad_[i]) ||
      stiffness_nm_per_rad_[i] <= 0.0)
    {
      throw std::invalid_argument("rope stiffness and radius must be finite and positive");
    }
  }
}

TendonResult StaticTendonModel::evaluate(const TendonAngles & nominal) const
{
  requireFinite(nominal, "joint angles must be finite");
  std::array<TendonVector, 6> joint_origin{}, world_axis{}, com{};
  TendonVector translation{};
  TendonRotation rotation{{{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}}};
  for (std::size_t i = 0; i < origin_rotation_.size(); ++i) {
    translation = add(translation, rotate(rotation, vector(config_.origin_xyz_m[i])));
    rotation = multiply(rotation, origin_rotation_[i]);
    if (i >= 2) {
      const std::size_t joint = i - 2;
      joint_origin[joint] = translation;
      world_axis[joint] = rotate(rotation, axis_[joint]);
      rotation = multiply(rotation, rotationAbout(axis_[joint], nominal[joint]));
      com[joint] = add(translation, rotate(rotation, vector(config_.com_m[joint])));
    }
  }
  const auto gravity = vector(config_.gravity_m_s2);
  const auto payload_com = add(translation, rotate(rotation, vector(config_.payload_com_m)));
  TendonResult result;
  std::copy(nominal.begin(), nominal.end(), result.effective_rad);
  for (const auto i : kTendonAxes) {
    auto moment = cross(
      subtract(payload_com, joint_origin[i]), scale(config_.payload_mass_kg, gravity));
    for (std::size_t link = i; link < nominal.size(); ++link) {
      moment = add(moment, cross(
        subtract(com[link], joint_origin[i]), scale(config_.mass_kg[link], gravity)));
    }
    result.hold_torque_nm[i] = -dot(world_axis[i], moment);
    result.compensation_rad[i] = result.hold_torque_nm[i] / stiffness_nm_per_rad_[i];
    result.effective_rad[i] += result.compensation_rad[i];
    result.rope_a_delta_mm[i] = config_.radius_mm[i] * result.compensation_rad[i];
    result.rope_a_delta_n[i] = config_.rope_stiffness_n_per_mm[i] * result.rope_a_delta_mm[i];
  }
  requireFinite(result.hold_torque_nm, "gravity torque exceeded numerical range");
  requireFinite(result.effective_rad, "compensated angle exceeded numerical range");
  requireFinite(result.rope_a_delta_mm, "rope elongation exceeded numerical range");
  requireFinite(result.rope_a_delta_n, "rope tension exceeded numerical range");
  return result;
}

bool StaticTendonModel::withinLimits(const TendonAngles & angles, double tolerance) const
{
  for (std::size_t i = 0; i < angles.size(); ++i) {
    if (!std::isfinite(angles[i]) || angles[i] < config_.lower_rad[i] - tolerance ||
      angles[i] > config_.upper_rad[i] + tolerance)
    {
      return false;
    }
  }
  return true;
}

TendonAngles StaticTendonModel::estimate(
  const TendonAngles & encoder_equivalent, const TendonAngles * seed) const
{
  requireFinite(encoder_equivalent, "encoder angles must be finite");
  TendonAngles estimated = encoder_equivalent;
  Active sqrt_stiffness{};
  for (std::size_t j = 0; j < kActiveCount; ++j) {
    const auto i = kTendonAxes[j];
    const auto start = seed && std::isfinite((*seed)[i]) ? (*seed)[i] : encoder_equivalent[i];
    estimated[i] = std::clamp(start, config_.lower_rad[i], config_.upper_rad[i]);
    sqrt_stiffness[j] = std::sqrt(stiffness_nm_per_rad_[i]);
  }
  if (!withinLimits(estimated, kTendonEncoderLimitToleranceRad)) {
    throw std::out_of_range("uncompensated encoder joint is outside the URDF limits");
  }
  const auto residual = [&](const TendonAngles & q) {
      const auto result = evaluate(q);
      Active difference{};
      for (std::size_t j = 0; j < kActiveCount; ++j) {
        const auto i = kTendonAxes[j];
        difference[j] = result.effective_rad[i] - encoder_equivalent[i];
      }
      requireFinite(difference, "static feedback residual exceeded numerical range");
      return difference;
    };
  constexpr double kDerivativeStep = 1e-5;
  constexpr double kResidualTolerance = 1e-10;
  for (int iteration = 0; iteration < 24; ++iteration) {
    const auto error = residual(estimated);
    const double norm = maxAbs(error);
    ActiveMatrix stiffness_jacobian{};
    for (std::size_t j = 0; j < kActiveCount; ++j) {
      auto plus = estimated;
      auto minus = estimated;
      plus[kTendonAxes[j]] += kDerivativeStep;
      minus[kTendonAxes[j]] -= kDerivativeStep;
      const auto torque_plus = evaluate(plus);
      const auto torque_minus = evaluate(minus);
      for (std::size_t row = 0; row < kActiveCount; ++row) {
        const auto i = kTendonAxes[row];
        stiffness_jacobian[row][j] = (row == j ? 1.0 : 0.0) +
          (torque_plus.hold_torque_nm[i] - torque_minus.hold_torque_nm[i]) /
          (2.0 * kDerivativeStep) / sqrt_stiffness[row] / sqrt_stiffness[j];
      }
    }
    Active rhs{};
    for (std::size_t j = 0; j < kActiveCount; ++j) {
      rhs[j] = -sqrt_stiffness[j] * error[j];
    }
    const auto step = solvePositiveDefinite(stiffness_jacobian, rhs);
    if (norm <= kResidualTolerance) {
      return estimated;
    }
    bool accepted = false;
    for (double fraction = 1.0; fraction >= 1.0 / 1024.0; fraction *= 0.5) {
      auto candidate = estimated;
      for (std::size_t j = 0; j < kActiveCount; ++j) {
        candidate[kTendonAxes[j]] += fraction * step[j] / sqrt_stiffness[j];
      }
      if (withinLimits(candidate, kTendonEncoderLimitToleranceRad) &&
        maxAbs(residual(candidate)) < norm)
      {
        estimated = candidate;
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      break;
    }
  }
  throw std::runtime_error("static feedback has no converged equilibrium within joint limits");
}

}  // namespace arm_hardware

namespace
{

arm_hardware::TendonAngles toAngles(const double * values)
{
  arm_hardware::TendonAngles result;
  std::copy_n(values, result.size(), result.begin());
  return result;
}

void setError(char * error, std::size_t capacity, const std::exception & exception)
{
  if (error && capacity) {
    std::snprintf(error, capacity, "%s", exception.what());
  }
}

}  // namespace

unsigned arm_tendon_abi_version()
{
  return 1;
}

void * arm_tendon_create(
  const arm_hardware::TendonConfig * config, char * error, std::size_t capacity)
{
  try {
    return new arm_hardware::StaticTendonModel(*config);
  } catch (const std::exception & exception) {
    setError(error, capacity, exception);
    return nullptr;
  }
}

void arm_tendon_destroy(void * model)
{
  delete static_cast<arm_hardware::StaticTendonModel *>(model);
}

int arm_tendon_evaluate(
  const void * model, const double * nominal, arm_hardware::TendonResult * result,
  char * error, std::size_t capacity)
{
  try {
    *result = static_cast<const arm_hardware::StaticTendonModel *>(model)->evaluate(toAngles(nominal));
    return 0;
  } catch (const std::exception & exception) {
    setError(error, capacity, exception);
    return 1;
  }
}

int arm_tendon_estimate(
  const void * model, const double * encoder_equivalent, const double * seed, double * estimated,
  char * error, std::size_t capacity)
{
  try {
    const auto seed_angles = seed ? toAngles(seed) : arm_hardware::TendonAngles{};
    const auto result = static_cast<const arm_hardware::StaticTendonModel *>(model)->estimate(
      toAngles(encoder_equivalent), seed ? &seed_angles : nullptr);
    std::copy(result.begin(), result.end(), estimated);
    return 0;
  } catch (const std::exception & exception) {
    setError(error, capacity, exception);
    return 1;
  }
}

void arm_tendon_coupled_matrix(double * matrix)
{
  for (const auto & row : arm_hardware::kCoupledMotorMatrix) {
    matrix = std::copy_n(row, 6, matrix);
  }
}
