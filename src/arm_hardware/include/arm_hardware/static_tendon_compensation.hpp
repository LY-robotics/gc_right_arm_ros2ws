#pragma once

#include <array>
#include <cstddef>

#if defined(_WIN32)
#ifdef ARM_TENDON_BUILD_DLL
#define ARM_TENDON_API __declspec(dllexport)
#else
#define ARM_TENDON_API __declspec(dllimport)
#endif
#else
#define ARM_TENDON_API
#endif

namespace arm_hardware
{

using TendonAngles = std::array<double, 6>;
using TendonVector = std::array<double, 3>;
using TendonRotation = std::array<TendonVector, 3>;

struct TendonConfig
{
  // The first two origins are fixed (Joint0 and Joint0_1).
  double origin_xyz_m[8][3]{};
  double origin_rpy_rad[8][3]{};
  double axis[6][3]{};
  double lower_rad[6]{};
  double upper_rad[6]{};
  double mass_kg[6]{};
  double com_m[6][3]{};
  double rope_stiffness_n_per_mm[6]{};
  double radius_mm[6]{};
  double gravity_m_s2[3]{};
  double payload_mass_kg{};
  double payload_com_m[3]{};
};

struct TendonResult
{
  double hold_torque_nm[6]{};
  double compensation_rad[6]{};
  double effective_rad[6]{};
  // Signed changes relative to pretension, for rope A. Rope B is the opposite.
  double rope_a_delta_mm[6]{};
  double rope_a_delta_n[6]{};
};

inline constexpr std::array<std::size_t, 4> kTendonAxes{{0, 1, 3, 5}};
inline constexpr double kTendonEncoderLimitToleranceRad = 1e-5;
inline constexpr double kCoupledMotorMatrix[6][6] = {
  {-100.8, 0., 0., 0., 0., 0.},
  {0., 36., -43.2, 0., 0., 0.},
  {0., -36., -43.2, 0., 0., 0.},
  {0., 0., 0., -64.8, 0., 0.},
  {0., 0., 0., 16., 13., 0.},
  {0., 0., 0., 0., 0., 15.}
};

class ARM_TENDON_API StaticTendonModel
{
public:
  explicit StaticTendonModel(const TendonConfig & config);

  TendonResult evaluate(const TendonAngles & nominal) const;
  // Throws if no nearby stable equilibrium is found within the iteration budget.
  TendonAngles estimate(const TendonAngles & encoder_equivalent, const TendonAngles * seed) const;
  bool withinLimits(const TendonAngles & angles, double tolerance = 0.0) const;

private:
  TendonConfig config_;
  std::array<TendonRotation, 8> origin_rotation_{};
  std::array<TendonVector, 6> axis_{};
  TendonAngles stiffness_nm_per_rad_{};
};

}  // namespace arm_hardware

// Flat C ABI for the Python SDK and the offline UI. A status of zero means success.
extern "C"
{
ARM_TENDON_API unsigned arm_tendon_abi_version();
ARM_TENDON_API void * arm_tendon_create(
  const arm_hardware::TendonConfig * config, char * error, std::size_t capacity);
ARM_TENDON_API void arm_tendon_destroy(void * model);
ARM_TENDON_API int arm_tendon_evaluate(
  const void * model, const double * nominal, arm_hardware::TendonResult * result,
  char * error, std::size_t capacity);
ARM_TENDON_API int arm_tendon_estimate(
  const void * model, const double * encoder_equivalent, const double * seed, double * estimated,
  char * error, std::size_t capacity);
ARM_TENDON_API void arm_tendon_coupled_matrix(double * matrix);
}
