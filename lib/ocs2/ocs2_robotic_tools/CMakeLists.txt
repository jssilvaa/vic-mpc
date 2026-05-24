# Transplanted subset of ocs2_robotic_tools (same transplant-not-import rationale
# as ocs2_core). Header-only for now: RotationDerivativesTransforms.h (CppAD-free,
# Eigen-only). NOTE: RotationTransforms.h (getRotationMatrixFromZyxEulerAngles,
# quaternionDistance) is intentionally NOT transplanted yet — it includes
# <ocs2_core/automatic_differentiation/Types.h> and defines ad_scalar_t overloads,
# i.e. it's CppAD-coupled (deferred). It's only consumed by D3 cost terms; bring it
# in (CppAD-free subset) when D3 needs it.
add_library(ocs2_robotic_tools INTERFACE)

target_include_directories(ocs2_robotic_tools INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(ocs2_robotic_tools INTERFACE
  Eigen3::Eigen)

add_library(ocs2::robotic_tools ALIAS ocs2_robotic_tools)
