#pragma once 

#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ocs2 {
  
  template <typename SCALAR_T> 
  using VECTOR_T = Eigen::Matrix<SCALAR_T, -1, 1>;
  template <typename SCALAR_T> 
  using VECTOR2_T = Eigen::Matrix<SCALAR_T, 2, 1>; 
  template <typename SCALAR_T> 
  using VECTOR3_T = Eigen::Matrix<SCALAR_T, 3, 1>;
  template <typename SCALAR_T> 
  using VECTOR4_T = Eigen::Matrix<SCALAR_T, 4, 1>;
  template <typename SCALAR_T> 
  using VECTOR6_T = Eigen::Matrix<SCALAR_T, 6, 1>; 
  template <typename SCALAR_T> 
  using VECTOR12_T = Eigen::Matrix<SCALAR_T, 12, 1>; 
  template <typename SCALAR_T> 
  using MATRIX_T = Eigen::Matrix<SCALAR_T, -1, -1>;
  template <typename SCALAR_T> 
  using MATRIX3_T = Eigen::Matrix<SCALAR_T, 3, 3>; 
  template <typename SCALAR_T> 
  using MATRIX4_T = Eigen::Matrix<SCALAR_T, 4, 4>; 
  template <typename SCALAR_T> 
  using MATRIX6_T = Eigen::Matrix<SCALAR_T, 6, 6>; 
  template <typename SCALAR_T> 
  using QUATERNION_T = Eigen::Quaternion<SCALAR_T>; 

  using scalar_t = double;
  // Dynamic-size aliases (in the reference these come from ocs2_core/Types.h).
  using vector_t = VECTOR_T<scalar_t>;
  using matrix_t = MATRIX_T<scalar_t>;
  using vector2_t = VECTOR2_T<scalar_t>;
  using vector3_t = VECTOR3_T<scalar_t>;
  using vector4_t = VECTOR4_T<scalar_t>;
  using vector6_t = VECTOR6_T<scalar_t>;
  using vector12_t = VECTOR12_T<scalar_t>;
  using matrix3_t = MATRIX3_T<scalar_t>;
  using matrix4_t = MATRIX4_T<scalar_t>;
  using matrix6_t = MATRIX6_T<scalar_t>;
  using quaternion_t = QUATERNION_T<scalar_t>;

 /* ============================================
  * Contacts definition 
  *  Wrench F = [f_x, f_y, f_z, M_x, M_y, M_z]^T
  * ============================================ */

  constexpr size_t N_CONTACTS = 2; 
  constexpr size_t CONTACT_WRENCH_DIM = 6; 

  template <typename T>
  using feet_array_t = std::array<T, N_CONTACTS>; 
  
  template <typename T> 
  using feet_vec_t = std::vector<T>; 

  // Feet contact mask [is_left_contact, is_right_contact]
  using contact_flag_t = feet_array_t<bool>; 

} // namespace ocs2