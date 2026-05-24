#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <urdf_model/model.h>
#include <memory>
#include "ocs2_core/Types.h"

namespace ocs2 {

// Templated like the reference's ocs2_pinocchio_interface (PinocchioInterfaceTpl<SCALAR>)
// so that centroidal/mapping functions templated on SCALAR_T can DEDUCE SCALAR_T from a
// PinocchioInterfaceTpl<SCALAR_T>& argument. (An earlier `using PinocchioInterfaceTpl =
// PinocchioInterface` alias-shim allowed explicit <scalar_t> uses but broke deduction.)
// We still only ever instantiate scalar_t — ad_scalar_t would require CppAD (deferred).
template <typename SCALAR>
class PinocchioInterfaceTpl final {
 public:
  using Model = pinocchio::ModelTpl<SCALAR>;
  using Data = pinocchio::DataTpl<SCALAR>;

  PinocchioInterfaceTpl(Model model, urdf::ModelInterfaceSharedPtr urdfTree)
      : model_(std::move(model)), data_(model_), urdfTree_(std::move(urdfTree)) {}

  PinocchioInterfaceTpl(const PinocchioInterfaceTpl&) = default;
  PinocchioInterfaceTpl(PinocchioInterfaceTpl&&) noexcept = default;
  PinocchioInterfaceTpl& operator=(const PinocchioInterfaceTpl&) = default;
  PinocchioInterfaceTpl& operator=(PinocchioInterfaceTpl&&) noexcept = default;
  ~PinocchioInterfaceTpl() = default;

  std::unique_ptr<PinocchioInterfaceTpl> clone() const { return std::make_unique<PinocchioInterfaceTpl>(*this); }

  const Model& getModel() const { return model_; }
  Model& getModel() { return model_; }

  const Data& getData() const { return data_; }
  Data& getData() { return data_; }

  const urdf::ModelInterfaceSharedPtr& getUrdfModelPtr() const { return urdfTree_; }

 private:
  Model model_;
  Data data_;
  urdf::ModelInterfaceSharedPtr urdfTree_;
};

using PinocchioInterface = PinocchioInterfaceTpl<scalar_t>;

}  // namespace ocs2
