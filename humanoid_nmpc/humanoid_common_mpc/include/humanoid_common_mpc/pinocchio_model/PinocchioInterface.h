#pragma once 

#include <pinocchio/fwd.hpp> 
#include <pinocchio/multibody/model.hpp> 
#include <pinocchio/multibody/data.hpp> 
#include <urdf_model/model.h> 
#include <memory> 
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2 { 


  class PinocchioInterface final {
    public:
      using Model = pinocchio::ModelTpl<scalar_t>; 
      using Data  = pinocchio::DataTpl<scalar_t>;

      PinocchioInterface(Model model, urdf::ModelInterfaceSharedPtr urdfTree)
        : model_(std::move(model)), 
          data_(model_), 
          urdfTree_(std::move(urdfTree)) {}

      PinocchioInterface(const PinocchioInterface&);
      PinocchioInterface(PinocchioInterface&&) noexcept;
      PinocchioInterface& operator=(const PinocchioInterface&);
      PinocchioInterface& operator=(PinocchioInterface&&) noexcept;
      ~PinocchioInterface();

      std::unique_ptr<PinocchioInterface> clone() const { 
        return std::make_unique<PinocchioInterface>(*this); 
      }
      
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

} // namespace ocs2 