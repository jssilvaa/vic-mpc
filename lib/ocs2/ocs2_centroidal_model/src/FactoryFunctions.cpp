/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "ocs2_centroidal_model/FactoryFunctions.h"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

#include <ocs2_core/misc/LoadData.h>

// vic-mpc: the two createPinocchioInterface(...) builders are dropped — they relied on
// ocs2_pinocchio_interface/urdf.h's getPinocchioInterfaceFromUrdf{File,Model}, which we
// don't transplant. Callers use our createDefault/CustomPinocchioInterface instead.

namespace ocs2 {
namespace centroidal_model {

CentroidalModelInfo createCentroidalModelInfo(const PinocchioInterface& interface, const CentroidalModelType& type,
                                              const vector_t& nominalJointAngles, const std::vector<std::string>& threeDofContactNames,
                                              const std::vector<std::string>& sixDofContactNames) {
  const auto& model = interface.getModel();
  auto data = interface.getData();

  if (model.nq != nominalJointAngles.size() + 6) {
    const int expectedNumJoints = model.nq - 6;
    throw std::runtime_error("[CentroidalModelInfo] nominalJointAngles.size() should be " + std::to_string(expectedNumJoints));
  }

  CentroidalModelInfoTpl<scalar_t> info;
  info.centroidalModelType = type;
  info.numThreeDofContacts = threeDofContactNames.size();
  info.numSixDofContacts = sixDofContactNames.size();
  info.generalizedCoordinatesNum = model.nq;
  info.actuatedDofNum = info.generalizedCoordinatesNum - 6;
  info.stateDim = info.generalizedCoordinatesNum + 6;
  info.inputDim = info.actuatedDofNum + 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  info.robotMass = pinocchio::computeTotalMass(model);

  for (const auto& name : threeDofContactNames) {
    info.endEffectorFrameIndices.push_back(model.getFrameId(name));
  }

  for (const auto& name : sixDofContactNames) {
    info.endEffectorFrameIndices.push_back(model.getFrameId(name));
  }

  // make sure the nominal base frame is aligned with the world frame
  info.qPinocchioNominal.resize(model.nq);
  info.qPinocchioNominal << vector_t::Zero(6), nominalJointAngles;
  info.centroidalInertiaNominal.setZero();
  info.comToBasePositionNominal.setZero();
  if (info.centroidalModelType == CentroidalModelType::SingleRigidBodyDynamics) {
    const vector_t vPinocchioNominal = vector_t::Zero(info.generalizedCoordinatesNum);
    pinocchio::ccrba(model, data, info.qPinocchioNominal, vPinocchioNominal);
    info.centroidalInertiaNominal = data.Ig.inertia().matrix();
    info.comToBasePositionNominal = info.qPinocchioNominal.template head<3>() - data.com[0];
  }

  return info;
}

CentroidalModelType loadCentroidalType(const std::string& configFilePath, const std::string& fieldName) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<CentroidalModelType>(type);
}

vector_t loadDefaultJointState(size_t numJointState, const std::string& configFilePath, const std::string& fieldName) {
  vector_t defaultJoints(numJointState);
  ocs2::loadData::loadEigenMatrix(configFilePath, fieldName, defaultJoints);
  return defaultJoints;
}

}  // namespace centroidal_model
}  // namespace ocs2
