/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

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

#include "humanoid_common_mpc/common/ModelSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <stdexcept>

#include <ocs2_core/misc/LoadData.h>

#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"


namespace ocs2::humanoid {

/// Helper functions contained in a local anonymous namespace

namespace {

  /// @brief Creates a Joint Index Map from a list of joint names. 
  /// @param[in] jointNames the joint names.
  /// @param[in] offset the offset to add to the joint indices map.
  /// @returns jointIndexMap, a map from joint names to joint indices, with the offset accounted for.
  
  static std::unordered_map<std::string, size_t> createJointIndexMap(const std::vector<std::string>& jointNames, size_t offset = 0) {
    std::unordered_map<std::string, size_t> jointIndexMap{};
    for (size_t i = 0; i < jointNames.size(); i++) {
      jointIndexMap[jointNames[i]] = i + offset;
    }
    return jointIndexMap;
  }


  /// @brief Initializes the mpc model joint names vector without the fixed joints.
  /// @param[in] fullJointNames a vector with all joint names.
  /// @param[in] fixedJointNames a vector with only the fixed joint names.
  /// @param[in] verbose a on/off flag to toggle diagnostics.
  /// @returns mpcModelJointNames a vector with only the free joint names.
  
  static std::vector<std::string> initializeJointNames(const std::vector<std::string>& fullJointNames, const std::vector<std::string>& fixedJointNames, bool verbose) {
    
    if (fullJointNames.size() <= fixedJointNames.size()) {
      throw std::invalid_argument("Number of joints must be greater than zero.");
    }
    size_t n_joints = fullJointNames.size() - fixedJointNames.size();

    // reserve space for joint names in the return variable
    std::vector<std::string> mpcModelJointNames{};
    mpcModelJointNames.reserve(n_joints);
    for (const std::string& jointName : fullJointNames) {
      if (std::find(fixedJointNames.begin(), fixedJointNames.end(), jointName) == fixedJointNames.end()) {
        mpcModelJointNames.push_back(jointName);
      }
    }

    // diagnostics
    if (verbose) {
      std::cout << "Num active joints: " << n_joints << std::endl;
      std::cout << "Initialize the following active MPC joints: " << std::endl;
      for (auto& jointName : mpcModelJointNames) {
        std::cout << "Joint " << jointName << std::endl;
      }
    }

    return mpcModelJointNames;
  }


  /// @brief Creates the joint index map from mpc joint names
  /// @param[in] fullJointNames the joint names vector.
  /// @param[in] mpcModelJointNames the mpc joint names vector.
  /// @returns mpcModelJointIndices the mpc joint indices vector.
  
  std::vector<size_t> initializeMpcToFullJointIndices(const std::vector<std::string>& fullJointNames, const std::vector<std::string>& mpcModelJointNames) {
    std::unordered_map<std::string, size_t> fullJointIndexMap = createJointIndexMap(fullJointNames);
    std::vector<size_t> mpcModelJointIndices;
    mpcModelJointIndices.reserve(mpcModelJointNames.size());
    for (size_t i = 0; i < mpcModelJointNames.size(); i++) {
      mpcModelJointIndices.push_back(fullJointIndexMap.at(mpcModelJointNames[i]));
    }
    return mpcModelJointIndices;
  }

  /// @brief Concatenates two string vectors (using copy semantics)
  /// @param[in] a the first string vector.
  /// @param[in] b the second string vector.
  /// @returns the concatenated string vectors b followed by a
  std::vector<std::string> concatenateStringVectors(const std::vector<std::string>& a, const std::vector<std::string>& b) {
    std::vector<std::string> tmp_vec(a);
    tmp_vec.insert(tmp_vec.begin(), b.begin(), b.end());
    return tmp_vec;
  }

} // namespace


ModelSettings::ModelSettings(const std::string& configFile, const std::string& urdfFile, const std::string& mpcName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFile, pt);

  std::string prefix{"model_settings."};

  if (verbose) {
    std::cerr << "\n #### Robot Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, this->robotName, prefix + "robotName", verbose);
  loadData::loadPtreeValue(pt, this->verboseCppAd, prefix + "verboseCppAd", verbose);
  loadData::loadPtreeValue(pt, this->recompileLibrariesCppAd, prefix + "recompileLibrariesCppAd", verbose);
  loadData::loadPtreeValue(pt, this->phaseTransitionStanceTime, prefix + "phaseTransitionStanceTime", verbose);

  loadData::loadPtreeValue(pt, this->j_l_shoulder_y_name, prefix + "armJointNames.left_shoulder_y", verbose);
  loadData::loadPtreeValue(pt, this->j_r_shoulder_y_name, prefix + "armJointNames.right_shoulder_y", verbose);
  loadData::loadPtreeValue(pt, this->j_l_elbow_y_name, prefix + "armJointNames.left_elbow_y", verbose);
  loadData::loadPtreeValue(pt, this->j_r_elbow_y_name, prefix + "armJointNames.right_elbow_y", verbose);
  modelFolderCppAd = "cppad_code_gen/cppad_" + mpcName + robotName;

  loadData::loadStdVector(configFile, prefix + "fixedJointNames", fixedJointNames, verbose);
  loadData::loadStdVector(configFile, prefix + "contactNames6DoF", contactNames6DoF, verbose);
  loadData::loadStdVector(configFile, prefix + "contactParentJointNames", contactParentJointNames, verbose);

  if (verbose) {
    std::cout << "Initializing MPC by fixing joints: " << std::endl;
    for (std::string fixedJoint : fixedJointNames) std::cout << fixedJoint << std::endl;
  }

  // Get full joint order from a full pinocchio interface, this removes any joints marked as fix in the urdf.
  PinocchioInterface fullPinocchioInterface = createDefaultPinocchioInterface(urdfFile);
  const pinocchio::Model& model = fullPinocchioInterface.getModel();
  if (verbose) std::cout << "Full URDF joints: " << std::endl;
  fullJointNames.reserve(model.njoints - 2);  // Substract universe and root joint
  for (pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id) {
    if (verbose) std::cout << model.names[joint_id] << std::endl;
    fullJointNames.emplace_back(model.names[joint_id]);
  }

  this->mpcModelJointNames = initializeJointNames(this->fullJointNames, this->fixedJointNames, verbose);
  this->mpcModelToFullJointsIndices = initializeMpcToFullJointIndices(this->fullJointNames, this->mpcModelJointNames);
  this->jointIndexMap = createJointIndexMap(this->mpcModelJointNames);
  this->contactNames = concatenateStringVectors(this->contactNames3DoF, this->contactNames6DoF);

  this->mpc_joint_dim = this->mpcModelJointNames.size();
  this->full_joint_dim = this->fullJointNames.size();

  j_l_shoulder_y_index = this->jointIndexMap.at(j_l_shoulder_y_name);
  j_r_shoulder_y_index = this->jointIndexMap.at(j_r_shoulder_y_name);
  j_l_elbow_y_index = this->jointIndexMap.at(j_l_elbow_y_name);
  j_r_elbow_y_index = this->jointIndexMap.at(j_r_elbow_y_name);

  const std::string footConstraintPrefix = prefix + "foot_constraint.";

  if (verbose) {
    std::cerr << "\n #### Robot Model Foot Constraint Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, this->footConstraintConfig.positionErrorGain_z, footConstraintPrefix + "positionErrorGain_z", verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.orientationErrorGain, footConstraintPrefix + "orientationErrorGain", verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearVelocityErrorGain_z, footConstraintPrefix + "linearVelocityErrorGain_z",
                           verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearVelocityErrorGain_xy, footConstraintPrefix + "linearVelocityErrorGain_xy",
                           verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.angularVelocityErrorGain, footConstraintPrefix + "angularVelocityErrorGain",
                           verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearAccelerationErrorGain_z,
                           footConstraintPrefix + "linearAccelerationErrorGain_z", verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearAccelerationErrorGain_xy,
                           footConstraintPrefix + "linearAccelerationErrorGain_xy", verbose);
  loadData::loadPtreeValue(pt, this->footConstraintConfig.angularAccelerationErrorGain,
                           footConstraintPrefix + "angularAccelerationErrorGain", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

} // namespace ocs2::humanoid