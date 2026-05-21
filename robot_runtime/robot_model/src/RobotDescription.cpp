#include "robot_model/RobotDescription.h"

#include <urdfdom/urdf_parser/urdf_parser.h>

#include <filesystem>
#include <fstream>

namespace robot::model {

RobotDescription::RobotDescription(const std::string& urdfPath) : urdf_path_(urdfPath) {
  if (!std::filesystem::exists(urdfPath)) {
    throw std::runtime_error("URDF file not found: " + urdfPath);
  }

  std::ifstream urdfFile(urdfPath);
  std::string urdfContent((std::istreambuf_iterator<char>(urdfFile)), std::istreambuf_iterator<char>());
  urdfFile.close();

  urdf::ModelInterfaceSharedPtr urdfModel = urdf::parseURDF(urdfContent);
  if (!urdfModel) {
    throw std::runtime_error("Failed to parse URDF file: " + urdfPath);
  }

  int32_t jointId = 0;
  for (const auto& jointPair : urdfModel->joints_) {
    const std::string& jointName = jointPair.first;
    const urdf::JointSharedPtr& joint = jointPair.second;

    // Skip fixed joints, continuous joints, and other non-controllable types.
    if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC) {
      continue;
    }

    JointDescription jointDesc;
    jointDesc.id = jointId;

    if (joint->limits) {
      jointDesc.min_angle = joint->limits->lower;
      jointDesc.max_angle = joint->limits->upper;
      jointDesc.max_velocity = joint->limits->velocity;
      jointDesc.max_effort = joint->limits->effort;
    }

    joint_name_description_map_[jointName] = jointDesc;
    joint_id_name_map_[jointId] = jointName;

    jointId++;
  }

  if (joint_name_description_map_.empty()) {
    throw std::runtime_error("No valid joints found in URDF: " + urdfPath);
  }

  joint_indices.reserve(joint_id_name_map_.size());
  joint_names.reserve(joint_id_name_map_.size());

  for (const auto& [index, name] : joint_id_name_map_) {
    joint_indices.push_back(index);
    joint_names.push_back(name);
  }
}

bool RobotDescription::containsJoint(const std::string& jointName) const {
  return joint_name_description_map_.find(jointName) != joint_name_description_map_.end();
}

std::vector<joint_index_t> RobotDescription::getJointIndices(const std::vector<std::string>& jointNames) const {
  std::vector<joint_index_t> jointIndices;
  jointIndices.reserve(jointNames.size());
  for (const std::string& jointName : jointNames) {
    jointIndices.emplace_back(getJointIndex(jointName));
  }
  return jointIndices;
}

const std::string RobotDescription::getURDFName() const {
  std::size_t lastSlashPos = urdf_path_.find_last_of("/");
  if (lastSlashPos == std::string::npos) {
    return urdf_path_;
  }
  return urdf_path_.substr(lastSlashPos + 1);
}

std::ostream& operator<<(std::ostream& os, const JointDescription& joint) {
  os << "JointDescription { " << "id: " << joint.id << ", min_angle: " << joint.min_angle << ", max_angle: " << joint.max_angle
     << ", max_velocity: " << joint.max_velocity << ", max_effort: " << joint.max_effort << " }";
  return os;
}

std::ostream& operator<<(std::ostream& os, const RobotDescription& robot) {
  os << "RobotDescription {" << std::endl;
  os << "Generated from URDF: " << robot.getURDFPath() << std::endl;
  os << " Joint names and descriptions:" << std::endl;
  for (const auto& joint : robot.joint_name_description_map_) {
    os << "  {" << joint.first << ": " << joint.second << " }" << std::endl;
  }
  os << "}";
  return os;
}

}  // namespace robot::model
