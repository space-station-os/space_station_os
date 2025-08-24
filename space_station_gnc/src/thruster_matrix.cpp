// Copyright 2025 Space Station OS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "space_station_gnc/thruster_matrix.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm> 
#include <unordered_set>
#include <sstream>

// ===========================
// Helpers (file-local)
// ===========================

namespace
{

// Convert URDF pose (RPY + translation) to Eigen Isometry
inline Eigen::Isometry3d poseToIso(const urdf::Pose & p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  p.rotation.getRPY(roll, pitch, yaw);
  Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
  T.linear() = (Rz * Ry * Rx).toRotationMatrix();
  return T;
}

// DFS over the URDF tree to compute base->link and base->joint transforms.
// Base is the URDF root link frame. Joint transform is at the joint origin.
void buildTransformsDFS(
  const urdf::ModelInterfaceSharedPtr & model,
  const urdf::LinkConstSharedPtr & link,
  const Eigen::Isometry3d & T_parent,
  std::map<std::string, Eigen::Isometry3d> & T_map_link,
  std::map<std::string, Eigen::Isometry3d> & T_map_joint)
{
  T_map_link[link->name] = T_parent;

  for (const auto & j : link->child_joints) {
    if (!j) continue;
    const Eigen::Isometry3d T_pj = poseToIso(j->parent_to_joint_origin_transform);
    const Eigen::Isometry3d T_base_joint = T_parent * T_pj;
    T_map_joint[j->name] = T_base_joint;

    urdf::LinkConstSharedPtr child_link = model->getLink(j->child_link_name);

    if (child_link) {
      buildTransformsDFS(model, child_link, T_base_joint, T_map_link, T_map_joint);
    }
  }
}

// Build the ordered list of thruster names by scanning joints_ in map order.
// Column ordering in all matrices follows this vector.
inline std::vector<std::string>
orderedThrusterNames(const urdf::ModelInterfaceSharedPtr & model, const URDFUtils & utils)
{
  std::vector<std::string> names;
  if (!model) return names;
  names.reserve(model->joints_.size());
  for (const auto & kv : model->joints_) {
    const auto & j = kv.second;
    if (j && utils.isThruster(j->child_link_name)) {
      names.push_back(j->child_link_name);
    }
  }
  return names;
}

} // namespace


//
// URDFUtils
//

URDFUtils::URDFUtils() = default;

bool URDFUtils::isThruster(const std::string & name) const
{
  if (name.rfind("thruster_", 0) == 0) return true;
  if (name.rfind("thr_", 0) == 0)      return true;
  if (name.rfind("th_", 0) == 0)       return true;
  return false;
}

std::size_t URDFUtils::getNumAct(const urdf::ModelInterfaceSharedPtr & model) const
{
  std::size_t count = 0;
  for (const auto & kv : model->joints_) {
    if (isThruster(kv.second->child_link_name)) ++count;
  }
  std::cout <<
    "=========================================================================================" <<
    std::endl;
  std::cout << "getNumAct: Number of thrusters: " << count << std::endl;
  return count;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
URDFUtils::getThrPos(const urdf::ModelInterfaceSharedPtr & model) const
{
  std::cout << "URDFUtils::getThrPos called" << std::endl;
  //std::size_t n = getNumAct(model);

  // 1) count thrusters without calling resize on any fixed-size vector
  std::size_t n = 0;
  for (const auto & kv : model->joints_) {
    const auto & j = kv.second;
    if (j && isThruster(j->child_link_name)) ++n;
  }


  Eigen::Matrix<double, 3, Eigen::Dynamic> pos(3, n);
  if (n == 0) {return pos;}

  // Build transforms from URDF root
  std::map<std::string, Eigen::Isometry3d> T_map_link;
  std::map<std::string, Eigen::Isometry3d> T_map_joint;
  const urdf::LinkConstSharedPtr root = model->getRoot();
  if (!root) {throw std::runtime_error("URDFUtils::getThrPos: URDF root link is null.");}
  buildTransformsDFS(model, root, Eigen::Isometry3d::Identity(), T_map_link, T_map_joint);
  // Fill in thruster positions following the same joint traversal order
  std::size_t idx = 0;
  for (const auto & kv : model->joints_) {
    auto j = kv.second;
    if (!isThruster(j->child_link_name)) continue;

    auto jt = T_map_joint.find(j->name);
    if (jt == T_map_joint.end()) {
      // Fallback: use parent_to_joint_origin in parent frame if base transform is missing
      pos.col(idx) << j->parent_to_joint_origin_transform.position.x,
        j->parent_to_joint_origin_transform.position.y,
        j->parent_to_joint_origin_transform.position.z;
    } else {
      pos.col(idx) = jt->second.translation();
    }
    ++idx;
  }
  return pos;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
URDFUtils::getThrOrient(const urdf::ModelInterfaceSharedPtr & model) const
{
std::cout << "URDFUtils::getThrOrient called" << std::endl;

  //std::size_t n = getNumAct(model);
  // 1) count thrusters without calling resize on any fixed-size vector
  std::size_t n = 0;
  for (const auto & kv : model->joints_) {
    const auto & j = kv.second;
    if (j && isThruster(j->child_link_name)) ++n;
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> orient(3, n);
  if (n == 0) return orient;
  std::map<std::string, Eigen::Isometry3d> T_map_link;
  std::map<std::string, Eigen::Isometry3d> T_map_joint;
  urdf::LinkConstSharedPtr root = model->getRoot();
  if (!root) {throw std::runtime_error("URDFUtils::getThrOrient: URDF root link is null.");}
  buildTransformsDFS(model, root, Eigen::Isometry3d::Identity(), T_map_link, T_map_joint);

  std::size_t idx = 0;
  for (const auto & kv : model->joints_) {
    auto j = kv.second;
    if (!isThruster(j->child_link_name)) continue;

    Eigen::Vector3d axis(j->axis.x, j->axis.y, j->axis.z);
    axis.normalize();

    auto jt = T_map_joint.find(j->name);
    if (jt == T_map_joint.end()) {
      // Fallback: rotate only by the joint origin in parent frame
      const Eigen::Isometry3d Tpj = poseToIso(j->parent_to_joint_origin_transform);
      orient.col(idx) = (Tpj.linear() * axis).normalized();
    } else {
      orient.col(idx) = (jt->second.linear() * axis).normalized();
    }
    ++idx;
  }
  return orient;
}

void URDFUtils::printLinks(
  const std::map<std::string, std::shared_ptr<urdf::Link>> & links) const
{
  for (const auto & kv : links) {
    std::cout << "Link name: " << kv.first << "\n";
  }
}

//
// ThrusterMatrix
//

ThrusterMatrix::ThrusterMatrix() = default;

void ThrusterMatrix::initialize(const std::string & urdf_xml)
{
  model_urdf_ = urdf::parseURDF(urdf_xml);
  if (!model_urdf_) throw std::runtime_error("Failed to parse URDF XML");
  loadURDF(model_urdf_);

}

void ThrusterMatrix::loadURDF(const urdf::ModelInterfaceSharedPtr & mdl)
{
  std::cout << "ThrusterMatrix::loadURDF called" << std::endl;
  if (!mdl) throw std::runtime_error("loadURDF: null model.");
  model_urdf_ = mdl;
  urdf::ModelInterfaceSharedPtr model = model_urdf_;

  // Discover thrusters and derive geometry (column order is orderedThrusterNames)
  const auto names = orderedThrusterNames(model, urdfUtils);
  n_thruster = names.size();

  thruster_order_.clear();
  thruster_order_.reserve(n_thruster);
  for (const auto& nm : names) {
    thruster_order_.push_back(nm);
  }

  // Derive geometry
  thruster_position   = urdfUtils.getThrPos(model);      // 3xN
  thruster_orientation= urdfUtils.getThrOrient(model);   // 3xN
  allocation_mat.resize(3, n_thruster);
  inverse_allocation_mat.resize(n_thruster, 3);

  for (std::size_t i = 0; i < n_thruster; ++i) {
    thruster_orientation.col(i).normalize();
  }

  for (std::size_t i = 0; i < n_thruster; ++i) {
    const Eigen::Vector3d r = thruster_position.col(i) - cgPos;
    const Eigen::Vector3d f = -thruster_orientation.col(i);
    allocation_mat.col(i) = r.cross(f);
  }
  inverse_allocation_mat = pseudoInverse<
      Eigen::Matrix<double, 3, Eigen::Dynamic>,
      Eigen::Matrix<double, Eigen::Dynamic, 3>>(allocation_mat);

 // Build urdf_thrusters_ map (geometry only at this point)
  urdf_thrusters_.clear();
  for (std::size_t i = 0; i < n_thruster; ++i) {
    ThrusterConfig tc;
    tc.name      = names[i];
    tc.position  = thruster_position.col(i);
    tc.direction = thruster_orientation.col(i);
    // ratings are default; properties.yaml may override later
    tc.active    = false;
    urdf_thrusters_[tc.name] = tc;
  }

  // YAML not loaded yet by design.
  pinv_cache_.clear();
  table_map_.clear();
  table_loaded_ = false;
  //current_table_.clear();
  current_mode_.clear();

  if (n_thruster == 0) {
    std::cerr << "[ThrusterMatrix] Warning: no thrusters found in URDF.\n";
  }
}


void ThrusterMatrix::loadProperties(const std::string & yaml_file)
{
  if (!model_urdf_) throw std::runtime_error("loadProperties: URDF must be loaded first.");
  YAML::Node root = YAML::LoadFile(yaml_file);
  if (!root || !root["thrusters"]) {
    throw std::runtime_error("loadProperties: Missing 'thrusters' at root.");
  }

  const auto thr = root["thrusters"];
  for (auto it = thr.begin(); it != thr.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    const YAML::Node entry = it->second;

    auto it_cfg = urdf_thrusters_.find(name);
    if (it_cfg == urdf_thrusters_.end()) {
      // Not registered in URDF; warn only
      std::cerr << "[ThrusterMatrix] Warning: properties for unknown thruster '"
                << name << "' ignored.\n";
      continue;
    }

    if (entry["max_force"])  it_cfg->second.max_force  = entry["max_force"].as<double>();
    if (entry["isp"])        it_cfg->second.isp        = entry["isp"].as<double>();
    if (entry["efficiency"]) it_cfg->second.efficiency = entry["efficiency"].as<double>();
  }
}

void ThrusterMatrix::loadTable(const std::string & yaml_file)
{
  if (!model_urdf_) throw std::runtime_error("loadTable: URDF must be loaded first.");
  YAML::Node root = YAML::LoadFile(yaml_file);
  if (!root || !root["tables"]) {
    throw std::runtime_error("loadTable: Missing 'tables' at root.");
  }

  // Set of URDF-registered names for validation
  std::unordered_set<std::string> registered;
  registered.reserve(urdf_thrusters_.size());
  for (const auto & kv : urdf_thrusters_) {
    registered.insert(kv.first);
  }
  table_map_.clear();

  const YAML::Node tables = root["tables"];
  for (auto it = tables.begin(); it != tables.end(); ++it) {
    const std::string mode_name = it->first.as<std::string>();
    const YAML::Node  tableNode = it->second;

    ThrusterTable table;
    std::vector<std::string> unknown_in_yaml;

    for (auto jt = tableNode.begin(); jt != tableNode.end(); ++jt) {
      const std::string thr_name = jt->first.as<std::string>();
      if (registered.find(thr_name) == registered.end()) {
        unknown_in_yaml.push_back(thr_name);
        continue; // collect all, then throw
      }

      const YAML::Node entry = jt->second;
      ThrusterWeight w;
      w.torque_weight = Eigen::Vector3d::Zero();
      w.force_weight  = Eigen::Vector3d::Zero();
      if (entry["torque"] && entry["torque"].IsSequence() && entry["torque"].size() == 3) {
        w.torque_weight = Eigen::Vector3d(entry["torque"][0].as<double>(),
                                          entry["torque"][1].as<double>(),
                                          entry["torque"][2].as<double>());
      }
      if (entry["force"] && entry["force"].IsSequence() && entry["force"].size() == 3) {
        w.force_weight  = Eigen::Vector3d(entry["force"][0].as<double>(),
                                          entry["force"][1].as<double>(),
                                          entry["force"][2].as<double>());
      }
      table.emplace(thr_name, w);
    }

    if (!unknown_in_yaml.empty()) {
      std::ostringstream oss;
      oss << "loadTable: mode '" << mode_name
          << "' contains thruster names not in URDF: ";
      for (size_t i = 0; i < unknown_in_yaml.size(); ++i) {
        if (i) oss << ", ";
        oss << unknown_in_yaml[i];
      }
      throw std::runtime_error(oss.str());
    }

    table_map_.emplace(mode_name, std::move(table));
  }

  table_loaded_ = !table_map_.empty();
  current_mode_.clear();
  pinv_cache_.clear(); // rebuild below

#if 0
  // Pre-build pinv cache for all modes (policy)
  for (const auto & kv : table_map_) {
    const std::string & mode = kv.first;
    Eigen::Matrix<double, 6, Eigen::Dynamic> W = buildWForMode(mode);
    if (W.isZero(0)) {
      std::cerr << "[ThrusterMatrix] Warning: mode '" << mode
                << "' produces zero W (no effective actuators). Cache not created.\n";
      continue;
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu(W);
    const int rank = lu.rank();
    if (rank < std::min<int>(6, W.cols())) {
      std::cerr << "[ThrusterMatrix] Warning: mode '" << mode
                << "' is underactuated (rank=" << rank << "). Continuing with pinv cache.\n";
    }
    auto pinvW = pseudoInverse<Eigen::Matrix<double, 6, Eigen::Dynamic>,
                               Eigen::Matrix<double, Eigen::Dynamic, 6>>(W);
    pinv_cache_.emplace(mode, std::move(pinvW));
  }
#endif
}

void ThrusterMatrix::setThrusterTable(const std::string & table_name)
{
  if (!table_loaded_) {
    throw std::runtime_error("setThrusterTable: No table loaded.");
  }
  auto it = table_map_.find(table_name);
  if (it == table_map_.end()) {
    throw std::out_of_range("setThrusterTable: Table '" + table_name + "' not found.");
  }
  current_mode_ = table_name;

  // (Optionally) build cache on demand if missing
  if (pinv_cache_.find(current_mode_) == pinv_cache_.end()) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> W = buildWForMode(current_mode_);
    if (W.isZero(0)) {
      std::cerr << "[ThrusterMatrix] Warning: selected mode '" << current_mode_
                << "' has zero W. Commands will fail.\n";
    } else {
      pinv_cache_[current_mode_] =
        pseudoInverse<Eigen::Matrix<double, 6, Eigen::Dynamic>,
                      Eigen::Matrix<double, Eigen::Dynamic, 6>>(W);
    }
  }
}

void ThrusterMatrix::setBaseLink(const std::string & link_name)
{
  if (!model_urdf_) throw std::runtime_error("setBaseLink: URDF model not initialized.");
  urdf::ModelInterfaceSharedPtr model = model_urdf_;

  auto it = model->links_.find(link_name);
  if (it == model->links_.end()) {
    throw std::out_of_range("setBaseLink: link '" + link_name + "' not found in URDF.");
  }

  base_link_name_ = link_name;


  // Rebuild transforms relative to this link as new base
  std::map<std::string, Eigen::Isometry3d> T_map_link;
  std::map<std::string, Eigen::Isometry3d> T_map_joint;
  buildTransformsDFS(model, it->second, Eigen::Isometry3d::Identity(), T_map_link, T_map_joint);

  // Update thruster positions/orientations following the same ordering
  const auto names = orderedThrusterNames(model, urdfUtils);
  if (names.size() != n_thruster) {
    // Should not happen unless URDF changed at runtime
    std::cerr << "[ThrusterMatrix] Warning: thruster count changed after setBaseLink.\n";
  }
  printf("before thruster_position %zu x %zu\n", thruster_position.rows(),thruster_position.cols());

  thruster_position.resize(3, n_thruster);
  thruster_orientation.resize(3, n_thruster);
  printf("after thruster_position %zu x %zu\n", thruster_position.rows(),thruster_position.cols());

  std::size_t idx = 0;
  for (const auto & kv : model->joints_) {
    const auto & j = kv.second;
    if (!urdfUtils.isThruster(j->child_link_name))   continue;
    

    auto jt = T_map_joint.find(j->name);
    if (jt != T_map_joint.end()) {
      thruster_position.col(idx) = jt->second.translation();
      thruster_orientation.col(idx) = (jt->second.linear() *
        Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z).normalized()).normalized();
    } else {
      // Fallback
      thruster_position.col(idx) << j->parent_to_joint_origin_transform.position.x,
        j->parent_to_joint_origin_transform.position.y,
        j->parent_to_joint_origin_transform.position.z;
      const auto Tpj = poseToIso(j->parent_to_joint_origin_transform);
      thruster_orientation.col(idx) = (Tpj.linear() *
        Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z).normalized()).normalized();
    }

    ++idx;
  }

  // Rebuild allocation_mat and inverse
  printf("before allocation_mat %zu x %zu\n", allocation_mat.rows(),allocation_mat.cols());
  allocation_mat.resize(3, n_thruster);
  printf("after allocation_mat %zu x %zu\n", allocation_mat.rows(),allocation_mat.cols());
  for (std::size_t i = 0; i < n_thruster; ++i) {
    const Eigen::Vector3d r = thruster_position.col(i) - cgPos;
    const Eigen::Vector3d f = -thruster_orientation.col(i);
    allocation_mat.col(i) = r.cross(f);
  }
  inverse_allocation_mat = pseudoInverse<
    Eigen::Matrix<double, 3, Eigen::Dynamic>,
    Eigen::Matrix<double, Eigen::Dynamic, 3>>(allocation_mat);

  // Update urdf_thrusters_ geometry
  for (std::size_t i = 0; i < std::min(n_thruster, names.size()); ++i) {
    auto itc = urdf_thrusters_.find(names[i]);
    if (itc != urdf_thrusters_.end()) {
      itc->second.position  = thruster_position.col(i);
      itc->second.direction = thruster_orientation.col(i);
    }
  }
  pinv_cache_.clear();   // changing base
  if (table_loaded_) {
    for (const auto & kv : table_map_) {
      const std::string & mode = kv.first;
      Eigen::Matrix<double, 6, Eigen::Dynamic> W = buildWForMode(mode);
      if (!W.isZero(0)) {
        pinv_cache_[mode] = pseudoInverse<
          Eigen::Matrix<double, 6, Eigen::Dynamic>,
          Eigen::Matrix<double, Eigen::Dynamic, 6>>(W);
      }
    }
  }
}

std::vector<std::string> ThrusterMatrix::getAllThrusterNames() const
{
  std::vector<std::string> names;
  names.reserve(urdf_thrusters_.size());
  for (const auto & kv : urdf_thrusters_) names.push_back(kv.first);
  std::sort(names.begin(), names.end());
  return names;
}


ThrusterConfigVec ThrusterMatrix::getActiveThrusters() const
{
  ThrusterConfigVec out;
  if (current_mode_.empty()) return out;
  auto it = table_map_.find(current_mode_);
  if (it == table_map_.end()) return out;

  const auto & tbl = it->second;
  out.reserve(tbl.size());
  // Keep URDF map iteration stable: follow orderedThrusterNames
  const auto names = orderedThrusterNames(model_urdf_, urdfUtils);
  for (const auto & name : names) {
    if (tbl.find(name) != tbl.end()) {
      auto itc = urdf_thrusters_.find(name);
      if (itc != urdf_thrusters_.end()) {
        ThrusterConfig c = itc->second;
        c.active = true;
        out.push_back(c);
      }
    }
  }
  return out;
}

std::size_t ThrusterMatrix::getNumThr()
{
  return this->n_thruster;
}

bool ThrusterMatrix::isReady()
{
  if (inverse_allocation_mat.rows() > 0 && inverse_allocation_mat.cols() > 0) {
    return true;
  }
  return false;
}

void ThrusterMatrix::bodyToThruster(
  const Eigen::Vector3d & body_wrench_moment,
  Eigen::VectorXd & thruster_force) const
{
  // std::cout << "--------------------------------------------------------------------------------------------------------" << std::endl;
  // std::cout << "inverse_allocation_mat: "
  //         << inverse_allocation_mat.rows() << "×" << inverse_allocation_mat.cols()
  //         << ", body_wrench_moment: "
  //         << body_wrench_moment.rows()      << "×" << body_wrench_moment.cols()
  //         << std::endl;

  thruster_force = inverse_allocation_mat * body_wrench_moment;
}

void ThrusterMatrix::thrusterToBody(
  const Eigen::VectorXd & recv_thruster_force,
  Eigen::Vector3d & body_force) const
{
  if (recv_thruster_force.size() != static_cast<int>(n_thruster)) {
    throw std::runtime_error("thrusterToBody: size mismatch with number of thrusters.");
  }
  body_force = allocation_mat * recv_thruster_force;
}

//
// YAML-based thrust table path
//

void ThrusterMatrix::loadThrusterTableFromYaml(const std::string & yaml_path)
{
 if (!model_urdf_) throw std::runtime_error("loadThrusterTableFromYaml: URDF must be initialized first.");
  loadTable(yaml_path);
}
  // Build

void ThrusterMatrix::generateCommandFromTable(
  const Eigen::VectorXd & desired_wrench,
  Eigen::VectorXd & thruster_output) const
{
  if (!table_loaded_ || current_mode_.empty()) {
    throw std::runtime_error("generateCommandFromTable: No active table selected.");
  }
  if (!model_urdf_) {
    throw std::runtime_error("generateCommandFromTable: URDF not initialized.");
  }
  if (desired_wrench.size() != 6) {
    throw std::runtime_error("generateCommandFromTable: desired_wrench must be 6x1 [Tx Ty Tz Fx Fy Fz].");
  }

  // Get or build pinv(W) for current mode
  Eigen::Matrix<double, Eigen::Dynamic, 6> pinvW;
  {
    auto it = pinv_cache_.find(current_mode_);
    if (it != pinv_cache_.end()) {
      pinvW = it->second;
    } else {
      // Fallback: build once here if YAML load skipped this (e.g. under special flow)
      Eigen::Matrix<double, 6, Eigen::Dynamic> W = buildWForMode(current_mode_);
      if (W.isZero(0)) {
        throw std::runtime_error(
          "generateCommandFromTable: active table produces zero matrix W (no actuators).");
      }
      // Rank-deficient OK
      pinvW = pseudoInverse<Eigen::Matrix<double, 6, Eigen::Dynamic>,
                            Eigen::Matrix<double, Eigen::Dynamic, 6>>(W);
      // cache for next time
      const_cast<ThrusterMatrix*>(this)->pinv_cache_[current_mode_] = pinvW;
    }
  }

  // Unconstrained least squares
  Eigen::VectorXd u = pinvW * desired_wrench;

  // Non-negativity (simple one-shot active-set refinement)
  constexpr double kNegTol = 1e-12;
  bool has_neg = false;
  for (Eigen::Index i = 0; i < u.size(); ++i) {
    if (u(i) < -kNegTol) { has_neg = true; break; }
  }
  if (!has_neg) {
    for (Eigen::Index i = 0; i < u.size(); ++i) if (u(i) < 0.0) u(i) = 0.0;
    thruster_output = std::move(u);
    return;
  }

  // Re-solve with active (non-negative) subset
  std::vector<int> active_idx; active_idx.reserve(static_cast<size_t>(u.size()));
  for (int i = 0; i < u.size(); ++i) if (u(i) >= 0.0) active_idx.push_back(i);

  if (active_idx.empty()) {
    thruster_output = Eigen::VectorXd::Zero(u.size()); // safe fallback
    return;
  }

  // Rebuild W and select columns
  Eigen::Matrix<double, 6, Eigen::Dynamic> W = buildWForMode(current_mode_);
  Eigen::Matrix<double, 6, Eigen::Dynamic> W_active(6, static_cast<int>(active_idx.size()));
  for (int c = 0; c < static_cast<int>(active_idx.size()); ++c) {
    W_active.col(c) = W.col(active_idx[static_cast<size_t>(c)]);
  }
  Eigen::Matrix<double, Eigen::Dynamic, 6> pinvW_active =
    pseudoInverse<Eigen::Matrix<double, 6, Eigen::Dynamic>,
                  Eigen::Matrix<double, Eigen::Dynamic, 6>>(W_active);
  Eigen::VectorXd u_active = pinvW_active * desired_wrench;
  // Compose final vector with non-negative entries
  Eigen::VectorXd u_nn = Eigen::VectorXd::Zero(u.size());
  for (int c = 0; c < static_cast<int>(active_idx.size()); ++c) {
    u_nn(active_idx[static_cast<size_t>(c)]) = std::max(0.0, u_active(c));
  }
  thruster_output = std::move(u_nn);
}



// --------------------------------------------------------------------------
// Build W for a mode, following thruster_order_
// --------------------------------------------------------------------------

Eigen::Matrix<double, 6, Eigen::Dynamic>
ThrusterMatrix::buildWForMode(const std::string & mode_name) const
{
  const auto it_mode = table_map_.find(mode_name);
  if (it_mode == table_map_.end()) {
    throw std::out_of_range("buildWForMode: mode '" + mode_name + "' not found.");
  }
  const ThrusterTable & table = it_mode->second;

  Eigen::Matrix<double, 6, Eigen::Dynamic> W(6, static_cast<int>(n_thruster));
  W.setZero();
  if (!model_urdf_ || thruster_order_.empty()) return W;

  int col = 0;
  for (const auto & thr_name : thruster_order_) {
    auto itw = table.find(thr_name);
    if (itw != table.end()) {
      const ThrusterWeight & w = itw->second;
      W.block<3,1>(0, col) = w.torque_weight;
      W.block<3,1>(3, col) = w.force_weight;
    }
    ++col;
    if (col >= static_cast<int>(n_thruster)) break;
  }

  return W;
}

