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

#pragma once

#include <Eigen/Dense>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/StdVector>

#include <map>
#include <memory>
#include <string>
#include <limits>
#include <vector>
#include <utility>


/// @brief Utilities for extracting thruster-related information from URDF.
class URDFUtils
{
public:
  URDFUtils();

  bool isThruster(const std::string & name) const;

  /// @brief Count all thrusters (based on child link name rule)
  std::size_t getNumAct(const urdf::ModelInterfaceSharedPtr & model) const;

  /// @brief Thruster positions in the current base frame (3xN)
  Eigen::Matrix<double, 3,
    Eigen::Dynamic> getThrPos(const urdf::ModelInterfaceSharedPtr & model) const;

  /// @brief Thruster directions in the current base frame (3xN), normalized
  Eigen::Matrix<double, 3,
    Eigen::Dynamic> getThrOrient(const urdf::ModelInterfaceSharedPtr & model) const;


  void printLinks(const std::map<std::string, std::shared_ptr<urdf::Link>> & links) const;
};

/// @brief Thruster unified configuration after merging URDF, properties.yaml, and table.yaml

struct ThrusterConfig
{
  std::string name;
  Eigen::Vector3d position;     ///< URDF: position in body frame
  Eigen::Vector3d direction;    ///< URDF: direction vector
  double max_force = 100.0;       ///< properties.yaml: [N]
  double isp = 0.0;             ///< properties.yaml: [s]
  double efficiency = 1.0;      ///< properties.yaml: [-]
  bool active = false;          ///< table.yaml: mode-dependent activation

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Forward declarations
using ThrusterConfigVec = std::vector<
  ThrusterConfig,
  Eigen::aligned_allocator<ThrusterConfig>>;

using PinvMat = Eigen::Matrix<double, Eigen::Dynamic, 6>;
using PinvCacheMap = std::map<
  std::string,
  PinvMat,
  std::less<>,
  Eigen::aligned_allocator<std::pair<const std::string, PinvMat>>>;

using ThrusterCfgMap = std::map<
  std::string,
  ThrusterConfig,
  std::less<>,
  Eigen::aligned_allocator<std::pair<const std::string, ThrusterConfig>>>;


/**
 * @brief YAML-defined per-thruster weights selecting contribution to wrench axes.
 * torque_weight corresponds to [Tx, Ty, Tz], force_weight to [Fx, Fy, Fz].
 */
struct ThrusterWeight
{
  Eigen::Vector3d torque_weight = Eigen::Vector3d::Zero();   // [Tx, Ty, Tz]
  Eigen::Vector3d force_weight = Eigen::Vector3d::Zero();    // [Fx, Fy, Fz]
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using ThrusterTable = std::map<
  std::string,
  ThrusterWeight,
  std::less<>,
  Eigen::aligned_allocator<std::pair<const std::string, ThrusterWeight>>
>;

//using ThrusterTable = std::map<std::string, ThrusterWeight>;     // map<thruster_name, weights> for one mode
using ThrusterTableSet = std::map<std::string, ThrusterTable>;   // map<mode_name, table>

/**
 * @brief Allocation helper combining URDF geometry and optional YAML tables.
 *
 *  - Legacy path:
 *      bodyToThruster / thrusterToBody use URDF-only allocation (3xN torque mapping).
 *  - Table path:
 *      loadProperties() + loadTable() + setThrusterTable() + generateCommandFromTable()
 *      enables mode-dependent selection/weighting over the same URDF geometry.
 *
 * Design contracts decided in this project:
 *  - URDF: geometry only (pose/direction/existence).
 *  - properties.yaml: ratings (max_force, isp, efficiency) per thruster.
 *  - table.yaml: operational modes (per-mode active set and optional weights).
 *  - The single source of truth in runtime is the merged ThrusterConfig set.
*/
class ThrusterMatrix
{
public:
  ThrusterMatrix();

  /// @brief Initialize merged thruster configuration
  ///        Combine URDF, properties.yaml, and table.yaml
  void initialize(const std::string & urdf_xml);

  /// @brief Load URDF-defined thruster geometry
  void loadURDF(const urdf::ModelInterfaceSharedPtr & model);

  /// @brief Load thruster properties from properties.yaml
  void loadProperties(const std::string & yaml_file);

  /// @brief Load thruster table (modes) from table.yaml
  void loadTable(const std::string & yaml_file);

  /// @brief Select current mode (table) by name. Throws if not found.
  void setThrusterTable(const std::string & table_name);

  /// @brief Change base link frame and rebuild geometry and caches.
  /// @note Changing base invalidates geometry-dependent caches (e.g., pinv(W)).
  void setBaseLink(const std::string & link_name);

  /// @brief Current base link name (empty means URDF root)
  std::string base_link_name_{};


  /// @brief Get full thruster list (URDF-defined, regardless of active state)
  std::vector<std::string> getAllThrusterNames() const;

  /// @brief Get active thruster configs (for current mode)
  //std::vector<ThrusterConfig> getActiveThrusters() const;
  ThrusterConfigVec getActiveThrusters() const;

  std::size_t getNumThr();

  bool isReady();

  // --------------------------------------------------------------------------
  // Table path (6xN W * u = desired_wrench)
  // --------------------------------------------------------------------------

  /**
   * @brief Generate per-thruster command vector from desired 6x1 wrench
   *        using the currently selected mode table.
   * @param desired_wrench [Tx Ty Tz Fx Fy Fz]^T
   * @param thruster_output output size N (resized inside)
   *
   * Policy (documented/decided):
   *  - If W becomes all zeros for the active mode, throw (no actuators effectively).
   *  - If the unconstrained solution contains negatives, apply one-shot active-set
   *    refinement and clamp to non-negative (NNLS-lite). See .cpp for details.
   */
  void generateCommandFromTable(
    const Eigen::VectorXd & desired_wrench,
    Eigen::VectorXd & thruster_output) const;

  // --------------------------------------------------------------------------
  // Legacy path (URDF-only 3xN moment allocation)
  // --------------------------------------------------------------------------

  /// @brief Map body moment (3x1) to thruster forces (Nx1) using inverse_allocation_mat.
  void bodyToThruster(
    const Eigen::Vector3d & body_wrench_moment,
    Eigen::VectorXd & thruster_force) const;

  /// @brief Map thruster forces (Nx1) back to body moment (3x1) using allocation_mat.
  void thrusterToBody(
    const Eigen::VectorXd & recv_thruster_force,
    Eigen::Vector3d & body_force) const;

  // --------------------------------------------------------------------------
  // Back-compat adapter (deprecated)
  // --------------------------------------------------------------------------

  /// @deprecated Use loadTable() instead. Kept for backward compatibility.
  void loadThrusterTableFromYaml(const std::string & yaml_path);

private:
/*
  template<typename _Matrix_Type_1_, typename _Matrix_Type_2_>
  _Matrix_Type_2_ pseudoInverse(
    const _Matrix_Type_1_ & a,
    double epsilon = std::numeric_limits<double>::epsilon()) const
  {
    Eigen::JacobiSVD<_Matrix_Type_1_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto & s = svd.singularValues();
    const double s_max = (s.size() > 0) ? s.maxCoeff() : 0.0;
    const double tol = epsilon * std::max(a.cols(), a.rows()) * s_max;
    //double tol = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
    //Eigen::ArrayXd inv = svd.singularValues().array();
    Eigen::ArrayXd inv = s.array();

    for (int i = 0; i < inv.size(); ++i) {
      inv(i) = (std::abs(inv(i)) > tol) ? 1.0 / inv(i) : 0.0;
    }
    return svd.matrixV() * inv.matrix().asDiagonal() * svd.matrixU().adjoint();
  }
*/
  template<typename MatA, typename MatPinv>
  MatPinv pseudoInverse(
    const MatA & a,
    double epsilon = std::numeric_limits<double>::epsilon()) const
  {
    using Index = Eigen::Index;

    // 0) Empty-guard
    if (a.rows() == 0 || a.cols() == 0) {
      MatPinv z(a.cols(), a.rows());  // pinv has shape (cols x rows)
      z.setZero();
      return z;
    }

    // 1) SVD
    Eigen::MatrixXd A = a;  //
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto & s = svd.singularValues();
    const double smax = (s.size() ? s.array().abs().maxCoeff() : 0.0);
    const double tol = epsilon * static_cast<double>(std::max<Index>(a.rows(), a.cols())) * smax;

    // 2) Build Σ^+ with tolerance
    Eigen::VectorXd sinv = s;
    for (Index i = 0; i < sinv.size(); ++i) {
      sinv(i) = (std::abs(s(i)) > tol) ? 1.0 / s(i) : 0.0;
    }

    // 3) Use only the first r columns where r = rank (== s.size() with Thin SVD)
    const Index r = sinv.size();
    if (r == 0) {
      MatPinv z(a.cols(), a.rows());
      z.setZero();
      return z;
    }

    // V is (cols x r), U is (rows x r) in Thin-SVD
    const auto V_r = svd.matrixV().leftCols(r);
    const auto U_r = svd.matrixU().leftCols(r);

    // 4) pinv = V_r * Σ^+ * U_r^T
    return V_r * sinv.asDiagonal() * U_r.adjoint();
  }


  // URDF-based structure
  URDFUtils urdfUtils;
  urdf::ModelInterfaceSharedPtr model_urdf_;

  // Stable column order for all matrices (derived from URDF traversal)
  std::vector<std::string> thruster_order_;

  // Geometry + merged attributes per thruster (key = thruster name)
  //std::map<std::string, ThrusterConfig> urdf_thrusters_;
  ThrusterCfgMap urdf_thrusters_;

  // Properties.yaml (ratings) by name
  //std::map<std::string, ThrusterProperties> properties_map_;

  // YAML-based table
  ThrusterTableSet table_map_;
  //ThrusterTable current_table_;
  std::string current_mode_;
  bool table_loaded_ = false;

  // Legacy allocation matrix
  Eigen::Vector3d cgPos{0, 0, 0};
  std::size_t n_thruster{0};
  Eigen::Matrix<double, 3, Eigen::Dynamic> thruster_position;
  Eigen::Matrix<double, 3, Eigen::Dynamic> thruster_orientation;
  double rating_thruster_force{100.0};
  Eigen::Matrix<double, 3, Eigen::Dynamic> allocation_mat;
  Eigen::Matrix<double, Eigen::Dynamic, 3> inverse_allocation_mat;
  //Eigen::Matrix<double, 3, 1> moment;


  // Cache: mode_name -> pinv(W_mode) where W_mode is 6xN for that mode
  //mutable std::map<std::string, Eigen::Matrix<double, Eigen::Dynamic, 6>> pinv_cache_;
  mutable PinvCacheMap pinv_cache_;

  // Helper: build 6xN W for a given mode, following the same joint/column order.
  Eigen::Matrix<double, 6, Eigen::Dynamic> buildWForMode(const std::string & mode_name) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
