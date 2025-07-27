#include "space_station_gnc/thruster_matrix.hpp"
#include <iostream>
#include <stdexcept>

//
// URDFUtils
//

URDFUtils::URDFUtils() = default;

bool URDFUtils::isThruster(const std::string& name) const {
    return name.rfind("th", 0) == 0;
}

std::size_t URDFUtils::getNumAct(const urdf::ModelInterfaceSharedPtr& model) const {
    std::size_t count = 0;
    for (const auto& kv : model->joints_) {
        if (isThruster(kv.second->child_link_name))
            ++count;
    }
        std::cout << "=========================================================================================" << std::endl;
        std::cout << "Number of thrusters: " << count << std::endl;
    return count;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
URDFUtils::getThrPos(const urdf::ModelInterfaceSharedPtr& model) const {
    std::size_t n = getNumAct(model);
    Eigen::Matrix<double, 3, Eigen::Dynamic> pos(3, n);
    std::size_t idx = 0;
    for (const auto& kv : model->joints_) {
        auto j = kv.second;
        if (isThruster(j->child_link_name)) {
            pos.col(idx) << j->parent_to_joint_origin_transform.position.x,
                            j->parent_to_joint_origin_transform.position.y,
                            j->parent_to_joint_origin_transform.position.z;
            ++idx;
        }
    }
    return pos;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
URDFUtils::getThrOrient(const urdf::ModelInterfaceSharedPtr& model) const {
    std::size_t n = getNumAct(model);
    Eigen::Matrix<double, 3, Eigen::Dynamic> orient(3, n);
    std::size_t idx = 0;
    for (const auto& kv : model->joints_) {
        auto j = kv.second;
        if (isThruster(j->child_link_name)) {
            orient.col(idx) << j->axis.x, j->axis.y, j->axis.z;
            ++idx;
        }
    }
    return orient;
}

void URDFUtils::printLinks(
    const std::map<std::string, std::shared_ptr<urdf::Link>>& links) const
{
    for (const auto& kv : links) {
        std::cout << "Link name: " << kv.first << "\n";
    }
}

//
// ThrusterMatrix
//

ThrusterMatrix::ThrusterMatrix() = default;

void ThrusterMatrix::initialize(const std::string& urdf_xml) {
    model = urdf::parseURDF(urdf_xml);
    if (!model)
        throw std::runtime_error("Failed to parse URDF XML");

    n_thruster = urdfUtils.getNumAct(model);
    thruster_position = urdfUtils.getThrPos(model);
    thruster_orientation = urdfUtils.getThrOrient(model);
    allocation_mat.resize(3, n_thruster);
    inverse_allocation_mat.resize(n_thruster, 3);

    for (std::size_t i = 0; i < n_thruster; ++i)
        thruster_orientation.col(i).normalize();

    for (std::size_t i = 0; i < n_thruster; ++i) {
        Eigen::Vector3d r = thruster_position.col(i) - cgPos;
        Eigen::Vector3d f = -thruster_orientation.col(i);
        allocation_mat.col(i) = r.cross(f);
    }

    inverse_allocation_mat = pseudoInverse<Eigen::Matrix<double, 3, Eigen::Dynamic>, Eigen::Matrix<double, Eigen::Dynamic, 3>>(allocation_mat);

    std::cout << inverse_allocation_mat.rows();


}

std::size_t ThrusterMatrix::getNumThr() {
    return this->n_thruster;
}

bool ThrusterMatrix::isReady() {
    if (inverse_allocation_mat.rows() > 0 && inverse_allocation_mat.cols() > 0) {
        return true;
    }
    return false;
}

void ThrusterMatrix::bodyToThruster(const Eigen::Vector3d& body_wrench_moment,
                                    Eigen::VectorXd& thruster_force) const
{
    // std::cout << "--------------------------------------------------------------------------------------------------------" << std::endl;
    // std::cout << "inverse_allocation_mat: "
    //         << inverse_allocation_mat.rows() << "×" << inverse_allocation_mat.cols()
    //         << ", body_wrench_moment: "
    //         << body_wrench_moment.rows()      << "×" << body_wrench_moment.cols()
    //         << std::endl;
    
    thruster_force = inverse_allocation_mat * body_wrench_moment;
}

void ThrusterMatrix::thrusterToBody(const Eigen::VectorXd& recv_thruster_force,
                                    Eigen::Vector3d& body_force) const
{
    body_force = allocation_mat * recv_thruster_force;
}
