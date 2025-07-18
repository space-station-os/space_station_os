#pragma once

#include <Eigen/Dense>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <map>
#include <memory>
#include <string>
#include <limits>

class URDFUtils {
public:
    URDFUtils();

    bool isThruster(const std::string& name) const;
    std::size_t getNumAct(const urdf::ModelInterfaceSharedPtr& model) const;
    Eigen::Matrix<double, 3, Eigen::Dynamic> getThrPos(const urdf::ModelInterfaceSharedPtr& model) const;
    Eigen::Matrix<double, 3, Eigen::Dynamic> getThrOrient(const urdf::ModelInterfaceSharedPtr& model) const;
    void printLinks(const std::map<std::string, std::shared_ptr<urdf::Link>>& links) const;
};

class ThrusterMatrix {
public:
    ThrusterMatrix();

    void initialize(const std::string& urdf_xml);

    // map body wrench to individual thruster forces
    void bodyToThruster(const Eigen::Vector3d& body_wrench_moment,
                        Eigen::VectorXd& thruster_force) const;

    // map thruster forces to body force
    void thrusterToBody(const Eigen::VectorXd& recv_thruster_force,
                        Eigen::Vector3d& body_force) const;

    std::size_t getNumThr();

    bool isReady();

private:
    template<typename _Matrix_Type_1_, typename _Matrix_Type_2_>
    _Matrix_Type_2_ pseudoInverse(const _Matrix_Type_1_& a,
                                double epsilon = std::numeric_limits<double>::epsilon()) const
    {
        Eigen::JacobiSVD<_Matrix_Type_1_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tol = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
        Eigen::ArrayXd inv = svd.singularValues().array();
        for (int i = 0; i < inv.size(); ++i) {
            inv(i) = (std::abs(inv(i)) > tol) ? 1.0 / inv(i) : 0.0;
        }
        return svd.matrixV() * inv.matrix().asDiagonal() * svd.matrixU().adjoint();
    }

    URDFUtils urdfUtils;
    urdf::ModelInterfaceSharedPtr model;
    Eigen::Vector3d cgPos{0, 0, 0};
    std::size_t n_thruster{0};
    Eigen::Matrix<double, 3, Eigen::Dynamic> thruster_position;
    Eigen::Matrix<double, 3, Eigen::Dynamic> thruster_orientation;
    double rating_thruster_force{100.0};
    Eigen::Matrix<double, 3, Eigen::Dynamic> allocation_mat;
    Eigen::Matrix<double, Eigen::Dynamic, 3> inverse_allocation_mat;
    Eigen::Matrix<double, 3, 1> moment;
};
