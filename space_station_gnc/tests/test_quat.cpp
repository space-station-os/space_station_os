#include <iostream>
#include "L_p_func.cpp"
#include <Eigen/Dense>
#include <vector>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


Eigen::Matrix<double,4,3> pseudoinverse(Eigen::VectorXd delta) {
    Eigen::Matrix<double,4,3> pseudoinv(4,3);
    double out[12];
    
    const casadi_real* deltaI[4] = {&delta(0), &delta(1), &delta(2), &delta(3)};

    std::vector<casadi_real> Inv(12, 0.0);  

    casadi_real* resInv[1] = {Inv.data()};
    casadi_int iwInv[2] = {0,}; 
    casadi_real wInv[2] = {0,};
    int mem = 0;

    pseudoInvFunc(deltaI, resInv, iwInv, wInv, mem);

    casadi_real* invArr = &Inv[0];
    pseudoinv = Eigen::Map<Eigen::Matrix<double,4,3>>(invArr);
    std::cout << pseudoinv << std::endl;
    return pseudoinv;
}

int main() {
    Eigen::Vector4d delta;
    delta << 1, 1, 1, 1;
    pseudoinverse(delta);
    return 1;
}
