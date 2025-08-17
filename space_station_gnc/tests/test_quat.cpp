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

#include <iostream>
#include "L_p_func.cpp"
#include <Eigen/Dense>
#include <vector>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


Eigen::Matrix<double, 4, 3> pseudoinverse(Eigen::VectorXd delta)
{
  Eigen::Matrix<double, 4, 3> pseudoinv(4, 3);
  double out[12];

  const casadi_real * deltaI[4] = {&delta(0), &delta(1), &delta(2), &delta(3)};

  std::vector<casadi_real> Inv(12, 0.0);

  casadi_real * resInv[1] = {Inv.data()};
  casadi_int iwInv[2] = {0, };
  casadi_real wInv[2] = {0, };
  int mem = 0;

  pseudoInvFunc(deltaI, resInv, iwInv, wInv, mem);

  casadi_real * invArr = &Inv[0];
  pseudoinv = Eigen::Map<Eigen::Matrix<double, 4, 3>>(invArr);
  std::cout << pseudoinv << std::endl;
  return pseudoinv;
}

int main()
{
  Eigen::Vector4d delta;
  delta << 1, 1, 1, 1;
  pseudoinverse(delta);
  return 1;
}
