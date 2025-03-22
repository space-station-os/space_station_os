from casadi import *
import casadi as ca
import numpy as np

H = 10
beta = 54.73*pi/180
delta_1, delta_2, delta_3, delta_4 = ca.MX.sym('delta_1',1), ca.MX.sym('delta_2',1), ca.MX.sym('delta_3',1), ca.MX.sym('delta_4',1)

delta = ca.horzcat(delta_1, delta_2, delta_3, delta_4)

r1 = ca.vertcat(-cos(beta)*sin(delta_1), cos(delta_1), sin(beta)*sin(delta_1))
r2 = ca.vertcat(-cos(delta_2), -cos(beta)*sin(delta_2), sin(beta)*sin(delta_2))
r3 = ca.vertcat(cos(beta)*sin(delta_3), -cos(delta_3), sin(beta)*sin(delta_3))
r4 = ca.vertcat(cos(delta_4), cos(beta)*sin(delta_4), sin(beta)*sin(delta_4))

h = 10*(r1 + r2 + r3 + r4)
hFunc = Function('hFunc', [delta_1, delta_2, delta_3, delta_4], [h])

jacob = jacobian(h,delta)

pseudoInv_par = (jacob@jacob.T)
pseudoInv_par_inverse = inv(pseudoInv_par)

pseudoInv = jacob.T @ pseudoInv_par_inverse
pseudoInvFunc = Function('pseudoInvFunc', [delta_1, delta_2, delta_3, delta_4], [pseudoInv])

opts = dict(cpp=True)
C = CodeGenerator('L_p_func.cpp', opts)
C.add(pseudoInvFunc)
C.add(hFunc)
C.generate()
# pseudoInvFunc.generate('L_p_func.cpp', opts)