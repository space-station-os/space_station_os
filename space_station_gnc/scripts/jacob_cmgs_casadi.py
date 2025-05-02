#!/usr/bin/env python3

from casadi import *
import casadi as ca
import numpy as np
import json
import math
import sys, os

config_path = sys.argv[1] if len(sys.argv) > 1 else "../config/config.json"
output_path = sys.argv[2] if len(sys.argv) > 1 else "../src"

with open(config_path) as f:
    config = json.load(f)

H = config["H"]
beta = math.radians(config["beta_degrees"])

output_filename = config["code_generation"]["output_file"]
use_cpp = config["code_generation"]["enable_cpp"]

delta_1, delta_2, delta_3, delta_4 = ca.MX.sym('delta_1',1), ca.MX.sym('delta_2',1), ca.MX.sym('delta_3',1), ca.MX.sym('delta_4',1)

delta = ca.horzcat(delta_1, delta_2, delta_3, delta_4)

r1 = ca.vertcat(-cos(beta)*sin(delta_1), cos(delta_1), sin(beta)*sin(delta_1))
r2 = ca.vertcat(-cos(delta_2), -cos(beta)*sin(delta_2), sin(beta)*sin(delta_2))
r3 = ca.vertcat(cos(beta)*sin(delta_3), -cos(delta_3), sin(beta)*sin(delta_3))
r4 = ca.vertcat(cos(delta_4), cos(beta)*sin(delta_4), sin(beta)*sin(delta_4))

h = H*(r1 + r2 + r3 + r4)
hFunc = Function('hFunc', [delta_1, delta_2, delta_3, delta_4], [h])

jacob = jacobian(h,delta)

pseudoInv_par = (jacob@jacob.T)
pseudoInv_par_inverse = inv(pseudoInv_par)

pseudoInv = jacob.T @ pseudoInv_par_inverse
pseudoInvFunc = Function('pseudoInvFunc', [delta_1, delta_2, delta_3, delta_4], [pseudoInv])

opts = dict(cpp=use_cpp)
C = CodeGenerator(output_filename, opts)
C.add(pseudoInvFunc)
C.add(hFunc)
os.chdir(output_path)
C.generate()
# pseudoInvFunc.generate('L_p_func.cpp', opts)