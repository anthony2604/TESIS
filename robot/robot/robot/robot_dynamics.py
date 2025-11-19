#!/usr/bin/env python3
import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi



def dh(d, theta, a, alpha):
    cth = cos(theta); sth = sin(theta)
    ca = cos(alpha); sa = sin(alpha)
    Tdh = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                    [sth,  ca*cth, -sa*cth, a*sth],
                    [0,        sa,      ca,     d],
                    [0,         0,       0,     1]])
    return Tdh
    
    
def fkine(q):
    T1 = dh(0.64786,q[0]-pi/2,0,pi/2)
    T2 = dh(0,pi/2-q[1],0.68572,0)
    T3 = dh(0,-pi/2-q[2],0.6,0)
    T4 = dh(0,pi/2-q[3],0,pi/2)
    T5 = dh(0.91615+q[4],pi,0,pi/2)
    T6 = dh(0,-q[5],0.504,0)
    T = T1@T2@T3@T4@T5@T6
    return T

# ... (resto de tus funciones: jacobian, ikine, rot2quat, etc.) ...
# ... (Pega todo tu script original aquí) ...
def TF2xyzquat(T):
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)
