#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from copy import copy


# --- FUNCIONES MATEMÁTICAS ORIGINALES (Sin cambios) ---
cos=np.cos; sin=np.sin; pi=np.pi

def dh(d, theta, a, alpha):
    cth = cos(theta); sth = sin(theta)
    ca = cos(alpha); sa = sin(alpha)
    Tdh = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                    [sth,  ca*cth, -sa*cth, a*sth],
                    [0,        sa,      ca,      d],
                    [0,         0,      0,      1]])
    return Tdh

def fkine(q):
    T1 = dh(0.1215,q[0],0,0)
    T2 = dh(0.405+q[1],0,0.402,0)
    T3 = dh(0.023,-pi/2+q[2],0.3,0)
    T4 = dh(0.0866,pi/2+q[3],0,pi/2)
    T5 = dh(0,q[4],0.1899,0)
    T = T1@T2@T3@T4
    return T

def jacobian(q, delta=0.0001):
    n = q.size
    J = np.zeros((3,n))
    T = fkine(q)
    for i in range(n):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine(dq)
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J

def ikine(xdes, q0):
    epsilon  = 0.001
    max_iter = 1000
    q  = copy(q0)
    for i in range(max_iter):
        T=fkine(q)
        x_actual=T[0:3,3]
        error=xdes-x_actual
        if np.linalg.norm(error)<epsilon:
            break
        J=jacobian(q)
        q=q+np.dot(np.linalg.pinv(J), error)      
    return q

def ik_gradient(xdes, q0):
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
    q  = copy(q0)
    for i in range(max_iter):
        T=fkine(q)
        x_actual=T[0:3,3]
        error=xdes-x_actual
        if np.linalg.norm(error) < epsilon:
            break
        J=jacobian(q)
        q=q+delta*np.dot(J.T,error)  
    return q

def rot2quat(R):
    dEpsilon = 1e-6
    quat = 4*[0.,]
    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)
    return np.array(quat)

def TF2xyzquat(T):
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


# --- NODO ROS 2 ---



def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()
    
    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
