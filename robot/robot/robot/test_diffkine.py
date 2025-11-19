#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from copy import copy

# Importamos tus módulos (asegúrate de que los archivos se llamen así)
from .markers import BallMarker, color
from .robot_functions import fkine, jacobian

class KinematicControlNode(Node):
    def __init__(self):
        super().__init__('test_kinematic_control_position')
        
        self.get_logger().info('Starting motion...')

        # Publisher: publish to the joint_states topic
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # Inicializar marcadores (Pasamos 'self' como el nodo)
        self.bmarker_current = BallMarker(self, color['RED'])
        self.bmarker_desired = BallMarker(self, color['GREEN'])

        # Nombres de las articulaciones
        self.jnames = ['joint1', 'joint2', 'joint3','joint4','joint5']

        # Configuración Inicial
        self.xd = np.array([0.02, 0.5, 0.4]) # Posición deseada
        self.q = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # q actual

        # Visualización inicial
        T = fkine(self.q)
        x0 = T[0:3, 3]
        self.bmarker_current.xyz(x0)
        self.bmarker_desired.xyz(self.xd)

        # Definición de la frecuencia y periodo
        self.freq = 50.0
        self.dt = 1.0 / self.freq
        
        # Límites de articulaciones
        self.limits = {
            0: (-6.28, 6.28),   # Joint 2
            1: (-1, 1), # Joint 3
            2: (-3.14, 3.14),   # Joint 4
            3: (-3.14, 3.14),      # Joint 5
            4: (-1.57, 1.57),  # Joint 6
            # Puedes agregar joint 7 y 8 si es necesario según tu lógica original
        }

        # Crear Timer (reemplaza al while loop y rate.sleep)
        self.timer = self.create_timer(self.dt, self.control_loop)

    def control_loop(self):
        # 1. Configurar mensaje JointState
        jstate = JointState()
        jstate.header.stamp = self.get_clock().now().to_msg()
        jstate.name = self.jnames

        # 2. Cinemática Directa Actual
        T = fkine(self.q)
        x = T[0:3, 3]
        self.get_logger().info(f"Pose actual: x={x[0]:.3f}, y={x[1]:.3f}, z={x[2]:.3f}")

        # 3. Cálculo del Error
        e = self.xd - x
        
        # 4. Verificar convergencia
        if np.linalg.norm(e) < 0.01:
            self.get_logger().info('Objetivo alcanzado. Deteniendo movimiento.')
            # Opcional: Cancelar el timer para dejar de calcular, 
            # o seguir publicando la última posición para mantener al robot ahí.
            # self.timer.cancel() 
            
            # Publicamos la última posición para que RViz no pierda el estado
            jstate.position = self.q.tolist()
            self.pub.publish(jstate)
            self.timer.cancel()
            return

        # 5. Ley de Control (q_dot = J_pinv * x_dot)
        k = 1.0
        x_dot = k * e # Velocidad operacional deseada
        
        J = jacobian(self.q)
        J_pseudo_inverse = np.linalg.pinv(J)
        
        q_dot = np.dot(J_pseudo_inverse, x_dot)
        
        # 6. Integración (Euler)
        self.q = self.q + q_dot * self.dt

        # 7. Saturación (Joint Limits)
        # Nota: q tiene indices 0-7. Tus limites usan indices 1-5 basados en tu codigo
        for idx, (min_val, max_val) in self.limits.items():
            self.q[idx] = max(min(self.q[idx], max_val), min_val)

        # 8. Publicar y Visualizar
        jstate.position = self.q.tolist()
        self.pub.publish(jstate)
        
        self.bmarker_desired.xyz(self.xd)
        self.bmarker_current.xyz(x)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('ending motion ...')

if __name__ == '__main__':
    main()
