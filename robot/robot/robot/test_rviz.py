#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

# Importamos tus librerías personalizadas
# Asegúrate de que markers.py y robot_functions.py estén accesibles
from .markers import BallMarker, FrameMarker, color
from .robot_functions import fkine, TF2xyzquat

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('test_forward_kinematics')
        
        # Publisher: JointState
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Inicializar Markers
        # IMPORTANTE: Pasamos 'self' porque en la versión ROS 2 que hicimos,
        # las clases Marker necesitan el nodo para crear sus publishers.
        self.bmarker = BallMarker(self, color['GREEN'])
        self.marker = FrameMarker(self)
        
        # Configuración del Robot
        self.jnames = ['joint1', 'joint2', 'joint3','joint4', 'joint5']
        
        # Configuración Articular (q)
        self.q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # --- CÁLCULOS CINEMÁTICOS (Se ejecutan una vez al inicio) ---
        T = fkine(self.q)
        
        # Usamos el logger de ROS 2 en lugar de print
        self.get_logger().info(f"\nMatriz Homogénea T:\n{np.round(T, 3)}")
        
        # Configurar posición de la bola (posición final)
        self.bmarker.position(T)
        
        # Configurar el frame (posición + orientación)
        x0 = TF2xyzquat(T)
        self.marker.setPose(x0)

        # Preparar el mensaje JointState
        self.jstate = JointState()
        self.jstate.name = self.jnames
        self.jstate.position = self.q.tolist() # Convertir numpy a lista para el mensaje

        # Configurar el bucle de publicación (20 Hz -> 0.05 segundos)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        """Este método se ejecuta cíclicamente a 20Hz"""
        # Actualizar timestamp
        self.jstate.header.stamp = self.get_clock().now().to_msg()
        
        # Publicar mensaje de articulaciones
        self.pub.publish(self.jstate)
        
        # Publicar marcadores (RViz)
        self.marker.publish()
        self.bmarker.publish()


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
