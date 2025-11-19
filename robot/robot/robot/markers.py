#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from visualization_msgs.msg import Marker
import numpy as np

class BallMarker(object):
    """
    Class to visualize ball markers in RViz for ROS 2
    """
    id = 0

    def __init__(self, node, color, alpha=1.0, scale=0.05):
        """
        Args:
            node: The ROS 2 node instance (self) used to create publishers.
            color: List [r,g,b] or dictionary value.
            alpha: Transparency.
            scale: Size of the ball.
        """
        self.node = node
        
        # Intentar obtener el parámetro del nodo, sino usar 'base_link' por defecto
        try:
            # Asumimos que el parámetro ya fue declarado en el nodo principal o usamos un default
            if self.node.has_parameter('reference_frame'):
                reference_frame = self.node.get_parameter('reference_frame').value
            else:
                reference_frame = 'base_link'
        except:
            reference_frame = 'base_link'

        self.marker_pub = self.node.create_publisher(Marker, "visualization_marker", 10)
        
        self.marker = Marker()
        self.marker.header.frame_id = reference_frame
        self.marker.ns = "ball_markers"
        self.marker.id = BallMarker.id
        BallMarker.id += 1
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale
        self.setColor(color, alpha)
        
        # Lifetime 0 significa infinito en ROS 2
        self.marker.lifetime = Duration(seconds=0).to_msg()

    def setColor(self, color, alpha=1.0):
        self.marker.color.r = float(color[0])
        self.marker.color.g = float(color[1])
        self.marker.color.b = float(color[2])
        self.marker.color.a = float(alpha)

    def position(self, T):
        """
        Set position (4x4 NumPy homogeneous matrix) for the ball and publish it
        """
        self.marker.pose.position.x = T[0,3]
        self.marker.pose.position.y = T[1,3]
        self.marker.pose.position.z = T[2,3]
        self.publish()

    def xyz(self, position):
        """
        Set position (list) for the ball and publish it
        """
        self.marker.pose.position.x = position[0]
        self.marker.pose.position.y = position[1]
        self.marker.pose.position.z = position[2]
        self.publish()

    def publish(self):
        # Actualizar el timestamp para que RViz no descarte el mensaje por ser "antiguo"
        self.marker.header.stamp = self.node.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)


"""
List for colors in BallMarker
"""
color = dict()
color['RED']       = (1.0, 0.0, 0.0)
color['GREEN']     = (0.0, 1.0, 0.0)
color['BLUE']      = (0.0, 0.0, 1.0)
color['YELLOW']    = (1.0, 1.0, 0.0)
color['PINK']      = (1.0, 0.0, 1.0)
color['CYAN']      = (0.0, 1.0, 1.0)
color['BLACK']     = (0.0, 0.0, 0.0)
color['DARKGRAY']  = (0.2, 0.2, 0.2)
color['LIGHTGRAY'] = (0.5, 0.5, 0.5)
color['WHITE']     = (1.0, 1.0, 1.0)


class FrameMarker(object):
    """
    Class to visualize frames as markers in RViz for ROS 2
    """
    id = 0

    def __init__(self, node, color_saturation=1.0, alpha=1.0, scale=0.1):
        """
        Args:
            node: The ROS 2 node instance.
        """
        self.node = node
        
        try:
            if self.node.has_parameter('reference_frame'):
                reference_frame = self.node.get_parameter('reference_frame').value
            else:
                reference_frame = 'base_link'
        except:
            reference_frame = 'base_link'

        self.marker_pub = self.node.create_publisher(Marker, "visualization_marker", 10)
        
        self.markerx = Marker()
        self.markery = Marker()
        self.markerz = Marker()
        
        for m in [self.markerx, self.markery, self.markerz]:
            m.header.frame_id = reference_frame
            m.ns = "frame_markers"
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.lifetime = Duration(seconds=0).to_msg()
            # Inicializar orientación identidad
            m.pose.orientation.w = 1.0 

        self.markerx.id = FrameMarker.id; FrameMarker.id += 1
        self.markery.id = FrameMarker.id; FrameMarker.id += 1
        self.markerz.id = FrameMarker.id; FrameMarker.id += 1

        # Setup inicial de posiciones y orientaciones base
        # X (Rojo)
        self.markerx.pose.orientation.w = 1.0
        
        # Y (Verde) - Rotado 90deg en Z
        self.markery.pose.orientation.w = np.cos(np.pi/4.0)
        self.markery.pose.orientation.z = np.sin(np.pi/4.0)
        
        # Z (Azul) - Rotado -90deg en Y
        self.markerz.pose.orientation.w = np.cos(-np.pi/4.0)
        self.markerz.pose.orientation.y = np.sin(-np.pi/4.0)

        # Escalas
        self.markerx.scale.x = scale; self.markerx.scale.y = 0.01; self.markerx.scale.z = 0.01
        self.markery.scale.x = scale; self.markery.scale.y = 0.01; self.markery.scale.z = 0.01
        self.markerz.scale.x = scale; self.markerz.scale.y = 0.01; self.markerz.scale.z = 0.01

        # Colores
        self.markerx.color.r = float(color_saturation); self.markerx.color.g = 0.0; self.markerx.color.b = 0.0; self.markerx.color.a = float(alpha)
        self.markery.color.r = 0.0; self.markery.color.g = float(color_saturation); self.markery.color.b = 0.0; self.markery.color.a = float(alpha)
        self.markerz.color.r = 0.0; self.markerz.color.g = 0.0; self.markerz.color.b = float(color_saturation); self.markerz.color.a = float(alpha)

    def setPose(self, pose):
        """
        Set the pose (7x1 NumPy matrix) for the frame and publish it.
        pose: [x, y, z, qw, qx, qy, qz]
        """
        # Actualizar posiciones
        for m in [self.markerx, self.markery, self.markerz]:
            m.pose.position.x = pose[0]
            m.pose.position.y = pose[1]
            m.pose.position.z = pose[2]

        if (len(pose)==7):
            # X is aligned with the pose quaternion
            self.markerx.pose.orientation.w = pose[3]
            self.markerx.pose.orientation.x = pose[4]
            self.markerx.pose.orientation.y = pose[5]
            self.markerx.pose.orientation.z = pose[6]

            # Y is rotated 90 wrt current Z (local rotation logic)
            q1 = np.array([np.cos(np.pi/4.0), 0., 0., np.sin(np.pi/4.0)])
            # q = quaternionMult(pose_quat, rotation)
            q = quaternionMult(pose[3:], q1)
            self.markery.pose.orientation.w = q[0]
            self.markery.pose.orientation.x = q[1]
            self.markery.pose.orientation.y = q[2]
            self.markery.pose.orientation.z = q[3]

            # Z is rotated -90 wrt current Y (local rotation logic)
            q1 = np.array([np.cos(-np.pi/4.0), 0., np.sin(-np.pi/4.0), 0.])
            q = quaternionMult(pose[3:], q1)
            self.markerz.pose.orientation.w = q[0]
            self.markerz.pose.orientation.x = q[1]
            self.markerz.pose.orientation.y = q[2]
            self.markerz.pose.orientation.z = q[3]

        self.publish()

    def publish(self):
        now = self.node.get_clock().now().to_msg()
        self.markerx.header.stamp = now
        self.markery.header.stamp = now
        self.markerz.header.stamp = now
        
        self.marker_pub.publish(self.markerx)
        self.marker_pub.publish(self.markery)
        self.marker_pub.publish(self.markerz)


def quaternionMult(q1, q2):
    """
    Multiplica dos cuaterniones.
    q1, q2: numpy array [w, x, y, z]
    """
    quat = 4*[0.,]
    quat[0] = -q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]+q1[0]*q2[0]
    quat[1] =  q1[0]*q2[1]-q1[3]*q2[2]+q1[2]*q2[3]+q1[1]*q2[0]
    quat[2] =  q1[3]*q2[1]+q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]
    quat[3] = -q1[2]*q2[1]+q1[1]*q2[2]+q1[0]*q2[3]+q1[3]*q2[0]
    return np.array(quat)


def vtotuple(v):
    return [val[0,0] for val in v]
