#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from math import pi, cos, sin

from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


def quaternion_msg_from_yaw(yaw):
    """Crée un Quaternion à partir d'un angle de lacet (cap)
    
    Fonction utilitaire qui crée un message Quaternion correspondant à une rotation
    représentant l'orientation 'yaw' (cap du robot en radians)

    :param yaw: angle de cap en radians
    """
    q = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(*q)


class WheelOdometry:
    def __init__(self):
        """Constructeur de la classe WheelOdometry"""
        # Publisher ROS, pour publier le resultat d'odométrie sur le topic 'odometry'
        self._odom_pub = rospy.Publisher('odometry', Odometry, queue_size=10)
        
        # Subscriber : s'abonne au topic 'nxt/encoders' (positions des roues). 
        # La méthode encoders_callback sera appelée à chaque message reçu
        self._encoder_sub = rospy.Subscriber("nxt/encoders", JointState, self.encoders_callback)

        # Position et orientation initiales dans le repère odométrique (0,0,0)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Paramètres géométriques du robot
        self.r = 0.0166  # Rayon d'une roue
        self.L = 0.15    # Voie (distance entre deux roues d'un même essieu)

        # Mémorisation de l'ancienne position de chaque roue
        self._old_left_pos = None
        self._old_right_pos = None

    def encoders_callback(self, motors_state):
        """Callback pour traiter les informations de position des roues (codeurs)"""
        # Extrait les positions angulaires des roues depuis le message ROS reçu
        left_pos = motors_state.position[0]
        right_pos = motors_state.position[1]

        # TODO: Il n'est pas possible de calculer une différence entre deux positions la première fois
        if self._old_left_pos is None or self._old_right_pos is None:
            self.old_left_pos = left_pos
            self.old_right_pos = right_pos
            # TODO: Gestion du premier appel

        # TODO: Calcul du déplacement angulaire de chaque roue 
        # et mise à jour des positions mémorisées
        d_left  = left_pos - self.old_left_pos
        d_right = right_pos - self.old_right_pos
        self.old_left_pos = left_pos
        self.old_right_pos = right_pos

        # TODO: Calcul du déplacement du robot
        ds     = self.r * (d_left + d_right) / 2
        dtheta = self.r * (d_left - d_right) / self.L

        # TODO: Mise à jour de la position estimée du robot
        self.x     = self.x  + ds * cos(self.theta)
        self.y     = self.y  + ds * cos(self.theta)
        self.theta = self.theta + dtheta

        
        #rospy.loginfo("x=%s y=%s theta=%s", self.x, self.y, self.theta)
        
        # TODO: Crée et publie le message d'odométrie
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x  = 0.0  #TODO
        msg.pose.pose.position.y  = 0.0  #TODO
        msg.pose.pose.orientation = Quaternion()  #TODO
        self._odom_pub.publish(msg)
    

def odometry_node():
    # Initialise le noeud ROS
    rospy.init_node('odometry')
    # Instancie un objet WheelOdometry
    wheel_odometry = WheelOdometry()
    # Boucle en traitant tous les messages ROS jusqu'à terminaison du programme
    rospy.spin()
    
if __name__ == '__main__':
    try:
        odometry_node()
    except rospy.ROSInterruptException:
        pass
