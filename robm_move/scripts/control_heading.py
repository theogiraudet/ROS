#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""robm_move/control_heading.py - Commande proportionnele du cap

Ce noeud permet de contrôler l'orientation du robot.
Il met en oeuvre une commande proportionnelle en vitesse de rotation.

S'abonne à :
    - move_base_simple/goal (geometry_msgs/PoseStamped): but (seul le cap 
        est pris en compte)
    - odometry (nav_msgs/Odometry): pose du robot calculée par odométrie
Publie :
    - cmd_vel (geometry_msgs/Twist): commande en vitesse
"""

import rospy
from math import pi, cos, sin, atan2, sqrt

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def yaw_from_quaternion_msg( q ):
    """Extract the yaw angle from a geometry_msgs/Quaternion message"""
    
    quat_array = [q.x, q.y, q.z, q.w]
    yaw = euler_from_quaternion(quat_array)[2]
    return yaw


def sawtooth( angle ):
    """Wrap an angle value within the interval [-pi; +pi["""
    return (angle + pi) % (2*pi) - pi


class ControlHeading:
    def __init__(self):
        """Constructeur de la classe ControlHeading"""
        
        # cap désiré
        self.theta_d = None

        # subscribers et publishers
        self._sub_goal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_callback)
        self._sub_odom = rospy.Subscriber("odometry", Odometry, self.odom_callback)
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # compteur pour arrêter l'envoi de commandes après arrêt du robot
        self._nb_successive_zeros = 0
    
    
    def publish_speed_cmd(self, v, w):
        """Publie la commande de vitesse lineaire v et angulaire w
            
        Arrête de publier des commandes d'arrêt si le robot est déjà à 
        l'arrêt depuis un certain temps, pour éviter de fatiguer les 
        moteurs. Les vitesses sont en unités arbitraires, entre -1 et 1
            
        :param v: commande de vitesse linéaire (entre -1 et 1)
        :param w: commande de vitesse angulaire (entre -1 et 1)
        """
        # Mise à jour du compteur de commandes d'arrêt
        if v == 0 and w == 0:
            self._nb_successive_zeros += 1
        else:
            self._nb_successive_zeros = 0
            
        # Si nécessaire, on publie la commande
        if self._nb_successive_zeros < 5: 
            vel_msg = Twist()
            vel_msg.linear.x = v
            vel_msg.angular.z = w
            self._cmd_pub.publish(vel_msg)	


    def goal_callback(self, goal):
        """Callback du mise à jour du cap désiré depuis le but défini dans Rviz"""
        
        # Extract goal orientation from msg
        self.theta_d = yaw_from_quaternion_msg(goal.pose.orientation)
        rospy.loginfo("Goal set to yaw=%s" % self.theta_d)


    def odom_callback(self, odom):
        """Callback de reception de message d'odometrie.
        Le calcul de la commande proprtionnelle est effectué dans 
        cette fonction.
        """
        
        # Ne fait rien si le cap désiré n'est pas encore défini
        if self.theta_d == None:
            return
        
        # Récupère l'angle de cap courant du robot depuis l'odométrie
        theta = yaw_from_quaternion_msg(odom.pose.pose.orientation)

        # TODO: Calculer l'erreur de cap
        
        
        # TODO: (dans un second temps)
        # Utiliser la fonction dent de scie (sawtooth) pour calculer
        # la valeur principale de l'erreur de cap. Comparer le comportement avec
        # et sans cette amélioration.
        
        
        # TODO: Calculer la vitesse de rotation à appliquer
        # - Commande proportionnelle à l'erreur
        # - Arrêt si suffisamment proche du but
        w = 0.0 # TODO
        
        
        # Envoi de la commande sur le topic ROS
        self.publish_speed_cmd(0, w)


def control_heading_node():
    # Initialize ROS node
    rospy.init_node('control_heading')
    # Create a ControlHeading object
    control_heading = ControlHeading()
    # Process incoming ROS messages until termination
    rospy.spin()

    
if __name__ == '__main__':
    try:
        control_heading_node()
    except rospy.ROSInterruptException:
        pass
