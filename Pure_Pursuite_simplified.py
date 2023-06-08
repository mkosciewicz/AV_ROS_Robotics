#!/usr/bin/env python

from __future__ import division
import time
import rospy
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Parameters
th = 0.005482709692924
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
prev_angular_z = 0  # Used for rate limiting
max_rate = 0.1  # Maksymalna szybkosc zmiany prędkosci kątowej

def calculate_c(a, b):
    return (a * b * np.sin(th)) / np.sqrt(a ** 2 + b ** 2 - 2 * a * b * np.cos(th))

def calculate_sr(list):
    return np.median(list) if len(list) > 0 else 0


# A simple moving average filter for path smoothing
def smooth_path(path, window_size=5):
    return np.convolve(path, np.ones(window_size), 'valid') / window_size if len(path) > 0 else []


def control_angular_velocity(sr_prawo, sr_lewo, sr_przod):
    global prev_angular_z
    twist = Twist()

    # Desired distance from the right side and buffer distance
    desired_distance = 0.5  # adjust as needed
    buffer_distance = 0.2  # adjust as needed

    # Modify the look-ahead distance based on speed
    L_d = 0.4 if twist.linear.x < 1 else 2

    med_x = sr_prawo if sr_prawo < sr_lewo else sr_lewo
    goal_x = desired_distance - med_x if sr_prawo < sr_lewo else med_x - desired_distance

    # R = (L_d ** 2) / (2 * abs(goal_x)) if abs(goal_x) != 0 else 1

    cos_a = np.clip(goal_x, -1, 1)
    delta = np.arccos(cos_a)
    if delta == 0:
        delta = 0.01
    delta = np.clip(delta, -0.90, 0.90)
    limf = 12.0


    # Modify linear speed based on goal distance and delta
    red = min(0.5, abs(goal_x) * delta)
    twist.linear.x = 1 - (red/4)

    angular_z_candidate = 0

    # Control the angular velocity based on the difference between the current distance and the desired distance
    if sr_prawo > desired_distance + buffer_distance:
        angular_z_candidate = -delta / limf
    elif sr_prawo < desired_distance - buffer_distance:
        angular_z_candidate = delta / limf

    # Rate limiting
    if abs(angular_z_candidate - prev_angular_z) > max_rate:
        twist.angular.z = prev_angular_z + max_rate if angular_z_candidate > prev_angular_z else prev_angular_z - max_rate
    else:
        twist.angular.z = angular_z_candidate

    prev_angular_z = twist.angular.z

    # Control the vehicle to stop and turn accordingly if the front distance is less than a safe distance
    safe_distance_front = 0.8
    if sr_przod < safe_distance_front:
        twist.linear.x = 1 - red
        twist.angular.z = 1 if sr_prawo < sr_lewo else -1

    safe_distance = 0.4  # adjust as needed for side distances
    if sr_lewo < safe_distance:
        twist.angular.z = -delta
    if sr_prawo < (safe_distance-0.1):
        twist.angular.z = delta

    pub.publish(twist)

def callback(msg):
    prawo_o, lewo_o, przod_o = [], [], []

    for i in range(850, 1000):
        if msg.ranges[i] < 100 and msg.ranges[i + 1] < 100:
            prawo_o.append(calculate_c(msg.ranges[i], msg.ranges[i + 1]))

    for i in range(110, 280):
        if msg.ranges[i] < 100 and msg.ranges[i + 1] < 100:
            lewo_o.append(calculate_c(msg.ranges[i], msg.ranges[i + 1]))

    for i in range(1071, 1146):
        if msg.ranges[i] < 100 and msg.ranges[i + 1] < 100:
            przod_o.append(calculate_c(msg.ranges[i], msg.ranges[i + 1]))

    for i in range(0, 75):
        if msg.ranges[i] < 100 and msg.ranges[i + 1] < 100:
            przod_o.append(calculate_c(msg.ranges[i], msg.ranges[i + 1]))

    # Smoothing paths using moving average filter
    prawo_o = smooth_path(prawo_o)
    lewo_o = smooth_path(lewo_o)
    przod_o = smooth_path(przod_o)

    sr_prawo = calculate_sr(prawo_o)
    sr_przod = calculate_sr(przod_o)
    sr_lewo = calculate_sr(lewo_o)

    control_angular_velocity(sr_prawo, sr_lewo, sr_przod)


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
