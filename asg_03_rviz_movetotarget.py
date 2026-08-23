#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import math

target_x=0.0
target_y=0.0
target_yaw=0.0

def target(data):
    global target_x, target_y, target_yaw
    target_x = data.pose.position.x
    target_y = data.pose.position.y
    target_z = data.pose.position.z
    target_qx = data.pose.orientation.x
    target_qy = data.pose.orientation.y
    target_qz = data.pose.orientation.z
    target_qw = data.pose.orientation.w
    (_,_,target_yaw) = euler_from_quaternion([target_qx, target_qy, target_qz, target_qw])


rospy.init_node("rviz_movetotarget")
listener=tf.TransformListener()
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.Subscriber("/move_base_simple/goal", PoseStamped, target)
msg=Twist()

while not rospy.is_shutdown():
    try:
        (trans,rot)=listener.lookupTransform("/odom","/base_footprint",rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    
    robot_x=trans[0]
    robot_y=trans[1]
    (robot_qx,robot_qy,robot_qz,robot_qw)=rot
    (_,_,robot_yaw)=euler_from_quaternion([robot_qx,robot_qy,robot_qz,robot_qw])
    
    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.sqrt(dx**2 + dy**2)
    print(f"Distance: {distance}")
    
    angle_rad = math.atan2(dy, dx)
    yaw_error = angle_rad - robot_yaw  
    # Normalize to [-pi, pi] to take the shortest turn
    yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
    
    if distance > 0.1: 
        if abs(yaw_error) > 0.1: 
            msg.linear.x = 0.0
            # Turn toward target
            msg.angular.z = 0.2 if yaw_error > 0 else -0.2  
        else:
            # Drive toward target
            msg.linear.x = 0.2   
            msg.angular.z = 0.0          
    else:
        final_yaw_error = target_yaw - robot_yaw
        final_yaw_error = math.atan2(math.sin(final_yaw_error), math.cos(final_yaw_error))
        
        if abs(final_yaw_error) > 0.1:
            msg.linear.x = 0.0
            # Turn toward arrow
            msg.angular.z = 0.2 if final_yaw_error > 0 else -0.2
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

    pub.publish(msg)
    rospy.sleep(0.1)