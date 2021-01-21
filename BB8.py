#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Accel

class MoveRobot():
    
    def __init__(self):
        self.robot_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.robot_position_publisher = rospy.Publisher('positionTopic', Point, queue_size=1)
        self.robot_orientation_publisher = rospy.Publisher('positionTopic', Vector3, queue_size=1)
        self.robot_acceleration_publisher = rospy.Publisher('positionTopic', Accel, queue_size=1)

        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        self.position = Point()
        self.orientation = Vector3()
        self.accel = Accel()
        rospy.on_shutdown(self.shutdownhook)
    
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_robot()
        self.ctrl_c = True
        
    def stop_robot(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_robot(self, moving_time, linear_speed=0.2, angular_speed=0.2):
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        i = 0
        rospy.loginfo("Moving Robot!")
        while not self.ctrl_c and i <= moving_time:
            self.publish_once_in_cmd_vel()
            i = i+1
            self.rate.sleep()
        self.stop_robot()
    
    def turn(self, x, y, z):
        self.orientation.x = x
        self.orientation.y = y
        self.orientation.z = z

    def update_position(self, x, y, z):
        self.position.x = x
        self.position.y = y
        self.position.z = z
        
    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.robot_vel_publisher.get_num_connections()
            if connections > 0:
                self.robot_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def publish_position(self):
         while not self.ctrl_c:
            connections = self.robot_position_publisher.get_num_connections()
            if connections > 0:
                self.robot_position_publisher.publish(self.position)
                rospy.loginfo("Coordinates Published")
                break
            else:
                self.rate.sleep()

    def publish_orientation(self):
         while not self.ctrl_c:
            connections = self.robot_orientation_publisher.get_num_connections()
            if connections > 0:
                self.robot_position_publisher.publish(self.orientation)
                rospy.loginfo("Orientation Published")
                break
            else:
                self.rate.sleep()

    def publish_acceleration(self):
         while not self.ctrl_c:
            connections = self.robot_acceleration_publisher.get_num_connections()
            if connections > 0:
                self.robot_acceleration_publisher.publish(self.acceleration)
                rospy.loginfo("Acceleration Published")
                break
            else:
                self.rate.sleep()
    

            
if __name__ == '__main__':
    rospy.init_node('move_robot_test', anonymous=True)
    moverobot_object = MoveRobot()
    try:
        moverobot_object.move_robot()
    except rospy.ROSInterruptException:
        pass