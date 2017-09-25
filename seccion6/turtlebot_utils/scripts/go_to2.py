#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,acos,fabs

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        print "Current position: ", self.pose.x, ",", self.pose.y

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)        

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = input("Set your x goal:")
        goal_pose.y = input("Set your y goal:")        
        distance_tolerance = input("Set your tolerance:")
        vel_msg = Twist()

        point_goal = (goal_pose.x, goal_pose.y)
        point_origin = (self.pose.x, self.pose.y)
        vec_goal = (point_goal[0] - point_origin[0], point_goal[1] - point_origin[1])
        print "Goal vector: ", vec_goal
        vec_goal_mod = sqrt(pow(vec_goal[0], 2) + pow(vec_goal[1], 2))
        vec_unity_x = (1.0, 0)
        scalar_product = vec_goal[0]*vec_unity_x[0];
        desired_angle = 0.0
        if goal_pose.y <= self.pose.y:
            desired_angle = 6.28319 - acos(scalar_product/vec_goal_mod)
        else:
            desired_angle = acos(scalar_product/vec_goal_mod)
        print "Desired angle: ", desired_angle
        print "Current angle: ", self.pose.theta
        print "Current position: ", self.pose.x, ",", self.pose.y
        # Giramos hasta obtener la orientacion deseada
        while fabs(self.pose.theta - desired_angle) >= distance_tolerance:
            print "Desired angle: ", desired_angle
            print "Current angle: ", self.pose.theta
            print "Error: ", fabs(self.pose.theta - desired_angle)
            vel_msg.angular.z = 0.1
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        # Avanzamos hasta obtener la posicion deseada
        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= 0.1:
            vel_msg.linear.x = 0.3
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = 0.0
            print "Desired position: ", goal_pose.x, ",", goal_pose.y
            print "Current position: ", self.pose.x, ",", self.pose.y
            print "Error: ", sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)
        print "Current position: ", self.pose.x, ",", self.pose.y
        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.move2goal()

    except rospy.ROSInterruptException: pass
