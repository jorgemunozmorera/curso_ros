#!/usr/bin/env python
import rospy
import actionlib
from turtlebot_utils.msg  import GoToAction
from turtlebot_utils.msg  import GoToFeedback
from turtlebot_utils.msg  import GoToResult
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt

class turtlebot():
    # Create messages that are used to publish feedback/result
    _feedback = GoToFeedback()
    _result = GoToResult()

    def __init__(self):
        #Create our publisher and subscriber.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(5)
        # Create the simple action server.
        self._as = actionlib.SimpleActionServer("goto", GoToAction, execute_cb=self.move2goal, auto_start = False)
        self._as.start()

    # Callback function for the subscriber, to update current pose
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    # Helper method top compute distance from currento position to goal,
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    # Execute callback for the action server. Its our GoTo PID function.
    def move2goal(self, goal):
        # publish info to the console for the user
        #rospy.loginfo('Executing GOTO, goal point (%f, %f)' % (goal.goal_pose.x, goal.goal_pose.y)
        # var to store the result of the goal
        success = True        
        goal_pose = Pose()
        goal_pose.x = goal.goal_pose.x
        goal_pose.y = goal.goal_pose.y
        distance_tolerance = 0.2
        vel_msg = Twist()

        while self.get_distance(goal_pose.x, goal_pose.y) >= distance_tolerance:
            # Check that preempt has not been requested by the client (this will never occur because our client only waits).
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted', "goto")
                self._as.set_preempted()
                success = False
                break

            # Publish feedback, our current pose
            self._feedback.current_pose = self.pose
            self._as.publish_feedback(self._feedback)    
            #linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * self.get_distance(goal_pose.x, goal_pose.y)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        #Stop our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

        if success:
            rospy.loginfo('%s: Succeeded', "goto")
            self._result.result_pose = self.pose
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    try:
        rospy.init_node('goto_server')
        server = turtlebot()
        rospy.spin()
    except rospy.ROSInterruptException: pass
