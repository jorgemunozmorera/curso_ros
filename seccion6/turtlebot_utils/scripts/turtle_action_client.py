#! /usr/bin/env python
import actionlib
import rospy
# Now we import the generated actionlib messages. All have the same signature: 
from turtlebot_utils.msg import GoToAction # NameOfActionAction
from turtlebot_utils.msg import GoToGoal # NameOfActionGoal
# This message comes from turtlesim, it was not created by us.
from turtlesim.msg import Pose

# This callback executes when server becomes active.
def active():
    rospy.loginfo("Turtle action server became active.");

# This callback executes when server sends feedback.
def feedback(feed):
    rospy.loginfo("Feedback: \n%s", feed);

# This callback executes when server ends its job.
def done(state, result):
    rospy.loginfo("Turtle action server finished.");
    rospy.loginfo("State: \n%s", state);
    # We don't print result here to show how to retrieve it later.

# Our main function.
def goto_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('goto', GoToAction) 
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server. We don't use tolerance here.
    m = Pose()
    m.x = input("Set your x goal:")
    m.y = input("Set your y goal:")
    goal = GoToGoal(m)
    # Sends the goal to the action server, specifying the callbacks.
    client.send_goal(goal, done, active, feedback)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Returns the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        rospy.init_node('turtle_client_py')
        result = goto_client()
        rospy.loginfo("Result: \n%s", result);
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion.");
