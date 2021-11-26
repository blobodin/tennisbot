# coding=utf-8
import rospy
import rospkg
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


def launch_ball(init_pos, init_vel, model_name):
    state_msg = ModelState()
    state_msg.model_name = model_name

    # Code for setting position
    state_msg.pose.position.x = init_pos[0]
    state_msg.pose.position.y = init_pos[1]
    state_msg.pose.position.z = init_pos[2]

    # Code for setting orientation
    # state_msg.pose.orientation.x = 0
    # state_msg.pose.orientation.y = 0
    # state_msg.pose.orientation.z = 0
    # state_msg.pose.orientation.w = 0

    # Code for setting linear velocities
    state_msg.twist.linear.x = init_vel[0]
    state_msg.twist.linear.y = init_vel[1]
    state_msg.twist.linear.z = init_vel[2]

    # Code for setting angular velocites
    # state_msg.twist.angular.x = 0
    # state_msg.twist.angular.x = 0
    # state_msg.twist.angular.x = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy(
          '/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
        print(state_msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e]
