# coding=utf-8
import rospy
import rospkg
from gazebo_msgs.srv import GetModelState, GetLinkState, SetModelState, DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import *

def delete_ball():
    # Deleting a model
    rospy.ServiceProxy('gazebo/delete_model', DeleteModel)("tennisball")

def spawn_ball(init_pos):
    # Adding a model
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    with open("/home/beau/Desktop/Caltech/Senior/Fall/me133a/133ws/src/tennisbot/models/tennisball/model.sdf", "r") as f:
        product_xml = f.read()

    item_name = "tennisball"

    init_pose = Pose()
    init_pose.position.x = init_pos[0]
    init_pose.position.y = init_pos[1]
    init_pose.position.z = init_pos[2]

    spawn_model(item_name, product_xml, "", init_pose, "world")

def launch_ball(init_pos, init_vel):
    state_msg = ModelState()
    state_msg.model_name = "tennisball"

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
    except rospy.ServiceException:
        print("Service call failed")
