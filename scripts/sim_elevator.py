#!/usr/bin/env python

import rospy
import math
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel, GetWorldPropertiesRequest, DeleteModelRequest, \
    SpawnModelRequest
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from os.path import expanduser


class sim_elevator:
    home = expanduser('~')
    panel_pressed_sdf = home + '/.gazebo/models/button_panel_pressed/model.sdf'
    elevator_door_sdf = home + '/.gazebo/models/elevator_door/model.sdf'
    level2_pressed_sdf = home + '/.gazebo/models/level2_pressed/model.sdf'
    model_name = 'armadillo2'

    def __init__(self):

        # wait for gazebo
        while "/gazebo/link_states" not in dict(rospy.get_published_topics()).keys():
            rospy.sleep(2)

        self.gripper_idx = 0
        self.panel_idx = 0
        self.door_idx = 0
        self.inner_panel_idx = 0

        self.outer_pressed = 0
        self.inner_pressed = 0

        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback)

        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")

        self.get_world_properties_srv = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    def del_model(self, name):
        """ delete a model whose name contains given name
        Args:
            name (string): part of model name
        """
        get_world_properties_request = GetWorldPropertiesRequest()
        delete_request = DeleteModelRequest()
        resp = self.get_world_properties_srv(get_world_properties_request)
        for model in resp.model_names:
            if name in model:
                delete_request.model_name = model
                self.delete_srv(delete_request)
                break

    def spawn_model(self, model_name, pose):
        """ spawn model into given pose
        Args:
            model_name (string): 'button_panel_pressed' or 'elevator_door' or 'level2_pressed'
            pose (geometry_msgs.msg.Pose): location in simulation
        """
        r = SpawnModelRequest()
        if model_name == "panel_pressed":
            f = open(self.panel_pressed_sdf)
        elif model_name == "level2_pressed":
            f = open(self.level2_pressed_sdf)
        else:
            f = open(self.elevator_door_sdf)
        r.model_xml = f.read()
        f.close()

        r.model_name = model_name
        r.initial_pose = pose
        self.spawn_srv(r)

    def link_states_callback(self, data):
        """Change buttons to pressed, open / close door
        Args:
            data (gazebo_msgs.msg.LinkStates): all link states in simulation
        """
        for i, name in enumerate(data.name):
            if name == 'armadillo2::right_finger_link':
                self.gripper_idx = i
            elif name == 'elevator_shaft::buttons_link':
                self.panel_idx = i
            elif name == 'elevator_door::link':
                self.door_idx = i
            elif name == 'elevator_shaft::buttons_panel':
                self.inner_panel_idx = i

        if not self.outer_pressed:
            x1 = data.pose[self.gripper_idx].position.x
            x2 = data.pose[self.panel_idx].position.x
            y1 = data.pose[self.gripper_idx].position.y
            y2 = data.pose[self.panel_idx].position.y
            z1 = data.pose[self.gripper_idx].position.z
            z2 = data.pose[self.panel_idx].position.z

            distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2))
            if distance < 0.25:
                self.outer_pressed = 1
                panel_pose = Pose()
                panel_pose.position.x = 1.343901
                panel_pose.position.y = -0.020712
                panel_pose.position.z = 1.215740
                panel_pose.orientation = data.pose[self.panel_idx].orientation
                self.spawn_model("panel_pressed", panel_pose)
                rospy.loginfo("[sim_elevator_node]: pressed elevator button")
                # open door after 15 seconds
                rospy.sleep(15)
                self.del_model("elevator_door")
                rospy.loginfo("[sim_elevator_node]: door opened")
                # close door after 30 seconds
                # rospy.sleep(40)
                # door_pose = Pose()
                # door_pose.position.x = 0.964234
                # door_pose.position.y = 1.064952
                # door_pose.position.z = 0
                # door_pose.orientation = data.pose[door_idx].orientation
                # self.spawn_model("elevator_door", door_pose)
                # rospy.loginfo("[sim_elevator_node]: door closed")

        # if outer button is pressed but inner still isn't
        elif not self.inner_pressed:
            x1 = data.pose[self.gripper_idx].position.x
            x2 = data.pose[self.inner_panel_idx].position.x
            y1 = data.pose[self.gripper_idx].position.y
            y2 = data.pose[self.inner_panel_idx].position.y
            z1 = data.pose[self.gripper_idx].position.z
            z2 = data.pose[self.inner_panel_idx].position.z

            distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2))
            if distance < 0.2:
                self.inner_pressed = 1
                panel_pose = Pose()
                panel_pose.position.x = 1.465790
                panel_pose.position.y = 1.915204
                panel_pose.position.z = 1.061664
                panel_pose.orientation = data.pose[self.inner_panel_idx].orientation
                self.spawn_model("level2_pressed", panel_pose)
                rospy.loginfo("[sim_elevator_node]: pressed level 2 button")


if __name__ == '__main__':
    rospy.init_node('sim_elevator_node', anonymous=True)

    sim = sim_elevator()
    rospy.loginfo("[sim_elevator_node]: started")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("[sim_elevator_node]: Shutting down")
