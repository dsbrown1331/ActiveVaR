#!/usr/bin/env python
import rospy
import numpy as np
import gazebo_msgs.srv
from geometry_msgs.msg import PoseStamped
import moveit_commander

### This is now just for simulation
class collision_objects:
    def __init__(self, scene=None, sim_mode=True):
        if scene:
            self.scene = scene
        else:
            self.scene = moveit_commander.PlanningSceneInterface()
        if sim_mode:
            self.gms = rospy.ServiceProxy('gazebo/get_model_state', gazebo_msgs.srv.GetModelState)

    def publish_collision_objects(self):
        hb_pose_stamped = PoseStamped()
        hb_pose_stamped.header.frame_id = "goal_frame"
        ## Assuming the orientation is right
        hb_pose_stamped.pose.position.x = 0. #-0.05
        hb_pose_stamped.pose.position.y = 0.
        hb_pose_stamped.pose.position.z = 0.0 
        hb_pose_stamped.pose.orientation.x = 0.
        hb_pose_stamped.pose.orientation.y = 0.
        hb_pose_stamped.pose.orientation.z = 0.
        hb_pose_stamped.pose.orientation.w = 1.
        hb_scale = [0.18, 0.26, 0.01] ## Attaching asquare leg instead of cylinder. Diameter of cylinder = diagoal of square
        ### Floor
        floor_pose =  PoseStamped()
        floor_pose.header.frame_id = "base_link"
        floor_pose.pose.position.x = 0
        floor_pose.pose.position.y = 0
        floor_pose.pose.position.z = 0.1
        floor_pose.pose.orientation.x = 0;
        floor_pose.pose.orientation.y = 0;
        floor_pose.pose.orientation.z = 0;
        floor_pose.pose.orientation.w = 1;
        floor_scale = [5,5,0.01]

        self.scene.attach_box("left_robotiq_85_base_link","hollow_box", hb_pose_stamped, hb_scale)
        rospy.sleep(3)
        self.scene.add_box("floor",floor_pose,floor_scale)
        rospy.sleep(2)
 
        self.scene.remove_world_object("table_box")
      
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.pose.position.x = 1.1 
        p.pose.position.y = 0.15
        p.pose.position.z = 0.40 
        p.pose.orientation.w = 0.0
        self.scene.add_box("table_box",p,(0.7, 1.5, 0.75))
     
        rospy.sleep(3)

        return

    def removeAll(self):
        print "Removing collision Objects"
        self.scene.remove_attached_object("left_robotiq_85_base_link", name="hollow_box")
        self.scene.remove_attached_object("right_robotiq_85_base_link",name="peg")
        self.scene.remove_world_object(name="peg")
        self.scene.remove_world_object(name="hollow_box")      
        rospy.sleep(2)

    def removeObject(self, object_name):
        print "Removing Peg as collision Object"
        self.scene.remove_world_object(name=object_name)

    def attachPeg(self):
        # peg = self.gms('peg', 'world').pose
        rel_pose3 = PoseStamped()
        rel_pose3.header.frame_id = "right_robotiq_85_base_link"
        ## Assuming the orientation is right
        rel_pose3.pose.position.x = 0.168
        rel_pose3.pose.position.y = 0.
        rel_pose3.pose.position.z = 0. 
        rel_pose3.pose.orientation.x = 0.
        rel_pose3.pose.orientation.y = 0.
        rel_pose3.pose.orientation.z = 0.
        rel_pose3.pose.orientation.w = 1.
        peg_scale = [0.10, 0.011, 0.011] ## Attaching asquare leg instead of cylinder. Diameter of cylinder = diagoal of square
        self.scene.attach_box("right_robotiq_85_base_link","peg", rel_pose3, peg_scale)

    def detachPeg(self):
        scene.remove_attached_object("right_robotiq_85_base_link",name="peg")
        rospy.sleep(1)
