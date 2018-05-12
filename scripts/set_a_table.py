#!/usr/bin/env python
from subprocess import call
from hlpr_single_plane_segmentation.srv import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from active_var.msg import *
from active_var.srv import *
import rospy
import math
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose
from hlpr_manipulation_utils.arm_moveit2 import *
from hlpr_manipulation_utils.manipulator import Gripper
from add_collision_objects import *
from copy import deepcopy
import matplotlib.pyplot as plt

markers = {}

tfBuffer = tf2_ros.Buffer()       
listener=tf2_ros.TransformListener(tfBuffer)

arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='right')
gripper = Gripper(prefix='right')
left_arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='left')
left_gripper = Gripper(prefix='left')

def collect_visual_demo():
	rospy.loginfo("Collecting visual demonstration")
	printed = False
	while not (7 in markers):
		if not printed:
		    rospy.loginfo("Waiting for marker 7 ...")
		    printed = True

	raw_input('Press enter to record location')
	transform = tfBuffer.lookup_transform('table','ar_marker_7',rospy.Time(0), rospy.Duration(1.0))
	ps = geometry_msgs.msg.PoseStamped()
	ps.header.stamp = rospy.Time.now()
	pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
	pt.header.stamp = rospy.Time.now()
	pt.header.frame_id = 'table'
	x = pt.pose.position.x
	y = pt.pose.position.y	  
				   	 
	return (x,y)

def update_marker_pose(data):
    for marker in data.markers:
        curr_pose = marker.pose.pose.position
        if marker.id in markers:
            markers[marker.id].x += curr_pose.x
            markers[marker.id].y += curr_pose.y
            markers[marker.id].z += curr_pose.z
            markers[marker.id].x /= 2
            markers[marker.id].y /= 2
            markers[marker.id].z /= 2
        else:
            markers[marker.id] = curr_pose


def distance2D(point1, point2):
    return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

def distance(point1, point2):
    return math.sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2+(point1.z-point2.z)**2)

def construct_world():
       #      #     #     #     #     #
    #marker0             
    #
    #
    #
    #
    global frames_client
    printed = False
    while not (0 in markers):
        if not printed:
            rospy.loginfo("Waiting for marker 0 ...")
            printed = True
    rospy.loginfo("Constructing working environment ...")
    width = 0.6  
    height = 0.5 
    rospy.loginfo("workspace size:"+str(width)+" " +str(height))
   
    marker0_transform = tfBuffer.lookup_transform('linear_actuator_link','ar_marker_0',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, marker0_transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'

    tf_frames_request = BroadcastObjectFramesRequest()
    tf_frames_request.parent_frames = ["linear_actuator_link"]
    tf_frames_request.child_frames = ["table"]
    pose = geometry_msgs.msg.Pose()
    pose.position.x = pt.pose.position.x 
    pose.position.y = pt.pose.position.y 
    pose.position.z = pt.pose.position.z + 0.05
    pose.orientation.w = 1
    tf_frames_request.poses = [pose]
    tf_res = frames_client(tf_frames_request)
    if tf_res is False:
        rospy.logerr("Update failed")
        return False

    raw_input('Workspace constructed, press Enter to continue extracting features...')
    rospy.loginfo('Detecting Features ...') 
    objects = ["cup","bowl","plate"]
    object_poses = {}

    for obj in objects:
		plant_transform = tfBuffer.lookup_transform('table',obj,rospy.Time(0), rospy.Duration(1.0))
		ps = geometry_msgs.msg.PoseStamped()
		ps.header.stamp = rospy.Time.now()
		pt = tf2_geometry_msgs.do_transform_pose(ps, plant_transform)
		pt.header.stamp = rospy.Time.now()
		obj_x = pt.pose.position.x
		obj_y = pt.pose.position.y
		rospy.loginfo('---> '+obj+' position: '+str(obj_x)+", "+str(obj_y))
		object_poses[obj] = (obj_x,obj_y)
    	
    hlds = []
    plt.xlim(0,width)
    plt.ylim(-height,0)
    for obj in object_poses:
        hlds.append(plt.scatter(-object_poses[obj][1],object_poses[obj][0]))
        plt.annotate(obj, (-object_poses[obj][1],object_poses[obj][0]))
    #plt.legend(hlds,objects)
    rospy.loginfo('Redeay to collect visual demos')
    plt.show()
    return True

def grasp_object(left=False):
    global arm, gripper
    if left:
        left_gripper.close()
    else:
        gripper.close()

def move_to_position(pos):
    global arm, gripper
    transform = tfBuffer.lookup_transform('linear_actuator_link','table',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'
    pt.pose.orientation.x = 0.006 
    pt.pose.orientation.y = 0.692
    pt.pose.orientation.z = 0.065
    pt.pose.orientation.w = 0.719
    pt.pose.position.x += pos[0]
    pt.pose.position.y += pos[1]
    pt.pose.position.z += 0.18
    waypoints = []
    waypoints.append(deepcopy(pt.pose))
    pt.pose.position.z -= 0.1
    waypoints.append(deepcopy(pt.pose))
    pt.pose.position.z += 0.1
    waypoints.append(deepcopy(pt.pose))
    gripper.close(100)
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    rospy.sleep(2)


def arm_homing():
    global arm
    jointTarget = [0.947, 5.015, 4.95, 1.144, 11.425, 4.870, 7.281]
    arm.move_to_joint_pose(jointTarget)
    rospy.sleep(3)
    


def acitve_var_set_a_table():
    global frames_client, features, arm
    rospy.init_node('acitve_var_learning_agent')

    env = collision_objects()
    env.publish_collision_objects()
    
    arm_homing()

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, update_marker_pose)
    rospy.wait_for_service('broadcast_object_frames')
    frames_client = rospy.ServiceProxy("broadcast_object_frames", BroadcastObjectFrames)
    rospy.wait_for_service('active_var')
    active_var_client = rospy.ServiceProxy("active_var", ActiveVaRQuery )

    if not construct_world():
        rospy.logerr("Failed to construct environment")
        return 
    
    iteration = 0

    demonstrations = []
    for itr in range(5):
        demo = collect_visual_demo()
        print("demo pose:",demo)
        move_to_position(demo)
    
        arm_homing()
    #iteration += 1
    # execute current policy
        
        
       

if __name__ == "__main__":
    acitve_var_set_a_table()
