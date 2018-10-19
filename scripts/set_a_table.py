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
from Tkinter import *
import numpy as np

import active_utils as autils
from plot_flower_queries import get_query_data 


markers = {}

tfBuffer = tf2_ros.Buffer()       
listener=tf2_ros.TransformListener(tfBuffer)


right_arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='right')
right_gripper = Gripper(prefix='right')

left_arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='left')
left_gripper = Gripper(prefix='left')

demo_pos = (-0.2,-0.2)
v = None
b = None
r = None
o = None
g = None

goal_pose = (0.0,0.0)

class SimpleGUI:
    def __init__(self, master):
		global demo_pos,v,r,g,b,o
		self.master = master
		master.title("User Interface for Set A Table Task")
		m = PanedWindow(orient=VERTICAL)
		m.pack(fill=BOTH, expand=1)
	
		self.label = Label(m, text="Select objects in current configuration:")
		self.label.pack(anchor=W)
		v = BooleanVar()
		r = BooleanVar()
		g = BooleanVar()
		b = BooleanVar()
		o = BooleanVar()
		
		obj_vase_button = Checkbutton(m, text="Vase", padx = 20, variable=v,onvalue=True, offvalue=False).pack(side=LEFT)
		obj_blue_button = Checkbutton(m, text="Blue",padx = 20, variable=b,onvalue=True, offvalue=False).pack(side=LEFT)
		obj_rend_button = Checkbutton(m, text="Red",padx = 20, variable=r,onvalue=True, offvalue=False).pack(side=LEFT)
		obj_green_button = Checkbutton(m, text="Green",padx = 20, variable=g,onvalue=True, offvalue=False).pack(side=LEFT)
		obj_orange_button = Checkbutton(m, text="Orange",padx = 20, variable=o,onvalue=True, offvalue=False).pack(side=LEFT)

		self.home_button = Button(master, text="Home Pose", command= arm_homing)
		self.home_button.pack(fill=X,side=BOTTOM)

		self.perceive_button = Button(master, text="Perceive Table", command=construct_world)
		self.perceive_button.pack(fill=X,side=TOP)

		#self.update_button = Button(master, text="Update Goal", command=collect_visual_demo)
		#self.update_button.pack()

		self.vase_button = Button(master, text="Place Vase", command=place_vase)
		self.vase_button.pack(fill=X,side=TOP)

		self.utensil_button = Button(master, text="Place Utensil", command=put_down_utensil)
		self.utensil_button.pack(fill=X,side=TOP)

		self.plate_button = Button(master, text="Place Plate", command=put_down_plate)
		self.plate_button.pack(fill=X,side=TOP)

		self.grasp_button1 = Button(master, text="Grasp Object (Left)", command=grasp_object_left)
		self.grasp_button1.pack(fill=X,side=TOP)
		self.grasp_button2 = Button(master, text="Grasp Object (Right)", command=grasp_object_right)
		self.grasp_button2.pack(fill=X,side=TOP)
		# self.close_button = Button(master, text="Close", command=master.quit)
		# self.close_button.pack(fill=X,side=TOP)

        
def collect_visual_demo(skip_check=False):
	global demo_pos
	rospy.loginfo("Collecting visual demonstration")
	printed = False
	while not (7 in markers):
		if not printed:
		    rospy.loginfo("Waiting for marker 7 ...")
		    printed = True
	if not skip_check:
	    raw_input('Press enter to record location')
	transform = tfBuffer.lookup_transform('table','ar_marker_7',rospy.Time(0), rospy.Duration(1.0))
	ps = geometry_msgs.msg.PoseStamped()
	ps.header.stamp = rospy.Time.now()
	pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
	pt.header.stamp = rospy.Time.now()
	pt.header.frame_id = 'table'
	x = pt.pose.position.x
	y = pt.pose.position.y	  
	demo_pos = (x,y)
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

def construct_world(objects=None):
       #      #     #     #     #     #
    #marker0             
    #
    #
    #
    #
    global frames_client,v,c,b,p, goal_pose
    printed = False
    while not (0 in markers):
        if not printed:
            rospy.loginfo("Waiting for marker 0 ...")
            printed = True
    rospy.loginfo("Constructing working environment ...")
    width = 0.9  
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

    #raw_input('Workspace constructed, press Enter to continue extracting features...')
    rospy.loginfo('Detecting Features ...') 
    if not objects:
        objects = []
        if v.get():  objects.append("vase")
        if r.get():  objects.append("red_bowl")
        if g.get():  objects.append("green_plate")
        if b.get():  objects.append("blue_cup")
        if o.get():  objects.append("orange_cup")
        print objects
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
		object_poses[obj] = (-obj_x,-obj_y)
    	
    centers = np.array(object_poses.values())
    #get leanred weights for MAP Reward function
    filename = "./flowers_seed0_randomFalse_demo10.txt"
    _, _, obj_weights, abs_weights, _, _  = get_query_data(filename)
    map_reward = autils.RbfComplexReward(centers, obj_weights, abs_weights)
    pi_map, _ = map_reward.estimate_best_placement()
    print(pi_map)
    hlds = []
    plt.xlim(0,width)
    plt.ylim(height,0)
    for obj in object_poses:
        hlds.append(plt.scatter(object_poses[obj][1],object_poses[obj][0]))
        plt.annotate(obj, (object_poses[obj][1],object_poses[obj][0]))
    hlds.append(plt.scatter(pi_map[1],pi_map[0]))
    plt.annotate('placement', (pi_map[1],pi_map[0]))
    plt.show()
    return True

def grasp_object_right():
    global right_gripper, left_gripper
    right_gripper.close()
        
def grasp_object_left():
    global left_gripper
    left_gripper.close()
        
def put_down_plate(pos=None):
   
    global right_arm, left_arm, right_gripper, left_gripper
           
    if not pos:
        pos = collect_visual_demo(skip_check=True)
    if pos[1] < -0.45:
        arm = right_arm
        gripper = right_gripper
        ori = [0.305, 0.588, 0.389, 0.640]
        disp = -0.08
    else:
        arm = left_arm
        gripper = left_gripper
        ori = [0.649, -0.302, -0.607, 0.346]
        disp = 0.08
        
    transform = tfBuffer.lookup_transform('linear_actuator_link','table',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'
    # 0.639, -0.202, -0.326, 0.667
    # 0.633, 0.383, -0.172, 0.650
    # side: 0.305, 0.588, 0.389, 0.640
    pt.pose.orientation.x = ori[0] 
    pt.pose.orientation.y = ori[1]
    pt.pose.orientation.z = ori[2]
    pt.pose.orientation.w = ori[3]
    pt.pose.position.x += pos[0] 
    pt.pose.position.y += pos[1] + disp
    pt.pose.position.z += 0.1 # TODO hardcode this to be table height
    waypoints = []
    waypoints.append(deepcopy(pt.pose))
    pt.pose.position.z -= 0.1
    waypoints.append(deepcopy(pt.pose))
    #gripper.close(100)
    
    
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    rospy.sleep(2)
    gripper.open()
    
    # Retract arm
    rospy.sleep(5)
    new_waypoints = []
    new_waypoints.append(deepcopy(pt.pose))
    pt.pose.position.y += disp/2
    pt.pose.position.z += 0.05
    new_waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x -= 0.1
    pt.pose.position.z += 0.2
    new_waypoints.append(deepcopy(pt.pose))
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(new_waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    return succeeded
    

def put_down_utensil(pos=None):
    global right_arm, left_arm, right_gripper, left_gripper
           
    if not pos:
        pos = collect_visual_demo(skip_check=True)
    if pos[1] < -0.45:
        arm = right_arm
        gripper = right_gripper
        ori = [0.006, 0.692, 0.065, 0.719]
    else:
        arm = left_arm
        gripper = left_gripper
        ori = [0.716, 0.014, -0.696, 0.053] #-0.690, 0.027, 0.722, 0.033] #0.714, 0.051, -0.698, 0.013]
    transform = tfBuffer.lookup_transform('linear_actuator_link','table',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'

    pt.pose.orientation.x = ori[0] 
    pt.pose.orientation.y = ori[1]
    pt.pose.orientation.z = ori[2]
    pt.pose.orientation.w = ori[3]
    pt.pose.position.x += pos[0] - 0.02
    pt.pose.position.y += pos[1]
    pt.pose.position.z += 0.15
    waypoints = []
    waypoints.append(deepcopy(pt.pose))
    pt.pose.position.z -= 0.14
    waypoints.append(deepcopy(pt.pose))
    #gripper.close(100)
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    rospy.sleep(2)
    gripper.open()
    
    # Retract arm
    rospy.sleep(5)
    new_waypoints = []
    new_waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x -= 0.1
    pt.pose.position.z += 0.05
    new_waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x -= 0.1
    pt.pose.position.z += 0.05
    new_waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x -= 0.05
    pt.pose.position.z += 0.2
    new_waypoints.append(deepcopy(pt.pose))
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(new_waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    return succeeded


def arm_homing():
    global right_arm, left_arm
    right_jointTarget = [0.947, 5.015, 4.95, 1.144, 11.425, 4.870, 7.281]
    right_arm.move_to_joint_pose(right_jointTarget)
    left_jointTarget = [-3.986, 4.736, 4.3435, 5.028, 7.656, 1.151, -4.271]
    left_arm.move_to_joint_pose(left_jointTarget)
    rospy.sleep(2)
    
def place_vase(pos=None):
    global right_arm, left_arm, right_gripper, left_gripper
           
    if not pos:
        pos = collect_visual_demo(skip_check=True)
    if pos[1] < -0.45:
        arm = right_arm
        gripper = right_gripper
        ori = [0.0, 0.0, 0.0, 1.0]
    else:
        arm = left_arm
        gripper = left_gripper
        ori = [1.0, 0.0, 0.0, 0.0]
    transform = tfBuffer.lookup_transform('linear_actuator_link','table',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'
    pt.pose.orientation.x = ori[0] 
    pt.pose.orientation.y = ori[1]
    pt.pose.orientation.z = ori[2]
    pt.pose.orientation.w = ori[3]
    pt.pose.position.x += pos[0] - 0.04
    pt.pose.position.y += pos[1]
    pt.pose.position.z += 0.15
    waypoints = []
    waypoints.append(deepcopy(pt.pose))
    pt.pose.position.z -= 0.13
    waypoints.append(deepcopy(pt.pose))
    #gripper.close(100)
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    rospy.sleep(2)
    gripper.open()
    
    # Retract arm
    rospy.sleep(5)
    new_waypoints = []
    new_waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x -= 0.1
    pt.pose.position.z += 0.05
    new_waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x -= 0.1
    pt.pose.position.z += 0.05
    new_waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x -= 0.05
    pt.pose.position.z += 0.2
    new_waypoints.append(deepcopy(pt.pose))
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(new_waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    return succeeded
    

def acitve_var_set_a_table():
    global frames_client, features, arm
    rospy.init_node('acitve_var_learning_agent')

    env = collision_objects()
    env.publish_collision_objects()
    
    #arm_homing()

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, update_marker_pose)
    rospy.wait_for_service('broadcast_object_frames')
    frames_client = rospy.ServiceProxy("broadcast_object_frames", BroadcastObjectFrames)
    #rospy.wait_for_service('active_var')
    #active_var_client = rospy.ServiceProxy("active_var", ActiveVaRQuery )

    root = Tk()
    my_gui = SimpleGUI(root)
    root.mainloop()

    '''if not construct_world():
        rospy.logerr("Failed to construct environment")
        return 
    
    iteration = 0

    demonstrations = []
    for itr in range(2):
        demo = collect_visual_demo()
        print("demo pose:",demo)
        put_down_plate(demo)
    
        arm_homing()'''
    #iteration += 1
    # execute current policy
        
        
       

if __name__ == "__main__":
    acitve_var_set_a_table()
