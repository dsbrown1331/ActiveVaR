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

GRID_WORLD_WIDTH = 6
GRID_WORLD_HEIGHT = 5

ACTIONS = {'UP':0, 'DOWN':1, 'LEFT':2, 'RIGHT':3, 'STAY':4 }

x_side = 0.1
y_side = 0.1

markers = {}

tfBuffer = tf2_ros.Buffer()       
listener=tf2_ros.TransformListener(tfBuffer)
features = {} # [is_goal[marker8], is_init[purple], is_plant[plant]]

arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='right')
gripper = Gripper(prefix='right')

def get_next_state(state, action):
    next_state = state
    if action == ACTIONS['UP']:
        next_state = state - GRID_WORLD_WIDTH
    if action == ACTIONS['DOWN']:
        next_state = state + GRID_WORLD_WIDTH
    if action == ACTIONS['LEFT']:
        next_state = state - 1
    if action == ACTIONS['RIGHT']:
        next_state = state + 1
    if action == ACTIONS['STAY']:
        next_state = state
    if next_state < 0 or next_state >= GRID_WORLD_WIDTH * GRID_WORLD_HEIGHT:
        return None
    return next_state

def collect_visual_demo(typed=False):
    rospy.loginfo("Collecting visual demonstrations")
    states = []
    actions = []
    
    if not typed:
		printed = False
		while not (7 in markers):
		    if not printed:
		        rospy.loginfo("Waiting for marker 7 ...")
		        printed = True
		option = int(input('Enter 1 to record demo, 0 to end:'))
		while option != 0:
		    transform = tfBuffer.lookup_transform('ar_marker_0','ar_marker_7',rospy.Time(0), rospy.Duration(1.0))
		    ps = geometry_msgs.msg.PoseStamped()
		    ps.header.stamp = rospy.Time.now()
		    ps.header.frame_id = 'ar_marker_7'
		    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
		    pt.header.stamp = rospy.Time.now()
		    pt.header.frame_id = 'ar_marker_0'
		    x = pt.pose.position.x
		    y = pt.pose.position.y
		    curr_state = get_state(x, y)
		    states.append(curr_state)
		    rospy.loginfo('state recorded: '+str(curr_state))
		    option = int(input('Enter 1 to record demo, 0 to end:'))
    else:
        option = int(input('Enter state or -1 to exit:'))
        while option != -1:
		    curr_state = option
		    states.append(curr_state)
		    rospy.loginfo('state recorded: '+str(curr_state))
		    option = int(input('Enter state or -1 to exit:'))
    	   	  
    for i in range(len(states)-1):
    	if states[i+1] == states[i]:
    	    actions.append(ACTIONS['STAY'])
    	elif states[i+1] == states[i] + 1:
    		actions.append(ACTIONS['RIGHT'])
    	elif states[i+1] == states[i] - 1:
    		actions.append(ACTIONS['LEFT'])
    	elif states[i+1] == states[i] + GRID_WORLD_WIDTH:
    		actions.append(ACTIONS['DOWN'])
    	elif states[i+1] == states[i] - GRID_WORLD_WIDTH:
    		actions.append(ACTIONS['UP'])
    	else:
    		actions.append(-1)
    		print("[ERROR finding action] state:",states[i+1],"is not right next to",states[i])
    		
    print "demos:",states,actions
    return [(states[i], actions[i]) for i in range(len(actions))]

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

def get_state_center(state):
    global x_side, y_side, num_states
    x = -(state//GRID_WORLD_WIDTH)*x_side 
    y = -(state%GRID_WORLD_WIDTH)*y_side 
    # print state, 'center:',x,", ", y
    return (x,y)

def get_state(x,y):
    global x_side, y_side, num_states
    state = ((int(math.floor(2*(x_side/2-x)/x_side))//2)*GRID_WORLD_WIDTH + int(math.floor(2*(y_side/2-y)/y_side))//2)
    print("x=",x,", y=",y, ", state=",state)
    return int(state)

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
    global frames_client, x_side, y_side, num_states, features
    rospy.sleep(1.0)
    printed = False
    while not (0 in markers):
        if not printed:
            rospy.loginfo("Waiting for marker 0 ...")
            printed = True
    rospy.loginfo("Constructing working environment ...")
    width = 0.5  #distance(markers[0],markers[4])
    height = 0.4 #distance(markers[0], markers[1])
    rospy.loginfo("workspace size:"+str(width)+" " +str(height))
    x_side = height / (GRID_WORLD_HEIGHT - 1)
    y_side = width / (GRID_WORLD_WIDTH - 1)
    rospy.loginfo("cell size:"+str(x_side)+" "+str(y_side))
    num_states = GRID_WORLD_WIDTH*GRID_WORLD_HEIGHT
    #parent_frames = ["ar_marker_0"]*num_states
    marker0_transform = tfBuffer.lookup_transform('linear_actuator_link','ar_marker_0',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, marker0_transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'

    parent_frames = ["linear_actuator_link"]*num_states
    tf_frames_request = BroadcastObjectFramesRequest()
    tf_frames_request.parent_frames = parent_frames
    child_frames = []
    for frame in range(num_states):
        child_frames.append("state_"+str(frame))
    tf_frames_request.child_frames = child_frames
    poses = []
    for frame in range(num_states):
        pose = Pose()
        #pose.position.x = pt.pose.position.x + x_side * ((num_states - 1 - frame) // GRID_WORLD_WIDTH)
        #pose.position.y = pt.pose.position.y + y_side * ((num_states - 1 - frame) % GRID_WORLD_WIDTH)
        pose.position.x = pt.pose.position.x - x_side * (frame // GRID_WORLD_WIDTH)
        pose.position.y = pt.pose.position.y - y_side * (frame % GRID_WORLD_WIDTH)
        pose.position.z = pt.pose.position.z + 0.03
        pose.orientation.w = 1
        poses.append(pose)
    tf_frames_request.poses = poses
    tf_res = frames_client(tf_frames_request)
    if tf_res is False:
        rospy.logerr("Update failed")
        return False

    raw_input('Workspace constructed, press Enter to continue extracting features...')
    rospy.loginfo('Detecting Features ...')
    printed = False
    while not (8 in markers): #target
        if not printed:
            rospy.loginfo("Waiting for marker 8 ...")
            printed = True

    plant_transform = tfBuffer.lookup_transform('state_0','plant',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, plant_transform)
    pt.header.stamp = rospy.Time.now()
    plant_x = pt.pose.position.x
    plant_y = pt.pose.position.y
    plant_state = get_state(plant_x,plant_y)
    rospy.loginfo('---> Plant center state: '+str(plant_state))
    
    target_transform = tfBuffer.lookup_transform('state_0','ar_marker_8',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, target_transform)
    pt.header.stamp = rospy.Time.now()
    target_x = pt.pose.position.x
    target_y = pt.pose.position.y
    target_state = get_state(target_x,target_y)

    rospy.loginfo('---> Target state: '+str(target_state))
    
    for state_i in range(GRID_WORLD_WIDTH*GRID_WORLD_HEIGHT):
        features[state_i] = [1.0,0.0,0.0]
        state_center = get_state_center(state_i)
        if (abs(state_center[0] - plant_x) < 0.11 and abs(state_center[1]-plant_y) < 0.05):
             features[state_i][2] = 1.0
             features[state_i][0] = 0.0
    features[target_state][1] = 1.0
    features[target_state][0] = 0.0
    	
    rospy.loginfo('Redeay to collect visual demos')
    return True

def grasp_cup():
    global arm, gripper
    gripper.open(100)
    target_transform = tfBuffer.lookup_transform('state_0','mug',rospy.Time(0), rospy.Duration(15.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'mug'
    pt = tf2_geometry_msgs.do_transform_pose(ps, target_transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'state_0'
    target_x = pt.pose.position.x
    target_y = pt.pose.position.y
    cup_state = get_state(target_x,target_y)

    transform = tfBuffer.lookup_transform('linear_actuator_link','mug',rospy.Time(0), rospy.Duration(15.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'mug'
    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'
    pt.pose.orientation.x = 0.0
    pt.pose.orientation.y = 0.0
    pt.pose.orientation.z = 0.0
    pt.pose.orientation.w = 1.0
    waypoints = []
    pt.pose.position.x -= 0.2
    pt.pose.position.z -= 0.01
    pt.pose.position.y += 0.025
    waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x += 0.1
    waypoints.append(deepcopy(pt.pose))
    pt.pose.position.x += 0.07
    waypoints.append(deepcopy(pt.pose))
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    rospy.sleep(2)
    gripper.close()
    gripper.close()
    return cup_state

def point_at_state(state):
    global arm, gripper
    transform = tfBuffer.lookup_transform('linear_actuator_link','state_'+str(state),rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'state_'+str(state)
    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'
    pt.pose.orientation.x = 0.006
    pt.pose.orientation.y = 0.692
    pt.pose.orientation.z = 0.065
    pt.pose.orientation.w = 0.719
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

def move_to_state(state):
    global arm
    transform = tfBuffer.lookup_transform('linear_actuator_link','state_'+str(state),rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'linear_actuator_link'
    pt.pose.orientation.w = 1.0
    pt.pose.orientation.x = 0.0
    pt.pose.orientation.y = 0.0
    pt.pose.orientation.z = 0.0
    pt.pose.position.x -= 0.1
    pt.pose.position.z += 0.18
    arm.move_to_ee_pose(pt)
    rospy.sleep(3)

def move_through_states(states, home_pose):
    global arm
    waypoints = []
    for state in states:
        transform = tfBuffer.lookup_transform('linear_actuator_link','state_'+str(state),rospy.Time(0), rospy.Duration(1.0))
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = 'linear_actuator_link'
        pt.pose.orientation.w = 1.0
        pt.pose.orientation.x = 0.0
        pt.pose.orientation.y = 0.0
        pt.pose.orientation.z = 0.0
        pt.pose.position.z += 0.18
        pt.pose.position.x -= 0.1
        waypoints.append(deepcopy(pt.pose))
    pt.pose.position.z -= 0.11
    waypoints.append(deepcopy(pt.pose))
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)
    print "Trajectory execution result: ", succeeded
    rospy.sleep(5)
    gripper.open(100)
    
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
    pt.pose.position.y -= 0.2
    pt.pose.position.z += 0.3
    new_waypoints.append(deepcopy(pt.pose))
    arm.set_start_state(None)
    (plan, fraction) = arm.group[0].compute_cartesian_path(new_waypoints, 0.01, 0.0)
    rospy.sleep(2)
    succeeded = arm.group[0].execute(plan)

    return succeeded

def arm_homing():
    global arm
    jointTarget = [0.947, 5.015, 4.95, 1.144, 11.425, 4.870, 7.281]
    arm.move_to_joint_pose(jointTarget)
    rospy.sleep(3)
    transform = tfBuffer.lookup_transform('linear_actuator_link','right_ee_link',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
    pt.pose.orientation.w = 1.0
    pt.pose.orientation.x = 0.0
    pt.pose.orientation.y = 0.0
    pt.pose.orientation.z = 0.0
    pt.pose.position.z += 0.1
    return pt.pose


def acitve_var_learning_agent():
    global frames_client, features, arm
    rospy.init_node('acitve_var_learning_agent')

    env = collision_objects()
    env.publish_collision_objects()
    
    home_pose = arm_homing()

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, update_marker_pose)
    rospy.wait_for_service('broadcast_object_frames')
    frames_client = rospy.ServiceProxy("broadcast_object_frames", BroadcastObjectFrames)
    rospy.wait_for_service('active_var')
    active_var_client = rospy.ServiceProxy("active_var", ActiveVaRQuery )

    
    if not construct_world():
        rospy.logerr("Failed to construct environment")
        return 
    
    rospy.loginfo('Redeay to collect visual demos')
    iteration = 0

    demonstrations = []
    init_states = [7,8,9,10,11,12,13,14,15,16,17,18,23,25,26,27,28]
    while True:
        demos = collect_visual_demo(typed=True)
        for demo in demos:
            sa = StateAction()
            sa.state = demo[0]
            sa.action = demo[1]
            demonstrations.append(sa)
        active_var_request = ActiveVaRQueryRequest()
        active_var_request.demonstration = demonstrations
        active_var_request.width = GRID_WORLD_WIDTH
        active_var_request.height = GRID_WORLD_HEIGHT
        active_var_request.num_features = 3
        active_var_request.discount = 0.95
        active_var_request.confidence = 100 
        active_var_request.initial_states = init_states
        active_var_request.terminal_states = []
        active_var_request.alpha = 0.95
        active_var_request.epsilon = 0.05
        active_var_request.delta = 0.1
        active_var_request.state_features = [FloatVector(features[i]) for i in range(GRID_WORLD_WIDTH*GRID_WORLD_HEIGHT)]
        response = active_var_client(active_var_request)
        iteration += 1
        # execute current policy
        curr_policy = response.policy
        raw_input('Ready to execute current policy, press Enter to continue...')
        cup_state = grasp_cup()
        state_waypoints = [cup_state]
        traj_length = 0
        while curr_policy[cup_state] != ACTIONS['STAY'] and traj_length < 8:
            cup_state = get_next_state(cup_state,curr_policy[cup_state])
            print "next state ", cup_state
            if cup_state == None:
                print "NONE state transition?"
                break
            state_waypoints.append(cup_state)
            traj_length += 1
        move_through_states(state_waypoints, home_pose)
        arm_homing()
        print "Status: ", response.status
        if response.status == 'Done':
            break
        print "Iteration", iteration, " query:", response.query_state
        point_at_state(response.query_state)
        '''call('rostopic pub -1 /tilt_controller/command std_msgs/Float64 "data: 0.0"',shell=True)
        gripper.open(100)
        call('rostopic pub -1 /tilt_controller/command std_msgs/Float64 "data: 0.7"',shell=True)'''
        gripper.open(100)
        arm_homing()
        
        

if __name__ == "__main__":
    acitve_var_learning_agent()
