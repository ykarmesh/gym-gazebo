import gym
import rospy
import roslaunch
import time
import numpy as np
import cv2
import sys
import os
import random
import time
import message_filters

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs import OccupancyGrid
from std_srvs.srv import Empty
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from gym.utils import seeding

class GazeboSlamSafeTurtlebotEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboSlamSafeTurtlebot_v0.launch")
        # ORB SLAM launched here
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/karmesh/catkin_ws/src/ORB_SLAM2/launch/orb_monocular.launch"])
        self.launch.start()
        self.launch.spin()

        self.reset_pub = rospy.Publisher('/ORB_SLAM2/Reset',String)
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.goal_pub = rospy.Publisher('/move_base/current_goal', PoseStamped, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        #message filter subscribers
        self.ts_pose_sub = message_filters.Subscriber('/ORB_SLAM2/Pose', PoseStamped)
        self.ts_status_sub = message_filter.Subscriber('/ORB_SLAM2/Status', Bool)
        self.ts_costmap_sub = message_filter.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid)

        ts = message_filters.ApproximateTimeSynchronizer([ts_pose_sub, ts_status_sub, ts_costmap_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(callback)
        #rospy.spin()

        self.reward_range = (-np.inf, np.inf) # NS Range itna kyun rakha hai 
        #self.reward_range = (-1, 1)

        low = -np.ones(2)
        high = -np.ones(2)
        self.action_space = spaces.Box(low, high)

        self.obs_dim = 296; #should be 2(v,w) + 8*8*3(bins(8*8) x features information(no, avg depth, quality)) + 2(goal in polar coor) + 10*10(occupancy grid) 
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        self._seed()  # Check seeding here
    	
        self.prev_cmd_vel = Twist()
    	
        self.goal_limit = 0.2


    def goal_observation(self, data):
    	if ((data.x - self.goal.x)**2 + (data.y - self.goal.y)**2)**(1/2) < self.goal_limit:
    		done = True
    	else:
    		done = False
            return done

    def callback(self, pose, status, occupancygrid)
    	self.goal_done = self.goal_observation(pose)
    	self.dist = self.perpen_distance(pose)
    	self.costmap_data = occupancygrid.data
    	self.breakage = status
        #costmap is publishing slowly, also add the information from the point cloud

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    def _close(self):
        self.launch.shutdown()
        # Kill gzclient, gzserver and roscore
        tmp = os.popen("ps -Af").read()
        gzclient_count = tmp.count('gzclient')
        gzserver_count = tmp.count('gzserver')
        roscore_count = tmp.count('roscore')
        rosmaster_count = tmp.count('rosmaster')

        if gzclient_count > 0:
            os.system("killall -9 gzclient")
        if gzserver_count > 0:
            os.system("killall -9 gzserver")
        if rosmaster_count > 0:
            os.system("killall -9 rosmaster")
        if roscore_count > 0:
            os.system("killall -9 roscore")

        if (gzclient_count or gzserver_count or roscore_count or rosmaster_count >0):
            os.wait()


    def perpen_distance(self, pose):
    	s_x = self.start.pose.position.x
    	s_y = self.start.pose.position.y
    	s_x = self.goal.pose.position.x
    	s_y = self.goal.pose.position.y
    	p_x = pose.pose.position.x
    	p_y = pose.pose.position.y
    	distance = abs((s_x - g_x)*p_y - (s_y - g_y)*p_x + s_y*g_x - g_y*s_x)/((s_x - g_x)**2 + (s_y - g_y)**2)**(1/2)
    	return distance

    def _step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)

        '''data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass
        done = self.calculate_observation(data)

        self.ob = self.take_observation()
        while(self.ob is None):
            self.ob = self.take_observation()'''

    	goal_done_ = self.goal_done
    	dist_ = self.dist
    	costmap_data = self.costmap_data
    	breakage = self.breakage

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
	

        # Define reward
        if not breakage_:
	    if not goal_done_:
                if dist_ > 2:
                    reward = 0.1
                else:
                    reward = 0.1*dist_/2
		done = False
	    else:
                reward = 1
		done = True
        else:
            reward = -1
	    done = True

    	state = [prev_cmd_vel.linear.x, prev_cmd_vel.angular.z, distance, costmap_data]
        return state, reward, done, {}


    def _reset(self):
	
    	reset = String()
    	reset.data = "r"
    	self.orb_reset.publish(reset)
    	rospy.sleep(1)
            # Resets the state of the environment and returns an initial observation.
            rospy.wait_for_service('/gazebo/reset_simulation')
            try:
                #reset_proxy.call()
                self.reset_proxy()
            except (rospy.ServiceException) as e:
                print ("/gazebo/reset_simulation service call failed")

    	self.goal = PoseStamped()
    	self.goal.pose.position.x = random.random() * self.map_size_x
    	self.goal.pose.position.y = random.random() * self.map_size_y
    	self.goal_pub.publish(self.goal)

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

    	#try to initialize SLAM
    	vel_cmd = Twist()
            vel_cmd.linear.x = -0.4
    	r = rospy.Rate(10)
    	status = False
    	count = 0
    	while status is False and count < 5:
    	    t = time.time()
    	    if count%3 is 0
    		vel_cmd.linear.x = -vel_cmd.linear.x
    	    while time.time() - t < 0.15:
    		self.vel_pub.publish(vel_cmd)
    		r.sleep()
    	    rospy.sleep(0.5)
    	    status = rospy.wait_for_message('/ORB_SLAM2/Status', PoseStamped, timeout=5)
    	    count += 1
    	    # what will happen if SLAM is still not initialized

    	#wait for observation
    	self.start = None
            while self.start is None:
                try:
                    self.start = rospy.wait_for_message('/ORB_SLAM2/Pose', PoseStamped, timeout=5)
                except:
                    pass

    	# pause simulation
            rospy.wait_for_service('/gazebo/pause_physics')
            try:
                #resp_pause = pause.call()
                self.pause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/pause_physics service call failed")


            state = 
            return state
