import gym
import rospy
import roslaunch
import time
import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
#sys.path.remove('/usr/lib/python2.7/dist-packages')
#sys.path.append('/usr/lib/python2.7/dist-packages')
import cv2
import os
import random
import time
import message_filters
import subprocess
import transforms3d as tf3d

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
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
        '''uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid,["home/rrc/gym-gazebo/gym_gazebo/envs/assets/launch/orb_monocular.launch"])
        self.launch.start()
        self.launch.spin()'''
        self.port = os.environ["ROS_PORT_SIM"]
        subprocess.Popen(["roslaunch","-p", self.port, "/home/rrc/gym-gazebo/gym_gazebo/envs/assets/launch/orb_monocular.launch"])

        self.reset_pub = rospy.Publisher('/ORB_SLAM2/Reset',String, queue_size=1)
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.goal_pub = rospy.Publisher('/move_base/current_goal', PoseStamped, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        #message filter subscribers
        self.ts_pose_sub = message_filters.Subscriber('/ORB_SLAM2/Pose', PoseStamped)
        self.ts_status_sub = message_filters.Subscriber('/ORB_SLAM2/Status', Bool)
        self.ts_costmap_sub = message_filters.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid)

        ts = message_filters.ApproximateTimeSynchronizer([self.ts_pose_sub, self.ts_status_sub, self.ts_costmap_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
        #rospy.spin()

        self.reward_range = (-np.inf, np.inf) # Range itna kyun rakha hai 
        #self.reward_range = (-1, 1)

        low = -np.ones(2)
        high = -np.ones(2)
        self.action_space = spaces.Box(low, high, dtype=np.float32)

        self.obs_dim = 296; #should be 2(v,w) + 8*8*3(bins(8*8) x features information(no, avg depth, quality)) + 2(goal in polar coor) + 10*10(occupancy grid) 
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        self._seed()  # Check seeding here
    	
        self.prev_cmd_vel = Twist()
    	
        self.goal_limit = 0.2
        self.robot_radius = 0.22 #size+padding in meters
        self.obstacle_threshold = 50
        self.map_size_x = 10
        self.map_size_y = 20

        self.goal_done = False
        self.dist = 0
        self.costmap_data = np.ones(100)
        self.breakage = False
        self.collision = False

        self.start = Pose()
        angles = Vector3()
        self.start.position.x = 7
        self.start.position.y = -3
        angles.z = 1.4
        self.start.orientation = self.euler_to_quat(angles)

    def goal_observation(self, data):
        if ((data.x - self.goal.x)**2 + (data.y - self.goal.y)**2)**(1/2) < self.goal_limit:
            done = True
        else:
            done = False
        return done

    def euler_to_quat(self, angles):
        q = tf3d.euler.euler2quat(angles.x, angles.y, angles.z)
        quat = Quaternion()
        quat.w = q[0]
        quat.x = q[1]
        quat.y = q[2]
        quat.z = q[3]
        return quat

    def collision_checker(self, occupancygrid):
        x1 = occupancygrid.info.width/2 - self.robot_radius/occupancygrid.info.resolution
        x2 = occupancygrid.info.width/2 + self.robot_radius/occupancygrid.info.resolution
        y1 = occupancygrid.info.height/2 - self.robot_radius/occupancygrid.info.resolution
        y2 = occupancygrid.info.height/2 + self.robot_radius/occupancygrid.info.resolution
        for i in range(x1,x2):
            for j in range(y1,y2):
                if occupancygrid.data[i+j*occupancygrid.info.width] > self.obstacle_threshold:
                    return True
        return False


    def callback(self, pose, status, occupancygrid):
        self.goal_done = self.goal_observation(pose)
        self.dist = self.perpen_distance(pose)
        self.costmap_data = occupancygrid.data
        self.collision = self.collision_checker(occupancygrid)
        self.breakage = status
        #costmap is publishing slowly, also add the information from the point cloud


    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    def _close(self):
        #self.launch.shutdown()
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
        costmap_data_ = self.costmap_data
        breakage_ = self.breakage
        collision_ = self.collision
        rospy.sleep(1)


        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")


        # Define reward
        if not breakage_:
            if not goal_done_:
                if dist_ > 1:
                    reward = -0.01
                else:
                    reward = -0.01*dist_
                if collision_ == True:
                    reward -= 0.1
                done = False
            else:
                reward = 1
                done = True
        else:
            reward = -1
            done = True

        state = [self.prev_cmd_vel.linear.x, self.prev_cmd_vel.angular.z, dist_, costmap_data_]
        self.prev_cmd_vel = vel_cmd
        return state, reward, done, {}


    def _reset(self):

        state = SetModelState()
        state.model_name = 'mobile_base'
        state.pose = self.start
        state.twist = Twist()
        state.reference_frame = ''
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.set_model_state(state)
        except (rospy.ServiceException) as e:
            print ("/gazebo/set_model_state service call failed")

        reset = String()
        reset.data = "r"
        self.reset_pub.publish(reset)
        rospy.sleep(3)

        self.goal = PoseStamped()
        self.goal.pose.position.x = random.random() * self.map_size_x
        self.goal.pose.position.y = random.random() * self.map_size_y

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
        status = Bool()
        count = 0
        while status.data is False and count < 10:
            t = time.time()
            if count%5 is 0:
                vel_cmd.linear.x = -vel_cmd.linear.x
                rospy.sleep(2)
            while time.time() - t < 2.0:
                self.vel_pub.publish(vel_cmd)
                r.sleep()
            status = rospy.wait_for_message('/ORB_SLAM2/Status', Bool, timeout=5)
            print(status)
            print(vel_cmd.linear.x)
            count += 1
            rospy.sleep(1)
                # what will happen if SLAM is still not initialized

        if status is False:
            state = self._reset()
            return state

        self.goal_pub.publish(self.goal)

        goal_done_ = self.goal_done
        dist_ = self.dist
        costmap_data_ = self.costmap_data
        breakage_ = self.breakage
        collision_ = self.collision

        # pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = [self.prev_cmd_vel.linear.x, self.prev_cmd_vel.angular.z, dist_, costmap_data_]

        return state
