'''
GPT Prompt - 
I am creating an RL based model for local motion planning in a ROS navigation stack.
My idea is to sample a few poses from the global path and use them as a part of the observation space.    
I want my action space to be the linear and angular acceleration limits of the robot.
Below is what I have done so far.

'''

import math
import gym
from gym import spaces
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

'''
Welcome to DopamiNav ~
The perfect environment to learn to make good short-term decisions, based on long-term goals.
'''

class SimplePose:
    '''
    For 2D navigation, we do not need the 7 valued pose. 
    We just need x, y, and the sin and cos of the orientation - this simplifies the observation space for our RL model.
    '''
    def __init__(self, x=0, y=0, sin=0, cos=1): 
        self.x = x
        self.y = y
        self.sin = sin
        self.cos = cos
    
    def get_pose(self, PoseStamped):
        self.x = PoseStamped.pose.position.x
        self.y = PoseStamped.pose.position.y
        #simplified way of getting sin and cos, as qx and qy will always be 0
        z = PoseStamped.pose.orientation.z
        w = PoseStamped.pose.orientation.w
        self.sin = 2 * (w*z)
        self.cos = math.sqrt(1 - self.sin**2)
    
    def get_rosPoseStamped(self):
        pose = PoseStamped()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0
        
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = self.sin/2
        pose.pose.orientation.w = self.cos
        return pose


class DopamiNav(gym.Env):
    def __init__(self):
        super(DopamiNav, self).__init__()

        # Initialize ROS node
        rospy.init_node('rl_local_planner', anonymous=True)       

        # Define action and observation space
        self.define_observation_space()
        self.define_action_space()


    def define_observation_space(self):
        #Parameters for observation space
        self.n_samples = rospy.get_param('~n_samples', 3)
        self.local_window = rospy.get_param('~local_window', 30)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.global_path_callback)
        # rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.global_path_callback)

        self.lin_vel = 0
        self.ang_vel = 0 
        self.current_pose = None
        self.goal_poses = self.sample_poses()

        # Define observation space
        size_lin_vel = 1
        size_ang_vel = 1
        #each goal pose has 4 values - x, y, sin, cos
        size_current_pose = 4
        size_goal_poses = 4 * self.n_samples

        total_size = size_lin_vel + size_ang_vel + size_current_pose + size_goal_poses
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(total_size,), dtype=np.float32)


    def define_action_space(self):
        #Parameters for action space
        self.linear_acc_limits = rospy.get_param('~linear_acc_limits', [-0.5, 0.5])
        self.angular_acc_limits = rospy.get_param('~angular_acc_limits', [-0.5, 0.5])
        
        # Define action space
        self.action_space = spaces.Box(low=np.array([self.linear_acc_limits[0], self.angular_acc_limits[0]]), 
                                       high=np.array([self.linear_acc_limits[1], self.angular_acc_limits[1]]), 
                                       dtype=np.float32) 
        
        # Pubilshers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def global_path_callback(self, data):
        self.global_path = data
        
    def sample_poses(self):
        total_poses = len(self.global_path.poses)
        num_poses_to_sample = min(self.local_window, total_poses)
        
        interval = max(1, num_poses_to_sample // self.n_samples)
        
        poses = []
        for i in range(0, num_poses_to_sample, interval):
            s = SimplePose()
            s.get_pose(self.global_path.poses[i])
            poses.append(s)
            
            # Break the loop if you've collected enough samples
            if len(poses) >= self.n_samples:
                break
            
        return poses
    
    def step(self):
        pass

    def reset(self):
        pass
    
    def render(self):
        pass


        
        
