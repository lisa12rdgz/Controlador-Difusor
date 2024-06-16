# ROS2 module imports
# ros2 service call /spawn turtlesim/srv/Spawn "{x: 5, y: 1, theta: 2.5}"

import rclpy  # ROS2 client library (rcl) for Python (built on rcl C API)
import math
from rclpy.node import Node  # Node class for Python nodes
from geometry_msgs.msg import Twist  # Twist (linear and angular velocities) message class
from turtlesim.msg import Pose  # Turtlesim pose (x, y, theta) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # Quality of Service (tune communication between nodes)
from math import pow, atan2, sqrt  # Common mathematical functions


class RobotController(Node):

    def __init__(self):
        # Information and debugging
        
        # ROS2 infrastructure
        super().__init__('turtle2')  # Create a node with name 'robot_controller'
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.robot_ctrl_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', qos_profile)  # Publisher for Twist message to '/turtle2/cmd_vel'
        self.robot_pose_sub = self.create_subscription(Pose, '/turtle2/pose', self.robot_feedback_callback, qos_profile)  # Subscriber for Pose message from '/turtle2/pose'
        
        timer_period = 0.01  # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback)  # Timer to execute 'robot_controller_callback()' every 'timer_period' seconds

        # Initialize variables
        self.robot_pose = Pose()
        self.robot_flag = False
        self.robot2_pose = Pose()
        self.robot2_flag = False
        self.goal_poses = [
            Pose(x=5.0, y=1.0, theta= math.radians(90)),
            Pose(x=1.0, y=5.0, theta= 0.0),
            Pose(x=5.0, y=10.0 , theta= -1.5),
            Pose(x=10.0, y=5.0, theta= -3.0),
            Pose(x=5.0, y=1.0, theta= math.radians(90))
        ]
        self.goal_index = 0
        self.goal_flag = False
        self.ctrl_msg = Twist()
        self.forward_direction = True  # Flag to indicate forward or reverse direction

    def robot_feedback_callback(self, message):
        '''Robot feedback (pose) callback'''
        self.robot_pose = message  # Capture incoming message (Pose)
        self.robot_flag = True  # Set robot flag to feedback available

    def robot2_feedback_callback(self, message):
        '''Robot2 feedback (pose) callback'''
        self.robot2_pose = message  # Capture incoming message (Pose)
        self.robot2_flag = True  # Set robot2 flag to feedback available

    def robot_controller_callback(self):
        '''Robot controller (twist) callback'''
        if self.robot_flag:
            lin_vel, ang_vel = self.set_robot_controls(pos_tol=0.1, rot_tol=0.01)  # Set robot controls
            
            self.ctrl_msg.linear.x = lin_vel  # Set robot linear velocity
            self.ctrl_msg.angular.z = ang_vel  # Set robot angular velocity
            self.robot_ctrl_pub.publish(self.ctrl_msg)  # Publish robot controls message
            
            if self.goal_flag:
                print('Robot reached specified goal pose: x = {} m, y = {} m, theta = {} rad'.format(
                    self.goal_poses[self.goal_index].x, self.goal_poses[self.goal_index].y, self.goal_poses[self.goal_index].theta))
                
                if self.forward_direction:
                    self.goal_index += 1  # Move to the next goal pose
                else:
                    self.goal_index -= 1  # Move to the previous goal pose

                if self.goal_index == len(self.goal_poses) or self.goal_index < 0:
                    print("All goal poses reached in current direction!")
                    self.forward_direction = not self.forward_direction  # Toggle direction
                    if self.forward_direction:
                        self.goal_index = 0
                    else:
                        self.goal_index = len(self.goal_poses) - 2  # Start from the second last goal

                self.goal_flag = False  # Reset goal flag
            else:
                print('Robot going to specified goal pose: x = {} m, y = {} m, theta = {} rad'.format(
                    self.goal_poses[self.goal_index].x, self.goal_poses[self.goal_index].y, self.goal_poses[self.goal_index].theta))

    def get_position_error(self):
        '''Error in position as Euclidean distance between current pose and goal pose.'''
        return sqrt(pow((self.goal_poses[self.goal_index].x - self.robot_pose.x), 2) + pow((self.goal_poses[self.goal_index].y - self.robot_pose.y), 2))
    
    def get_rotation_error(self):
        '''Error in rotation as relative angle between current pose and goal pose.'''
        return atan2(self.goal_poses[self.goal_index].y - self.robot_pose.y, self.goal_poses[self.goal_index].x - self.robot_pose.x) - self.robot_pose.theta

    def get_linear_velocity(self, gain=1.0):
        '''Compute robot linear velocity using P-controller'''
        return gain * self.get_position_error()

    def get_angular_velocity(self, gain=1.0):
        '''Compute robot angular velocity using P-controller'''
        return gain * self.get_rotation_error()

    def set_robot_controls(self, pos_tol=0.1, rot_tol=0.1):
        '''Set robot controls (twist) based on deviation from goal'''
        if self.get_position_error() > pos_tol:  # Go to goal
            lin_vel = self.get_linear_velocity(gain=1.5)  # Set robot linear velocity
            ang_vel = self.get_angular_velocity(gain=6.0)  # Set robot angular velocity
            return lin_vel, ang_vel
        if abs(self.goal_poses[self.goal_index].theta - self.robot_pose.theta) > rot_tol:  # Orient at goal
            lin_vel = 0.0  # Set robot linear velocity
            ang_vel = self.goal_poses[self.goal_index].theta - self.robot_pose.theta  # Set robot angular velocity
            return lin_vel, ang_vel
        lin_vel = 0.0  # Set robot linear velocity
        ang_vel = 0.3  # Set robot angular velocity
        self.goal_flag = True  # Set goal flag to reached
        return lin_vel, ang_vel

    def check_collision(self):
        distance_threshold = 2
        distance = sqrt(pow((self.robot_pose.x - self.robot2_pose.x), 2) + pow((self.robot_pose.y - self.robot2_pose.y), 2))
        return distance < distance_threshold  
    
    def collisionforsure(self):
        distance_threshold = 1.5
        distance = sqrt(pow((self.robot_pose.x - self.robot2_pose.x), 2) + pow((self.robot_pose.y - self.robot2_pose.y), 2))
        return distance < distance_threshold 

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = RobotController()  # Create an instance of RobotController
    rclpy.spin(node)  # Spin the node so the callback functions are called
    node.destroy_node()  # Destroy the node explicitly
    rclpy.shutdown()  # Shutdown the ROS2 client library

if __name__ == "__main__":
    main()
