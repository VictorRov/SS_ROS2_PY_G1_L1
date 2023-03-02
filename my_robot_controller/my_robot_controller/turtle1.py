
import ptvsd
ptvsd.enable_attach(address=('localhost', 5678))
import rclpy
import time
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3
from turtlesim.srv import Spawn

from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import Kill
from math import atan2, pi, sqrt

class TurtleControl(Node):
    def __init__(self, turtle_name):
           # Creates a node with name 'turtlebot_controller' and make sure it is a
           # unique node (using anonymous=True).
           super().__init__('turtle_' + turtle_name)
   
           # Publisher which will publish to the topic '/turtle1/cmd_vel'.
           self.velocity_publisher = self.create_publisher(Twist, '/' + turtle_name + '/cmd_vel', 100)
   
           # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
           # when a message of type Pose is received.
           self.pose_subscriber = self.create_subscription(Pose, '/' + turtle_name + '/pose', self.update_pose, 100)
   
           self.pose = Pose()

           self.rate = self.create_rate(10)


    def set_pose(sefl, p):
        sefl.pose.x = p[0]
        sefl.pose.y = p[1]

    def update_pose(self, data):
        self.pose = data
        print('xose='),
        print(data)
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
   
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))
   
    def linear_vel(self, goal_pose):      
        velocity = Vector3()
        velocity.x = goal_pose.x - self.pose.x
        velocity.y = goal_pose.y - self.pose.y
        return velocity
   
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
   
    def angular_vel(self, goal_pose, constant=0):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
    
    
    def wait_until_ready(self,goal_pose):
        while not hasattr(self, 'pose') or self.pose is None:
            rclpy.spin_once(self)
        de = self.euclidean_distance(goal_pose)
        while  de > 0.1: 
            rclpy.spin_once(self)

        rclpy.spin_once(self, timeout_sec=10)
        print("Ready to move!")

    def move2XY(self, goal_pose):
        vel_msg = Twist()
        vel_msg.linear = self.linear_vel(goal_pose)
        
               # Angular velocity in the z-axis.
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
               # Publishing our vel_msg
      
    
        self.velocity_publisher.publish(vel_msg)
        time.sleep(1)
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        self.pose = goal_pose

           
def main(args=None):
    rclpy.init(args=args)
    tpose = Pose()
    turtleSetup()


   
    turtle1 = TurtleControl('turtle1')
    turtle2 = TurtleControl('turtle2')
     

    pointsV = [(2.0,5.0), (3.0, 1.0), (4.0, 5.0)] 
    pointsR = [(5.0,1.0), (5.0, 5.0), (6.5, 5.0), (7.0, 4.5), (7.0, 3.5), (6.5, 3.0),(5.0, 3.0),(7.0, 1.0)]

    turtle1.set_pose(pointsV[0])
    
    turtle2.set_pose(pointsR[0])

    for point in pointsV:
        goal_pose = Pose()
        goal_pose.x, goal_pose.y = point
        goal_pose.theta = 0.0
        turtle1.move2XY(goal_pose)

    for point in pointsR:
        goal_pose = Pose()
        goal_pose.x, goal_pose.y = point
        goal_pose.theta = 0.0
        turtle2.move2XY(goal_pose)

    turtle1.destroy_node()
    turtle2.destroy_node()

def turtleSetup():
    node = rclpy.create_node('setup_turtles')
    
    # Spawn two turtles
    spawn_client = node.create_client(Spawn, 'spawn')
    spawn_request = Spawn.Request()
    spawn_request.x = 2.0
    spawn_request.y = 5.0
    spawn_request.theta = 0.0
    spawn_request.name = 'turtle1'
    spawn_future = spawn_client.call_async(spawn_request)
    rclpy.spin_until_future_complete(node, spawn_future)
    turtle1_name = spawn_future.result().name
   
    spawn_request.x = 5.0
    spawn_request.y = 1.0
    spawn_request.theta = 0.0
    spawn_request.name = 'turtle2'
    spawn_future = spawn_client.call_async(spawn_request)
    rclpy.spin_until_future_complete(node, spawn_future)
    turtle2_name = spawn_future.result().name
    
    # Set initial velocities to stop turtles
    vel_pub1 = node.create_publisher(Twist, f'{turtle1_name}/cmd_vel', 10)
    vel_pub2 = node.create_publisher(Twist, f'{turtle2_name}/cmd_vel', 10)
    vel_msg = Twist()

    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub1.publish(vel_msg)
    vel_pub2.publish(vel_msg)

    # Wait for the turtles to spawn
    rclpy.spin_once(node, timeout_sec=1)

    # Get the initial pose of the turtles
    pose_sub1 = node.create_subscription(Pose, f'{turtle1_name}/pose', pose_callback1, 10)
    pose_sub2 = node.create_subscription(Pose, f'{turtle2_name}/pose', pose_callback2, 10)
    pose1 = Pose()
    pose2 = Pose()

    # Cleanup
    vel_pub1.destroy()
    vel_pub2.destroy()
    pose_sub1.destroy()
    pose_sub2.destroy()
    node.destroy_node()


def pose_callback1(msg):
    global pose1
    pose1 = msg

def pose_callback2(msg):
    global pose2
    pose2 = msg


    
if __name__ == '__main__':
    main()
