import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
import numpy as np
import math

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.get_logger().info("controller has been started!!!")

        self.init_variables()

        self.init_publisher()
        self.init_subscriber()

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_variables(self):

        #Actual Pose of the robot
        self.actual_pose = Pose()

        #Target to the robot
        self.goal = Pose2D()
        self.goal.x = 5.000
        self.goal.y = 5.000
        self.goal.theta = 0.0

        #Command to the robot
        self.cmd = Twist()
        self.cmd.linear.x = 0.000
        self.cmd.linear.y = 0.000
        self.cmd.angular.x = 0.000
        self.cmd.angular.y = 0.000

        #controller variables
        self.ro = 0
        self.alfa = 0

        # Max Velocity for the controller
        self.vmax = 3
        self.kw = 2

        #controller error
        self.err = np.array([0,0])

        self.working = "not moving"

    def timer_callback(self):
        
        self.controller()
        
        self.get_logger().info("Erro em x: {errorx:.5f}".format(errorx = self.err[0]))
        self.get_logger().info("Erro em y: {errory:.5f}".format(errory = self.err[1]))
        self.get_logger().info("Turtle {value}".format(value = self.working))

    def init_publisher(self):

	    self.publisher_cmd_vel = self.create_publisher(msg_type=Twist, topic='/main_ns/turtle1/cmd_vel', qos_profile=10)

    def init_subscriber(self):
        self.subscriber_pose = self.create_subscription(msg_type=Pose, topic='/main_ns/turtle1/pose', callback=self.pose_callback, qos_profile=10)
        self.subscriber_goal = self.create_subscription(msg_type=Pose2D, topic='/main_ns/goal', callback=self.goal_callback, qos_profile=10)
        # self.subscriber_goal = self.create_subscription(msg_type=Pose2D, topic='/main_ns/goal', callback=self.goal_callback, qos_profile=10)
    
    def pose_callback(self, pose=Pose):
        
        self.actual_pose.x = pose.x
        self.actual_pose.y = pose.y
        self.actual_pose.theta = pose.theta

    def goal_callback(self, pose2d=Pose2D):
        
        self.goal.x = pose2d.x
        self.goal.y = pose2d.y

    def controller(self):
        
        #calculate distance error
        self.get_error()

        #calculate euclidian dist
        self.ro = math.sqrt(self.err[0]**2 + self.err[1]**2)

        #calc. tilt angle
        self.alfa = math.atan2(self.err[1],self.err[0]) - self.actual_pose.theta
        if self.alfa > math.pi:
            self.alfa -= 2*math.pi
        if self.alfa < -math.pi:
            self.alfa += 2*math.pi

        if abs(self.err[0]) > 0.01 or abs(self.err[1]) > 0.01:
            #calc. v cmd
            self.cmd.linear.x = self.vmax * math.tanh(self.ro)
            self.cmd.linear.y = 0.0
            self.cmd.linear.z = 0.0

            self.cmd.angular.x = 0.0
            self.cmd.angular.y = 0.0
            self.cmd.angular.z = self.kw * self.alfa

            self.working = "moving!!!"

            self.publisher_cmd_vel.publish(self.cmd)
        
        else:
            self.working = "not moving"


    def get_error(self):

        self.err[0] = self.goal.x - self.actual_pose.x
        self.err[1] = self.goal.y - self.actual_pose.y


#-------------------------------------------------------------

	
def main(args=None):

    rclpy.init(args=args)

    ctrl = Controller()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.spin(ctrl)
    node.destroy()
    rclpy.try_shutdown()
	

if __name__ == '__main__':
    main()