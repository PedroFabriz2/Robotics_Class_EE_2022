
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
import numpy as np
import math
from nav_msgs.msg import Odometry

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

        self.iterator = 0

        #Actual Pose of the robot
        self.actual_pose = Pose()

        #Target to the robot
        self.goal = Pose()
        self.goal.x = 0.0
        self.goal.y = 0.0
        self.goal.theta = 0.0

        #Command to the robot
        self.cmd = Twist()
        self.cmd.linear.x = 0.000
        self.cmd.linear.y = 0.000
        self.cmd.linear.z = 0.000

        self.cmd.angular.x = 0.000
        self.cmd.angular.y = 0.000
        self.cmd.angular.z = 0.000

        #controller variables
        self.ro = 0
        self.alfa = 0

        # Max Velocity for the controller
        self.vmax = 0.5
        self.kw = 2

        #controller error
        self.err = np.array([0,0])

        # Array of Target positions for turtlebot (Gazebo + nav2 Lab5)
        self.x_array = [1.0, 0.6, 0.0, 0.0, 0.5, 0.6]
        self.y_array = [0.5, 0.5, 0.2, 0.5, 1.0, 1.1]

        self.working = "not moving"

    def timer_callback(self):
        
        self.controller()
        
        self.get_logger().info("Erro em x: {errorx:.5f}".format(errorx = self.err[0]))
        self.get_logger().info("Erro em y: {errory:.5f}".format(errory = self.err[1]))
        self.get_logger().info("Turtle {value}".format(value = self.working))

    def init_publisher(self):
	    self.publisher_cmd_vel = self.create_publisher(msg_type=Twist, topic='/cmd_vel', qos_profile=10)


    def init_subscriber(self):
        self.subscriber_pose = self.create_subscription(msg_type=Odometry, topic='/odom', callback=self.pose_callback, qos_profile=10)
    

    def pose_callback(self, odometry_msg=Odometry):
        
        # x and y
        self.actual_pose.x = odometry_msg.pose.pose.position.x
        self.actual_pose.y = odometry_msg.pose.pose.position.y

        # orientation in free-space in quaternion form
        ori_ = odometry_msg.pose.pose.orientation.z*odometry_msg.pose.pose.orientation.w
        
        #transformation 
        self.actual_pose.theta = math.atan2(2*ori_, 1 - 2*ori_)
        

    def controller(self):

        
        while self.iterator < 6:

            self.goal.x = self.x_array[self.iterator]
            self.goal.y = self.y_array[self.iterator]

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

            #monitor if the goal was reached and update the values with the vector of positions
            if abs(self.err[0]) > 0.1 or abs(self.err[1]) > 0.1:
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
                self.iterator += 1
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
