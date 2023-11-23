import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import radians

PHYSICAL_SPEED = 0.05
SIMULATED_SPEED = 0.5

class  Patrol(Node):

    def __init__(self, speed):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('patrol')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 
                                                   QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.50
        # define the variable to save the received info
        self.laser_forward = 0
        self.laser_frontLeft = 0
        self.laser_frontRight = 0
        self.speed = speed

        self.leftDistance = 0
        self.rightDistance = 0

        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg):
        boundary=len(msg.ranges) // 8
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[-1]
        self.laser_frontLeft = min(msg.ranges[0:boundary])
        self.laser_frontRight = min(msg.ranges[-boundary:])
        self.leftDistance = msg.ranges[len(msg.ranges) // 4]
        self.rightDistance = msg.ranges[len(msg.ranges) // 4 * 3]
        self.backDistance = msg.ranges[len(msg.ranges) // 4 * 3]

    def calculateAngularRequiredFor(self, degrees):
        #return 1.0
        return radians(degrees) * (1 / self.timer_period)

    def log(self, msg):
        self.get_logger().info(msg)

    def motion(self):
        # print the data
        self.log('Forward: "%s"' % str(self.laser_forward))
        self.log(str(self.laser_frontLeft))
        self.log(str(self.laser_frontRight))
        self.log(f"left {self.leftDistance}")
        self.log(f"right {self.rightDistance}")
        
        # Logic of move
        if self.laser_forward > 5:
            self.cmd.linear.x = self.speed
            self.cmd.angular.z = 0.0
        elif self.laser_forward < 2 and self.laser_forward >= 0.4:
            self.cmd.linear.x = self.speed / 2
            self.cmd.angular.z = 0.0         
        else:
            radians = self.calculateAngularRequiredFor(5)
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = radians

            # if self.laser_frontLeft > self.laser_frontRight:
            #     self.cmd.angular.z = - self.cmd.angular.z

            self.log(f"Turning {radians}")
        self.publisher_.publish(self.cmd)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    patrol = Patrol(PHYSICAL_SPEED)       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(patrol)
    # Explicity destroy the node
    patrol.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()