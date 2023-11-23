import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import radians
import math

PHYSICAL_SPEED = 0.05
SIMULATED_SPEED = 0.1

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
        self.lastMsg = None

    def laser_callback(self,msg):
        boundary=len(msg.ranges) // 12
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[-1]
        self.laser_frontLeft = min(msg.ranges[0:boundary])
        self.laser_frontRight = min(msg.ranges[-boundary:])
        self.leftDistance = msg.ranges[len(msg.ranges) // 4]
        self.rightDistance = msg.ranges[len(msg.ranges) // 4 * 3]
        self.backDistance = msg.ranges[len(msg.ranges) // 4 * 3]
        self.lastMsg = msg.ranges

    def rangeFromCenter(self, degrees):
        if self.lastMsg:
            range = (len(self.lastMsg) * (degrees) // 360) // 2
            self.log(f"calc range: {len(self.lastMsg)} {range}")
            return self.lastMsg[-range:] + self.lastMsg[0:range]
        return None


    def minRangeFromCenter(self, degrees):
        range = self.rangeFromCenter(degrees)
        if range:
            return min(range)
        return None

    def avgRangeFromCenter(self, degrees):
        range = self.rangeFromCenter(degrees)
        if range:
            return sum(range) / len(range)
        return None

    def calculateAngularRequiredFor(self, degrees):
        #return 1.0
        return radians(degrees) * (1 / self.timer_period)

    def log(self, msg):
        self.get_logger().info(str(msg))

    def turn(self, degrees):
        radians = self.calculateAngularRequiredFor(degrees)
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = radians
        self.log(f"Turning {radians}")  
    
    def turnLeft(self, degrees):
        self.turn(degrees)
        self.log("Turning left")

    def turnRight(self, degrees):
        self.turn(- degrees)
        self.log("Turning right")
    
    def goForward(self, speed):
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0.0
    
    def go(self):
        self.publisher_.publish(self.cmd)

    def logDistances(self):
        self.log(f"Forward: {self.laser_forward}")
        self.log(f"Front-left {self.laser_frontLeft}")
        self.log(f"Front-right {self.laser_frontRight}")
        self.log(f"left {self.leftDistance}")
        self.log(f"right {self.rightDistance}")

    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

    def motion(self):
        # Logic of move

        self.logDistances()
        #forward = self.laser_forward
        forward = self.minRangeFromCenter(5)
        self.log(f"new forward: {forward}" )

        for i in range(1,12):
            self.log(f"avg for {i * 10} ->  {self.avgRangeFromCenter(i * 10)}" )

        if forward is None:
            self.stop()
            return

        self.log("aaaa")

        if forward > 5:
            self.goForward(self.speed)
        elif forward >= 0.5:
            self.goForward(self.speed / 2)
        elif forward >= 0.3:
            self.goForward(self.speed / 4)
        else:
            if math.isnan(self.leftDistance):
                self.turnRight(5)
            elif math.isnan(self.rightDistance):
                self.turnLeft(5)

            #if self.leftDistance > self.rightDistance:
            if self.laser_frontLeft < self.laser_frontRight:
                self.turnRight(40)
            else:
                self.turnLeft(40)

        self.go()
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    patrol = Patrol(SIMULATED_SPEED)       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(patrol)
    # Explicity destroy the node
    patrol.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()