import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import radians
import math

PHYSICAL_SPEED = 0.08
SIMULATED_SPEED = 0.1

class  Practical_test(Node):

    def __init__(self, speed):
        super().__init__('patrol')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 
                                                   '/scan', 
                                                   self.laser_callback, 
                                                   QoSProfile(depth=10, 
                                                              reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        
        self.timer_period = 0.50
        self.speed = speed
        self.navigator = Navigation(self.publisher_, self.timer_period, self.get_logger())
        self.timer = self.create_timer(self.timer_period, self.motion)
        self.patrolEngine = None
        self.lidar = None
        self.wait = True

    def laser_callback(self,msg):
        self.lidar = Lidar(msg,self. get_logger())
        self.patrolEngine = PatrolEngine(self.navigator, self.lidar, self.get_logger())

    def log(self, msg):
        self.get_logger().info(str(msg))

    def motion(self):
        if self.lidar is None:
            return

        if self.wait:
            self.navigator.stop()
            if not self.lidar.isBlockedBehind():
                self.log("not blocked behind")
                return
            else:
               self.log("blocked behind")
               self.wait = False
        
        # if self.lidar.isBlockedBehind() and self.lidar.isBlockedFront():
        #     self.wait = True
        #     self.log("stop...")
        #     self.navigator.stop()

        if self.patrolEngine is not None:
            self.patrolEngine.motion()

class PatrolEngine:
    def __init__(self, navigator, lidar,  log, speed=PHYSICAL_SPEED,
                 degrSide=1, adjustSideAt = 0.05, degrFactor=8, slowDownAt = 0.5, stopAndTurnAt = 0.2,
                    fowardAngle = 55 ):
        self.navigator = navigator
        self.lidar = lidar
        self.logger = log
        self.speed = speed
        self.degrFactor = degrFactor
        self.degrSide = degrSide
        self.slowDownAt = slowDownAt
        self.adjustSideAt = adjustSideAt
        self.stopAndTurnAt = stopAndTurnAt
        self.fowardAngle = fowardAngle
    
    def log(self, msg):
        self.logger.info(str(msg))

    def motion(self):
        if self.lidar is None or self.navigator is None:
            return

        #self.lidar.logDistances()
        forward = self.lidar.minRangeFromCenter(self.fowardAngle)
        #self.log(f"new forward: {forward}" )

        # for i in range(1,12):
        #     self.log(f"avg for {i * 10} ->  {self.lidar.avgRangeFromCenter(i * 10)}" )

        if forward is None:
            self.navigator.stop()
            return
        
        # if math.isnan(self.lidar.leftDistance):
        #     self.navigator.turnRight(degrees=self.degrSide)
        # elif math.isnan(self.lidar.rightDistance):
        #     self.navigator.turnLeft(degrees=self.degrSide)

        if forward > self.slowDownAt:
            self.navigator.goForward(self.speed)
        elif forward >= self.stopAndTurnAt:
            self.navigator.goForward(self.speed / 2)
        else:
            #if self.lidar.leftDistance > self.lidar.rightDistance:
            if self.lidar.laser_frontLeft < self.lidar.laser_frontRight:
                self.navigator.turnRight(degrees=self.degrFactor)
            else:
                self.navigator.turnLeft(degrees=self.degrFactor)

class Navigation:
    def __init__(self, publisher, timer_period, logger):
        self.timer_period = timer_period
        self.logger = logger
        self.cmd = Twist()
        self.publisher = publisher
    
    def calculateAngularRequiredFor(self, degrees):
        return radians(degrees) * (1 / self.timer_period)

    def log(self, msg):
        self.logger.info(str(msg))

    def turn(self, degrees):
        radians = self.calculateAngularRequiredFor(degrees)
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = radians
        self.log(f"Turning {radians}")
        self.go()  
    
    def turnLeft(self, degrees):
        self.turn(degrees)
        self.log("Turning left")

    def turnRight(self, degrees):
        self.turn(- degrees)
        self.log("Turning right")
    
    def goForward(self, speed):
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0.0
        self.go()
    
    def go(self):
        self.publisher.publish(self.cmd)

    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.go()


class Lidar:
    def __init__(self, msg, logger):
        self.logger = logger
        if msg:
            ranges = msg.ranges
            boundary=len(ranges) // 12 #30 graden * 2 = 60 graden TODO config
            self.laser_forward = ranges[-1]
            self.laser_frontLeft = min(ranges[0:boundary])
            self.laser_frontRight = min(ranges[-boundary:])
            self.leftDistance = ranges[len(ranges) // 4]
            self.rightDistance = ranges[len(ranges) // 4 * 3]
            self.backDistance = ranges[len(ranges) // 2]
            self.lastMsg = ranges
    
    def isBlockedBehind(self):
        blocked = True
        middle = len(self.lastMsg) // 2
        r = 2
        for i in range(middle - r,middle + r):
            if (not math.isnan(self.lastMsg[i])):
                blocked = False
        return blocked
    
    def isBlockedFront(self):
        r = 15
        for i in self.lastMsg[-r:] + self.lastMsg[0:r]:
            self.log(f"front {i}")
            #if (not math.isnan(i)):
            if not math.isnan(i) and i > 1.0:
                return False
        return True

    def log(self, msg):
        self.logger.info(str(msg))

    def logDistances(self):
        self.log(f"Forward: {self.laser_forward}")
        self.log(f"Front-left {self.laser_frontLeft}")
        self.log(f"Front-right {self.laser_frontRight}")
        self.log(f"left {self.leftDistance}")
        self.log(f"right {self.rightDistance}")

    def rangeFromCenter(self, degrees):
        if self.lastMsg:
            range = (len(self.lastMsg) * (degrees) // 360) // 2
            #self.log(f"calc range: {len(self.lastMsg)} {range}")
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


def main(args=None):
    rclpy.init(args=args)
    patrol = Practical_test(SIMULATED_SPEED)       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(patrol)
    # Explicity destroy the node
    patrol.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()