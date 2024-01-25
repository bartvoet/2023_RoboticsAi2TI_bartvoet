import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import radians
import math
from example_interfaces.srv import SetBool
from nav_msgs.msg import Odometry
from example_interfaces.msg import String

PHYSICAL_SPEED = 0.08
SIMULATED_SPEED = 0.1

class  Prep_lidar(Node):

    def __init__(self, speed):
        super().__init__('prep_lidar')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 
                                                   '/scan', 
                                                   self.laser_callback, 
                                                   QoSProfile(depth=10, 
                                                              reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        self.odomSubscriber = self.create_subscription(Odometry, '/odom', self.odom_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.timer_period = 0.50
        self.speed = speed
        self.navigator = Navigation(self.publisher_, self.timer_period, self.get_logger())
        self.timer = self.create_timer(self.timer_period, self.motion)
        self.patrolEngine = None
        self.lidar = None
        self.service_ = self.create_service(SetBool, "activate_robot", self.callback_activate_robot)
        
        self.odom = Odometry() 
        self.starting_position_x = None
        self.orientation = 1
        self.position_x = 0
        self.position_y = 0
        self.coordinate = (0,0)
        
        self.eventPublisher = self.create_publisher(String, 'patrolEvents', 10)
        
    def publishEvent(self, eventText):
        self.log(f"Sending event: {eventText}")
        msg = String()
        msg.data = eventText
        self.eventPublisher.publish(msg)
        
    def odom_callback(self,msg):
        self.orientation = msg.pose.pose.orientation.w
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

        if self.starting_position_x is None:
            self.starting_position_x = msg.pose.pose.position.x
        self.distance_traveled = abs(msg.pose.pose.position.x - self.starting_position_x)

        return msg.pose.pose.orientation.w
    
    def callback_activate_robot(self, request, response):
        self.activated_ = request.data
        response.success = True
        if self.activated_:
            response.message = "Robot has been activated"
        else:
            response.message = "Robot has been deactivated"
        return response

    def laser_callback(self,msg):
        self.lidar = Lidar(msg,self. get_logger(), 60)
        
        if self.patrolEngine:
            self.patrolEngine.lidar = self.lidar
        else:
            self.patrolEngine = Balancer(self.navigator, self.lidar, self.get_logger())

    def log(self, msg):
        self.get_logger().info(str(msg))
        
    def isFarEnoughFromPreviousPoint(self):
        minDistance = 0.4
        x, y = self.coordinate
        return abs(self.position_x - x) > minDistance or abs(self.position_y - y) > minDistance
    
    def nowWayFurther(self):
        return self.lidar.laser_forward < 0.45 and self.lidar.leftDistance < 0.5 and self.lidar.rightDistance < 0.5
 
    def performUTurn(self):
        self.navigator.turnLeft(180)
        self.coordinate = (self.position_x, self.position_y)
        self.publishEvent(f"U-turn at {self.coordinate}")
        
    def logCoordinate(self):
        self.coordinate = (self.position_x, self.position_y)
        
    def performRightTurn(self):    
        self.navigator.turnRight(90)
        self.logCoordinate()
        self.publishEvent(f"Turning at {self.coordinate}")
        
    def performLeftTurn(self):    
        self.navigator.turnLeft(90)
        self.logCoordinate()
        self.publishEvent(f"Turning at {self.coordinate}")
        
    def isThereAGapToTheRight(self):
        scanRangeClose = 30
        entryRange = 45
        return self.checkSide(self.lidar.getRangesAroundRight(scanRangeClose), 
                              self.lidar.getRangesAroundRight(entryRange))
    
    def isThereAGapToTheLeft(self):
        scanRangeClose = 30
        entryRange = 45
        return self.checkSide(self.lidar.getRangesAroundLeft(scanRangeClose), 
                              self.lidar.getRangesAroundLeft(entryRange))

    def motion(self):
        if self.lidar is None:
            return
        self.lidar.logDistances()
        self.log(f"x = {self.position_x}, y = {self.position_y}")
        if self.patrolEngine:
            if self.nowWayFurther() and self.isFarEnoughFromPreviousPoint():
                self.performUTurn()
            elif self.isThereAGapToTheRight() and self.isFarEnoughFromPreviousPoint():
                self.performRightTurn()
            # elif self.checkSide(self.lidar.getRangesAroundLeft(scanRangeClose), 
            #                   self.lidar.getRangesAroundLeft(entryRange)) \
            #             and self.isFarEnoughFromPreviousPoint():
            #     self.performLeftTurn()
            else:
                self.patrolEngine.motion()


    def printRange(self, range):
        self.log(list(map(lambda x: round(x,2), range)))

    def cleanRange(self, range):
        maxLength = 3.5 
        return list(map(
            lambda x: maxLength if math.isnan(x) or math.isinf(x) else x,
            range))

    def avg(self, slice):
        return sum(slice) / len(slice)

    def isGap(self, slice):
        thresholdDistance = 0.90
        sortedCopy = slice.copy()
        sortedCopy.sort(reverse=True)
        topTen = sortedCopy[0:11]
        averageTopTen = self.avg(topTen)
        result = list(filter(lambda x: abs(x - averageTopTen) > (averageTopTen * 0.1), topTen))
        
        if averageTopTen < thresholdDistance:
            return False
        
        if result:
            return False
        else:
            return True 
        
    def isEntry(self, entries):
        self.log(f"min: {min(entries)}")
        return min(entries) > 0.4

    def checkSide(self, scan, entry):
        maxLength = 2  
        
        rightSliceList = self.cleanRange(scan)
        entries = self.cleanRange(entry)
        rightSlice = list(map(
            lambda x: maxLength if math.isnan(x) or math.isinf(x) else x,
            rightSliceList))
        
        if(self.isGap(rightSlice)):
            self.log("gap")
            if self.isEntry(entries):
                self.log("entry")
                return True
        else:
            self.log("no gap")
        return False

class Balancer:
    def __init__(self, navigator, lidar,  log, speed=PHYSICAL_SPEED,
                 degrSide=1, adjustSideAt = 0.05, degrFactor=8, slowDownAt = 0.5, stopAndTurnAt = 0.3,
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
        self.turnedRight = False
    
    def log(self, msg):
        self.logger.info(str(msg))

    def motion(self):
        if self.lidar is None or self.navigator is None:
            return

        forward = self.lidar.minRangeFromCenter(self.fowardAngle)

        if forward is None:
            self.navigator.stop()
            return
       
        if forward > self.slowDownAt:
            self.navigator.goForward(self.speed)
            if self.lidar.leftDistance < 0.15:
                self.log(f"l => {self.lidar.leftDistance}")
                self.navigator.turnRight(degrees=self.degrFactor)
            elif self.lidar.rightDistance < 0.15:
                self.log(f"l => {self.lidar.rightDistance}")
                self.navigator.turnLeft(degrees=self.degrFactor)
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
    def __init__(self, msg, logger, rangeDegree):
        self.logger = logger
        if msg:
            self.ranges = msg.ranges
            self.numberOfMeasurements = len(self.ranges)
            boundary=len(self.ranges) // (360 // (rangeDegree // 2)) #30 graden * 2 = 60 graden TODO config
            self.laser_forward = self.ranges[-1]
            self.laser_frontLeft = min(self.ranges[0:boundary])
            self.laser_frontRight = min(self.ranges[-boundary:])
            self.leftDistance = self.ranges[len(self.ranges) // 4]
            self.rightDistance = self.ranges[len(self.ranges) // 4 * 3]
            self.backDistance = self.ranges[len(self.ranges) // 2]
            self.lastMsg = self.ranges
            
    def degreeToRange(self, degree):
        return (len(self.lastMsg) * (degree) // 360)
    
    def getRangesAround(self, baseDegree, around):
        if self.lastMsg:
            rangeAround = self.degreeToRange(around) // 2
            rangeDegreeFrom = self.degreeToRange(baseDegree)
            
            return self.lastMsg[rangeDegreeFrom - rangeAround : rangeDegreeFrom + rangeAround]
        return None
    
    def getRangesAroundLeft(self, around):
        return self.getRangesAround(90, around)

    def getRangesAroundRight(self, around):
        return self.getRangesAround(270, around)
    
    def getRangesAroundBottom(self, around):
        return self.getRangesAround(180, around)
    
    def rangeFromFront(self, degrees):
        if self.lastMsg:
            range = (len(self.lastMsg) * (degrees) // 360) // 2
            #self.log(f"calc range: {len(self.lastMsg)} {range}")
            return self.lastMsg[-range:] + self.lastMsg[0:range]
        return None

    def log(self, msg):
        self.logger.info(str(msg))

    def logDistances(self):
        self.log(f"Forward: {self.laser_forward}")
        self.log(f"Front-left {self.laser_frontLeft}")
        self.log(f"Front-right {self.laser_frontRight}")
        self.log(f"left {self.leftDistance}")
        self.log(f"right {self.rightDistance}")
    
    def dumpAllRanges(self):
        self.log(f"ranges: {self.ranges}")

    def rangeFromCenter(self, degrees):
        if self.lastMsg:
            range = (len(self.lastMsg) * (degrees) // 360) // 2
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
    patrol = Prep_lidar(SIMULATED_SPEED)       
    rclpy.spin(patrol)
    patrol.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()