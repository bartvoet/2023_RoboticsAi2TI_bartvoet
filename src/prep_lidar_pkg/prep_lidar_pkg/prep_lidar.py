import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import radians
import math
from example_interfaces.srv import SetBool

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
        
        self.timer_period = 0.50 # 0.50 * 2 # * 10
        self.speed = speed
        self.navigator = Navigation(self.publisher_, self.timer_period, self.get_logger())
        self.timer = self.create_timer(self.timer_period, self.motion)
        self.patrolEngine = None
        self.lidar = None
        self.wait = False
        self.turnedRecently = False
        self.service_ = self.create_service(SetBool, "activate_robot", self.callback_activate_robot)
        self.turnedRecently = False
    
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
            self.patrolEngine = PatrolEngine(self.navigator, self.lidar, self.get_logger())
            

    def log(self, msg):
        self.get_logger().info(str(msg))

    def motion(self):
        if self.lidar is None:
            return

        if self.patrolEngine:
            scanRangeClose = 30
            entryRange = 45

            if self.checkSide(self.lidar.getRangesAroundRight(scanRangeClose), self.lidar.getRangesAroundRight(entryRange)) and not self.turnedRecently:
                self.navigator.turnRight(90)
                self.log("------turning-------")
                self.turnedRecently = True
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
        self.log(topTen)
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
        self.turnedRight = False
    
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

    def motionn(self):
        if self.lidar is None or self.navigator is None:
            return

        #self.lidar.logDistances()
        #self.log(f"new forward: {forward}" )

        # for i in range(1,12):
        #     self.log(f"avg for {i * 10} ->  {self.lidar.avgRangeFromCenter(i * 10)}" )

        
        # if math.isnan(self.lidar.leftDistance):
        #     self.navigator.turnRight(degrees=self.degrSide)
        # elif math.isnan(self.lidar.rightDistance):
        #     self.navigator.turnLeft(degrees=self.degrSide)

        # if forward > self.slowDownAt:
        #     self.navigator.goForward(self.speed)
        # elif forward >= self.stopAndTurnAt:
        #     self.navigator.goForward(self.speed / 2)
        # else:
        #     #if self.lidar.leftDistance > self.lidar.rightDistance:
        #     if self.lidar.laser_frontLeft < self.lidar.laser_frontRight:
        #         self.navigator.turnRight(degrees=self.degrFactor)
        
        
        thresholdDistance = 0.90
        scanRange = 90      
        
        rightSliceList = self.lidar.getRangesAroundRight(30)
        rightSliceLen = len(rightSliceList)
        rightSlice = map(lambda x: 2 if math.isnan(x) else x,
            self.lidar.getRangesAroundRight(scanRange))
        
        rightAverage = sum(rightSlice) / rightSliceLen
        
        self.log(f"{rightAverage}")
        
        forward = self.lidar.minRangeFromCenter(self.fowardAngle)
        #self.log(f"new forward: {forward}" )

        # for i in range(1,12):
        #     self.log(f"avg for {i * 10} ->  {self.lidar.avgRangeFromCenter(i * 10)}" )

        if forward is None:
            self.navigator.stop()
            return
        
        if rightAverage > thresholdDistance and not self.turnedRight:
            self.log("going right")
            self.navigator.stop()
            self.navigator.turnRight(degrees=90)
            self.turnedRight = True
        else:
            self.navigator.goForward(self.speed / 2)
            # if forward > self.slowDownAt:
            #     self.navigator.goForward(self.speed)
            # elif forward >= self.stopAndTurnAt:
            #     self.navigator.goForward(self.speed / 2)
            # else:
            #     #if self.lidar.leftDistance > self.lidar.rightDistance:
            #     if self.lidar.laser_frontLeft < self.lidar.laser_frontRight:
            #         self.navigator.turnRight(degrees=self.degrFactor)
            #     else:
            #         self.navigator.turnLeft(degrees=self.degrFactor)
  

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
    
    def dump(self):
        self.log(f"ranges: {self.ranges}")

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
    patrol = Prep_lidar(SIMULATED_SPEED)       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(patrol)
    # Explicity destroy the node
    patrol.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()