import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Patrol(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('patrol')

           
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    patrol = Patrol()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(patrol)
    # Explicity destroy the node
    patrol.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()