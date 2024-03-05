import rclpy
from rclpy.node import Node
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math


class Controller(Node):
   def __init__(self):
       super().__init__('controller')
       self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 1000)
       self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.laser_callback, 1000)
       self.kp = 14
       self.kd = 0
       self.ki = 0.09
       self.target = 0.3
       self.lookahead = 
       self.current_time = time.time()
       self.dt = 0

       self.integral = 0
       self.prev_error = 0
       self.error = 0
       self.derivate = 0
  
   def laser_callback(self, msg):
       # Print LiDAR output
       #print(msg.ranges)


       # Example of how to send drive command
       mymsg = AckermannDriveStamped()
       # Steering angle should be between -40 and 40 most of the time
       # mymsg.drive.steering_angle=10.0
       # Speed should be between 0 and 1 most of the time
       # Send the drive command
       self.publisher_.publish(mymsg)
       self.get_logger().info('Publishing: "%s"' % mymsg.drive)


       b = msg.ranges[160]
       a = msg.ranges[280]


       alpha = math.atan((a*math.cos(math.pi/6) - b)/(a*math.sin(math.pi/6)))
       D = b*math.cos(alpha)
       self.prev_error = self.target - D
       self.error = D + self.lookahead * math.sin(alpha)
       self.dt = time.time() - self.current_time
       self.current_time = time.time()
       self.integral += self.error * dt
       self.derivative = (self.error - self.prev_error)/self.dt
       steeringAngle = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
      
       mymsg.drive.steering_angle = steeringAngle


       steeringAngleDeg = np.rad2deg(steeringAngle)


       if(steeringAngleDeg < 0):
           mymsg.drive.speed = 1.5
          
       if(steeringAngleDeg >= 0 and steeringAngleDeg <= 10):
           mymsg.drive.speed = 1.5
      
       elif(steeringAngleDeg > 10 and steeringAngleDeg <= 20):
           mymsg.drive.speed = 1


       else:
           mymsg.drive.speed = 0.5






def main(args=None):
   rclpy.init(args=args)
   print("WallFollow Initialized")
   controller = Controller()
   rclpy.spin(controller)
   controller.destroy_node()
   rclpy.shutdown()




if __name__ == '__main__':
   main()
"""import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers

        # TODO: set PID gains
        # self.kp = 
        # self.kd = 
        # self.ki = 

        # TODO: store history
        # self.integral = 
        # self.prev_error = 
        # self.error = 

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        return 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        return 0.0

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = 0.0 # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()"""
