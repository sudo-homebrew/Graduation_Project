import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import matlab.engine

class laser_sub(Node):
    def __init__(self):
        super().__init__('laser_sub')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.scan_ranges = []
        self.scanMsg = 0
        
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges # 0 ~ 359 list if 3.5 > = inf
        self.scanMsg = msg
        #print(self.scanMsge)
      
def main(args=None):
    rclpy.init(args=None)
    sub = laser_sub()
#    eng = matlab.engine.start_matlab()
    
    print("Start test codes")
#    eng.GetSLAM_Alg()
    
    for i in range(10):
      print(i, '1')
      rclpy.spin_once(sub) # get laser_scan(10 for certain)
      print(i, '2')
    print(sub.scanMsg)  
#    eng.UpdateSLAM(sub.scanMsg)
    
    
    for _ in range(10):
      rclpy.spin_once(sub) # get laser_scan(10 for certain)
    
#    eng.UpdateSLAM(sub.scanMsg)
    
#    map = eng.GetMap()
    
#    print(map)
#    eng.PyGetMapFile()

if __name__ == '__main__':
    main()