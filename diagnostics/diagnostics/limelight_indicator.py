import os
from pydoc import cli
import rclpy
from rclpy.node import Node

from autobt_msgs.srv import StringService

LIMELIGHT_IP = "10.41.45.67"
roboRIO_IP = "10.41.45.2"

def pingLimelight():
    return os.system("ping -c 1 " + LIMELIGHT_IP)

def pingRoboRIO():
    return os.system("ping -c 1 " + roboRIO_IP)
    
class clientNode(Node):
    def __init__(self): 
        print("here")
        client = self.create_client(StringService, "/limelight/set_light_state")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            self.req = StringService.Request()
    
    def send_request(self, wantLEDOn):
        self.req = StringService.Request()
        if(wantLEDOn):
            self.req.request_string = 3
            self.future = self.cli.call_async(self.req)
            rclpy.spin_once(self)
        else:
            self.req.request_string = 1
            self.future = self.cli.call_async(self.req)
            rclpy.spin_once(self)

        

def blinkLimelight(args = None):
    client = clientNode()
    for x in range(10):
        client.send_request(client, 1)
        client.send_request(client, 0)
        

def limelightOff(args = None):
    client = clientNode()
    client.send_request(client, 0)
    


def main(args= None):
    rclpy.init(args = args)

    client = clientNode()
    #client.send_request()
    print('created client')

    limelightWorks = pingLimelight()
    roboRIOWorks = pingRoboRIO()

    if(limelightWorks and roboRIOWorks) :
        blinkLimelight()
    else :
        limelightOff()
    
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()
      
    

if __name__ == '__main__':
    main()