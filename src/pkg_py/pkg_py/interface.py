#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from robot_interfaces.msg import SendUserCommand
import threading

class InterfaceNode(Node): 

    def __init__(self):
        super().__init__("interface") 
        self.publisher_ = self.create_publisher(SendUserCommand, "user_command", 10)
        self.get_logger().info("Interface node has been started")
        
        thread = threading.Thread(target=self.input_loop, daemon=True)
        thread.start()

    def input_loop(self):
        while rclpy.ok():
            #show menu
            print("Press: ")
            print(" - 'a' to insert the goal position (x, y, theta)")
            print(" - 's' to stop the robot (cancel the goal)")
            print(" - 'q' to shutdown")

            #read input
            cmd = input()
            if cmd == 'q': #shutdown
                msg = SendUserCommand()
                msg.shutdown = True
                self.publisher_.publish(msg)
                break
            elif cmd == 's': #stop robot --> cancel goal
                msg = SendUserCommand()
                self.publisher_.publish(msg)
                msg.cancel = True
            else: #send goal
                x = float(input("x = "))
                y = float(input("y = "))
                theta = float(input("theta = "))

                msg = SendUserCommand()
                msg.x = x
                msg.y = y
                msg.theta = theta
                self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = InterfaceNode() 
    rclpy.spin(node) 
    rclpy.shutdown()

if __name__ == "__main__": 
    main()