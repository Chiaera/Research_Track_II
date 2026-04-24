#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from robot_interfaces.msg import SendUserCommand
import threading

class InterfaceNode(Node): 

    def __init__(self):
        super().__init__("interface") 
        self.publisher_ = self.create_publisher(SendUserCommand, "send_command", 10)
        self.get_logger().info("Interface node has been started")
        
        thread = threading.Thread(target=self.input_loop, daemon=True)
        thread.start()

    def input_loop(self):
        while rclpy.ok():
            #show menu
            print("\nPress: ")
            print(" - 'a' to insert the goal position (x, y, theta)")
            print(" - 's' to STOP the robot (cancel the goal)")
            print(" - 'q' to shutdown")

            #read input
            cmd = input().strip().lower()
            self.get_logger().info(f"Pressed key: '{cmd}'")

            msg = SendUserCommand()

            if cmd == 'q': #shutdown
                msg.shutdown = True
                self.publisher_.publish(msg)
                self.get_logger().info("Send shutdown")
                rclpy.shutdown()
            elif cmd == 's': #stop robot --> cancel goal
                msg.cancel = True
                self.publisher_.publish(msg)
                self.get_logger().info("Send cancelation")
            elif cmd == 'a': #send goal
                try:
                    x = float(input("x = "))
                    y = float(input("y = "))
                    theta = float(input("theta = "))
                    if x < -11 or x > 11 or y < -11 or y > 11:  #check x,y are inside the world limit
                        print("Invalid position: the world size has range [-11, 11]")
                    else: #publish goal
                        msg.x = x
                        msg.y = y
                        msg.theta = theta
                        msg.cancel = False
                        msg.shutdown = False
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Send goal with position ({msg.x}, {msg.y}) and rotation {msg.theta}")
                except ValueError:
                    print("Invalid number")
            else:
                print("Key NOT recognized")
                
        

def main(args=None):
    rclpy.init(args=args)
    node = InterfaceNode() 
    rclpy.spin(node) 
    rclpy.shutdown()

if __name__ == "__main__": 
    main()