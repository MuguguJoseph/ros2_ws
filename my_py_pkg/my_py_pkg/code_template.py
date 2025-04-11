#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):#change name
    def __init__(self):
        super().__init__("test_py")#change name
        
   
def main(args=None):
    rclpy.init(args=args)#change name
    node=MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
