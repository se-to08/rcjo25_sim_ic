#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node

class Move(Node):
    def __init__(self):
        

def main():
    rclpy.init()
    node = Move()
    node.execute()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()