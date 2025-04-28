#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node

class Receive(Node):
    def __init__(self):
        super().__init__('search')

    def execute(self):

def main():
    rclpy.init()
    node = Search()
    node.execute()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()