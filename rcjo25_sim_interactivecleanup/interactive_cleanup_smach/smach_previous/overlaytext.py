#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import String

class OverlayTextNode(Node):
    def __init__(self):
        super().__init__('overlay_text_node')
        self.pub_rviz_text = self.create_publisher(OverlayText, "/clean_up_state", 1, latch=True)

    def processing(self, msg):
        text = OverlayText()
        text.text = msg
        self.pub_rviz_text.publish(text)
        self.get_logger().info(msg)

async def main(args=None):
    rclpy.init(args=args)
    overlay_text_node = OverlayTextNode()
    try:
        overlay_text_node.processing('publish overlaytext')
        # Keep the node alive to continue publishing (latching)
        await asyncio.spin(overlay_text_node)
    except KeyboardInterrupt:
        pass
    finally:
        overlay_text_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())