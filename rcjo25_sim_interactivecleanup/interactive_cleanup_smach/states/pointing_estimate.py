#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

def get_tf(node, point_frame):
    map_frame = "map"

    print("I-----------------------I")
    print(point_frame)
    print("I-----------------------I")

    # ssd_nodeletの立ち上がりを待つ
    if point_frame == "person_first":
        point_frame = "person"
        print("person_first ----->>>>>>>> person")
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    # 座標系 (point_frame) で表現された点の位置を、グローバルな地図座標系である /map における座標に変換し、その結果を geometry_msgs.msg.Point オブジェクトとして取得する
    try:
        node.get_logger().info('Waiting for frame: person')
        tf_buffer.can_transform(map_frame, point_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=10.0))
        trans = tf_buffer.lookup_transform(map_frame, point_frame, rclpy.time.Time())

        point = Point()
        point.x = trans.transform.translation.x
        point.y = trans.transform.translation.y
        point.z = trans.transform.translation.z
        print("point person", point)

    except LookupException as ex:
        node.get_logger().error(f"TF ERROR frame: {point_frame}")
        node.get_logger().error(str(ex))
        print("エラー1")
        return Point()
    except ConnectivityException as ex:
        node.get_logger().error(f"TF ERROR frame: {point_frame}")
        node.get_logger().error(str(ex))
        print("エラー2")
        return Point()
    except ExtrapolationException as ex:
        node.get_logger().error(f"TF ERROR frame: {point_frame}")
        node.get_logger().error(str(ex))
        print("エラー3")
        return Point()

    return point
