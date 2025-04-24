import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection3DArray
import math

class PersonRegionEstimator(Node):
    def __init__(self):
        super().__init__('person_region_estimator')
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/ssd_ros/object_3d_poses',  # Person の座標がPublishされるトピック名
            self.person_pose_callback,
            10
        )
        self.initial_person_pose = Point()  # 初期の人間の座標
        self.initial_pose_set = False

    def person_pose_callback(self, msg):
        if msg.detections:  # detections 配列が空でないか確認
            first_detection = msg.detections[0]
            if first_detection.results: # results 配列が空でないか確認
                current_person_pose = first_detection.results[0].pose.pose.position
                if not self.initial_pose_set:
                    self.initial_person_pose = current_person_pose
                    self.initial_pose_set = True
                    self.get_logger().info(f"Initial person pose set: x={self.initial_person_pose.x}, y={self.initial_person_pose.y}")
                    self.search_person_region(current_person_pose, Point()) # 初期位置が原点の場合
                else:
                    self.search_person_region(current_person_pose, self.initial_person_pose)
            else:
                self.get_logger().warn("No detection results found in the first Detection3D object.")
        else:
            self.get_logger().warn("No person detected in the Detection3DArray message.")

    def search_person_region(self, person, initial_person):
        difference_posi = Point()
        target_region = ""

        difference_posi.x = initial_person.x - person.x
        difference_posi.y = initial_person.y - person.y

        if initial_person == Point():
            if 1.5 <= person.x <= 2.8 and -0.5 <= person.y <= 0.5:
                target_region = "center"
            elif 2.8 < person.x and -0.5 <= person.y <= 0.5:
                target_region = "top"
            elif person.x < 1.5 and -0.5 <= person.y <= 0.5:
                target_region = "bottom"
            elif 2.0 <= person.x <= 2.6 and 0.5 < person.y:
                target_region = "left"
            elif 2.0 <= person.x <= 2.6 and person.y < -0.5:
                target_region = "right"
            elif 2.6 < person.x and person.y < -0.5:
                target_region = "top_right"
            elif 2.6 < person.x and  0.5 < person.y:
                target_region = "top_left"
            elif person.x < 2.0 and  person.y < -0.5:
                target_region = "bottom_right"
            elif person.x < 2.0 and 0.5 < person.y:
                target_region = "bottom_left"
            else:
                target_region = ""
            self.get_logger().info(f"### Person Position (Initial Pose at Origin): {target_region} ###")
            return target_region

        else:
            if math.fabs(difference_posi.x) <= 0.3 and math.fabs(difference_posi.y) <= 0.4:
                target_region = "center"
            elif difference_posi.x < -0.3 and math.fabs(difference_posi.y) <= 0.4:
                target_region =  "top"
            elif difference_posi.x > 0.3 and math.fabs(difference_posi.y) <= 0.4:
                target_region = "bottom"
            elif math.fabs(difference_posi.x) <= 0.3 and difference_posi.y < -0.4:
                target_region = "left"
            elif math.fabs(difference_posi.x) <= 0.3 and difference_posi.y > 0.4:
                target_region = "right"
            elif difference_posi.x < -0.3 and difference_posi.y < -0.4:
                target_region = "top_left"
            elif difference_posi.x < -0.3 and difference_posi.y > 0.4:
                target_region = "top_right"
            elif difference_posi.x > 0.3 and difference_posi.y < -0.4:
                target_region = "bottom_left"
            elif difference_posi.x > 0.3 and difference_posi.y > 0.4:
                target_region = "bottom_right"
            else:
                target_region = ""
            self.get_logger().info(f"### Person Position (Relative to Initial Pose): {target_region} ###")
            return target_region

def main(args=None):
    rclpy.init(args=args)
    person_region_estimator = PersonRegionEstimator()
    rclpy.spin(person_region_estimator)
    person_region_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()