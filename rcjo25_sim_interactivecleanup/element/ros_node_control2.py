import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import signal
import psutil 

class RosNodeController(Node):
    def __init__(self):
        super().__init__('ros_node_controller')
        self.get_logger().info("Ros Node Controller initialized.")
        self.processes = []

    def processing(self, msg):
        if msg == "start":
            self.start_process("ros2 launch hsr_sim_common minimal.launch.py")
            time.sleep(3)
            self.start_process("ros2 launch sobits_navigation hsr_nav2.launch.py")
            time.sleep(3)
            self.get_logger().info("ROS nodes started.")

        elif msg == "kill":
            self.get_logger().info("Killing all started processes...")
            for p in self.processes:
                if p.poll() is None:
                    kill_process_tree(p.pid)
                    self.get_logger().info(f"Killed process tree {p.pid}")
            self.processes.clear()
            time.sleep(5)
            self.get_logger().info('************* ROS_NODE RESET FINISH *************')


        else:
            self.get_logger().error("ros_node_ctrl: unknown msg...")

    def start_process(self, command):
        self.get_logger().info(f"Starting: {command}")
        p = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        self.processes.append(p)

    # プロセスツリーを終了させる関数
    def kill_process_tree(pid):
        try:
            parent = psutil.Process(pid)
            children = parent.children(recursive=True)
            for child in children:
                child.kill()
            parent.kill()
        except Exception as e:
            print(f"Error killing process tree for PID {pid}: {e}")


def main(args=None):
    rclpy.init(args=args)
    ros_node_ctrl = RosNodeController()

    try:
        message = "start"  # ←ここを "start" または "kill" に手動で変更
        ros_node_ctrl.processing(message)
        rclpy.spin(ros_node_ctrl)  # 追加：CTRL+Cの監視に必要
    except KeyboardInterrupt:
        pass
    finally:
        ros_node_ctrl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
