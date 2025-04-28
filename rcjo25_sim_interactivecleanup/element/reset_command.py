import rclpy  # ROS 2 Pythonクライアントライブラリをインポート
from rclpy.node import Node  # ノードクラスをインポート

import os  # OS関連の機能を提供（プロセス管理など）
import signal  # シグナル処理を提供（プロセス終了用）
import subprocess  # サブプロセスの起動や管理に使用

import time  # スリープ処理用

# from robocup_simopl_handyman.send_message import SendMessage
from rcjo25_sim_interactivecleanup.element.task_common_send_ready import TaskCommonSendReady
from interactive_cleanup.msg import InteractiveCleanupMsg # カスタムメッセージ型のインポート

class ResetCommand(Node):
    def __init__(self):
        super().__init__('reset_command')  # ノード名を指定して初期化
        self.send_message_node = SendMessage()  # メッセージ送信ノード
        self.nav_xterm = None
        # self.ollama_xterm = None
        # 外部からのレイアウト情報を受信するサブスクライバの設定
        self.subscription_for_wait = self.create_subscription(
            HandymanMsg,
            "/interactive_cleanup/message/to_robot",
            self.wait_callback,
            10
        )

    # メッセージ受信時のコールバック関数
    def wait_callback(self, msg):
        self.message = msg.message  # メッセージの種類（"Environment", "Instruction" など）
        self.detail = msg.detail  # メッセージの詳細（レイアウト名や命令内容）

    # メインの実行処理
    def run(self):
        self.message = ""
        self.detail = ""
        # self.layout_info = ""  # レイアウト情報（例: Layout2020HM01）
        # self.llm_room_name = ""  # 実際に起動したレイアウト
        # self.imperative_statement = ""  # 命令文（例: "Go to kitchen"）


        #ollamaのxtermが開いているときのみ閉じる
        # if self.ollama_xterm != None:  # プロセスがまだ実行中か確認
        #     self.ollama_xterm.terminate()
        #     self.ollama_xterm.wait()  # プロセスの終了を待つ
        #     self.ollama_xterm = None

        # Nav2関連ノード名リスト（終了対象）
        nav2_nodes = [
            '/map_server', '/amcl', '/behavior_server', '/bt_navigator',
            '/bt_navigator_navigate_through_poses_rclcpp_node',
            '/bt_navigator_navigate_to_pose_rclcpp_node',
            '/controller_server', '/global_costmap/global_costmap',
            '/lifecycle_manager_localization', '/lifecycle_manager_navigation',
            '/local_costmap/local_costmap', '/location_tf_broadcaster',
            '/map_server', '/planner_server', '/rviz', '/rviz_navigation_dialog_action_client',
            '/waypoint_follower', '/smoother_server', '/velocity_smoother'
        ]

        hsr_sim_nodes = [
            '/camera/container', '/camera/points_xyzrgb', '/hsr_sim/joint_action_server', 
            '/hsr_sim/transform_listener_impl_5767ec37d760', '/hsr_sim/wheel_action_server',
            '/launch_ros_530', '/robot_state_publisher', '/rviz2', '/transform_listener_impl_6435286c9ed0', 
        ]

        # 各ノードを強制終了する
        for node_name in nav2_nodes:
            self.terminate_node(node_name)

        for node_name in hsr_sim_nodes:
            self.terminate_node(node_name)

        # nav2のxtermが開いているときのみ閉じる
        if self.nav_xterm != None:  # プロセスがまだ実行中か確認
            self.nav_xterm.terminate()
            self.nav_xterm.wait()  # プロセスの終了を待つ
            self.nav_xterm = None

        # レイアウト情報が来るまで待機
        # while rclpy.ok():
        #     rclpy.spin_once(self, timeout_sec=1.0)
        #     self.get_logger().info(f'waiting_sim')
        #     if self.message == "Environment":
        #         self.layout_info = self.detail  # 受信したレイアウト情報を保存
        #         self.get_logger().info(f"レイアウト情報: {self.layout_info}")
        #         break        

        time.sleep(5)
        # レイアウトに応じてNav2を起動
        if self.message == "Are_you_ready?":
            self.nav_xterm = subprocess.Popen(['xterm', '-e', 'ros2 launch sobits_navigation hsr_nav2.launch.py'])
            self.get_logger().info(f"Nav2起動")
            #llmのroomを指定する
            # self.llm_room_name = "Layout2019HM01"
            
        # elif self.layout_info == "Layout2019HM02":
        #     self.nav_xterm = subprocess.Popen(['xterm', '-e', 'ros2 launch robocup_simopl_handyman nav2_2019HM02.launch.py'])
        #     self.get_logger().info(f"Nav2起動: {self.layout_info}")
        #     #llmのroomを指定する
        #     self.llm_room_name = "Layout2019HM02"  

        # elif self.layout_info == "Layout2020HM01":
        #     self.nav_xterm = subprocess.Popen(['xterm', '-e', 'ros2 launch robocup_simopl_handyman nav2_2020HM01.launch.py'])
        #     self.get_logger().info(f"Nav2起動: {self.layout_info}")      
        #     #llmのroomを指定する
        #     self.llm_room_name = "Layout2020HM01"   

        # elif self.layout_info == "Layout2021HM01":
        #     self.nav_xterm = subprocess.Popen(['xterm', '-e', 'ros2 launch robocup_simopl_handyman nav2_2021HM01.launch.py'])
        #     self.get_logger().info(f"Nav2起動: {self.layout_info}")   
        #     #llmのroomを指定する
        #     self.llm_room_name = "Layout2021HM01"     
        else:
            #当てはまるマップがないとき
            message = "Give_up"
            detail = ""
            self.send_message_node.send_message(message, detail)
            self.get_logger().info("")
        
        time.sleep(1)  #nav2が起動するまで待機

        #ollamaを再起動
        if self.ollama_xterm == None:
            self.ollama_xterm = subprocess.Popen(['xterm', '-e', 'ros2 launch hsr_sim_common minimal.launch.py'])
        time.sleep(2)

        # 準備完了メッセージを送信
        message = "I_am_ready"
        detail = ""
        self.send_message_node.send_message(message, detail)

        # 命令文（指示）が来るまで待機
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            if self.message == "Instruction":
                self.imperative_statement = self.detail  # 指示内容を取得
                self.get_logger().info(f"命令文: {self.imperative_statement}")
                break
        # 命令文とレイアウト情報を返す
        return self.imperative_statement, self.llm_room_name, self.layout_info

    # 指定したノード名に関連するプロセスを終了させる関数
    def terminate_node(self, node_name):
        try:
            result = subprocess.run(['pgrep', '-f', node_name], stdout=subprocess.PIPE)
            pids = result.stdout.decode().split()  # PIDのリストを取得
            self.get_logger().info(f"{node_name} のPID: {pids}")
            for pid in pids:
                os.kill(int(pid), signal.SIGTERM)  # SIGTERMでプロセスを終了
                self.get_logger().info(f"プロセス {pid} を終了しました")
        except Exception as e:
            self.get_logger().error(f"{node_name} の終了中にエラーが発生: {e}")

# エントリーポイント
def main(args=None):
    rclpy.init(args=None)  # ROS 2ノードの初期化
    reset_command = ResetCommand()  # クラスのインスタンス生成
    imperative_statement, llm_room_name, layout_info = reset_command.run()  # メイン処理実行

    # ※self.get_logger() は Node クラス内でのみ使えるため、ここでは print に変更
    print(f"命令文: {imperative_statement}")
    print(f"LLM_room_name: {llm_room_name}")
    print(f"レイアウト情報: {layout_info}")


    rclpy.shutdown()  # ROS 2のシャットダウン

if __name__ == '__main__':
    main()