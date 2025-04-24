import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose
import time
import math
from tf_transformations import quaternion_from_euler
from smach import State
from smach_ros import IntrospectionServer
import asyncio

# グローバル変数の代替
Next_State = None
Receive_Challenge_Times = 1
Object_Area = ""
Pointing_Position1 = Point()
Pointing_Position2 = Point()
Initial_Position = Point()
Head_Direction1 = None
Head_Direction2 = None
Pointing_Direction1 = None
Pointing_Direction2 = None
Target_Obj_Frame_Name = []
Grasp_Challenge_Times = 3
Target_Put_Frame_Name = ""

class Ros2Start(Node):
    def __init__(self):
        super().__init__('start_state')
        self.publisher_processing = self.create_publisher(String, 'processing_command', 10)
        self.publisher_overlaytext = self.create_publisher(String, 'overlay_text', 10)

    async def execute(self):
        global Next_State
        self.get_logger().info('STATE : Challenge Start')
        processing_kill_msg = String()
        processing_kill_msg.data = 'kill'
        self.publisher_processing.publish(processing_kill_msg)
        self.get_logger().info('Published: "kill" to processing_command')
        processing_start_msg = String()
        processing_start_msg.data = 'start'
        self.publisher_processing.publish(processing_start_msg)
        self.get_logger().info('Published: "start" to processing_command')
        self.get_logger().info('Task flags reset (implementation needed)')
        self.get_logger().info('Person follow initialized (implementation needed)')
        overlay_text_msg = String()
        overlay_text_msg.data = 'STATE : Challenge Start'
        self.publisher_overlaytext.publish(overlay_text_msg)
        self.get_logger().info('Published: "STATE : Challenge Start" to overlay_text')
        time.sleep(10)
        Next_State = 'receive'
        return 'start_OK'

class Ros2Check(Node):
    def __init__(self):
        super().__init__('check_state')
        self.publisher_overlaytext = self.create_publisher(String, 'overlay_text', 10)

    async def execute(self):
        global Next_State
        # ここに遷移ロジックを実装します。
        # 例として、常に 'receive' に遷移するようにしています。
        self.get_logger().info('STATE : Check')
        overlay_text_msg = String()
        overlay_text_msg.data = 'STATE : Check'
        self.publisher_overlaytext.publish(overlay_text_msg)
        return Next_State # グローバル変数 Next_State に基づいて遷移

class Ros2Receive(Node):
    def __init__(self):
        super().__init__('receive_state')
        self.publisher_processing = self.create_publisher(String, 'processing_command', 10)
        self.publisher_overlaytext = self.create_publisher(String, 'overlay_text', 10)
        self.publisher_person_follow_launch = self.create_publisher(String, 'person_follow_launch', 10)
        self.publisher_base_ctrl = self.create_publisher(String, 'base_ctrl_command', 10)
        self.publisher_task_msg = self.create_publisher(String, 'task_message', 10)
        self.pointing_estimate_client = self.create_client(self.get_service_type('get_tf'), 'get_tf_service')
        self.wait_ready_client = self.create_client(self.get_service_type('wait_ready'), 'wait_ready_service')
        self.wait_pick_it_up_client = self.create_client(self.get_service_type('wait_command'), 'wait_pick_it_up_service')
        self.wait_clean_up_client = self.create_client(self.get_service_type('wait_command'), 'wait_clean_up_service')
        self.repointing_client = self.create_client(self.get_service_type('repointing'), 'repointing_service')

    def get_service_type(self, service_name):
        if service_name == 'get_tf':
            from tf2_ros.srv import FrameGraph
            return FrameGraph
        elif service_name == 'wait_ready':
            from std_srvs.srv import Trigger
            return Trigger
        elif service_name == 'wait_command':
            from std_srvs.srv import Trigger
            return Trigger
        elif service_name == 'repointing':
            from std_srvs.srv import Trigger
            return Trigger
        else:
            return None

    async def call_service(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{client.srv_name} service not available, waiting again...')
        future = client.call_async(request)
        await future
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Exception while calling service {client.srv_name}: {future.exception()}')
            return None

    async def execute(self):
        global Next_State, Receive_Challenge_Times, Object_Area, Pointing_Position1, Pointing_Position2, Initial_Position
        Object_Area = ""
        Pointing_Position1 = Point()
        Pointing_Position2 = Point()
        Initial_Position = Point()
        time.sleep(1)
        self.get_logger().info("=========================")
        overlay_text_msg = String()
        overlay_text_msg.data = 'STATE : Receive Instruction'
        self.publisher_overlaytext.publish(overlay_text_msg)
        self.get_logger().info('Published: "STATE : Receive Instruction" to overlay_text')
        person_follow_launch_msg = String()
        self.publisher_person_follow_launch.publish(person_follow_launch_msg)
        self.get_logger().info('Published command to launch person follow')
        time.sleep(1)
        self.get_logger().info("=========================")
        overlay_text_msg.data = 'STATE : Receive Instruction\nWait for Are_you_ready message.'
        self.publisher_overlaytext.publish(overlay_text_msg)
        self.get_logger().info('Published: "STATE : Receive Instruction\nWait for Are_you_ready message." to overlay_text')
        wait_ready_request = self.get_service_type('wait_ready').Request()
        wait_ready_response = await self.call_service(self.wait_ready_client, wait_ready_request)
        if wait_ready_response is not None and wait_ready_response.success:
            overlay_text_msg.data = 'STATE : Receive Instruction\nSend I_am_ready message.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            self.get_logger().info('Published: "STATE : Receive Instruction\nSend I_am_ready message." to overlay_text')
        else:
            self.get_logger().warn('Failed to wait for ready command.')
        base_ctrl_msg = String()
        base_ctrl_msg.data = "X:50"
        self.publisher_base_ctrl.publish(base_ctrl_msg)
        self.get_logger().info('Published: "X:50" to base_ctrl_command')
        time.sleep(2.0)
        self.get_logger().info("1回目")
        get_tf_request = self.get_service_type('get_tf').Request()
        get_tf_request.frame_id = "person"
        initial_position_response = await self.call_service(self.pointing_estimate_client, get_tf_request)
        if initial_position_response is not None:
            Initial_Position.x = initial_position_response.transform.translation.x
            Initial_Position.y = initial_position_response.transform.translation.y
            Initial_Position.z = initial_position_response.transform.translation.z
        else:
            self.get_logger().warn('Could not get initial person position, using default.')
            Initial_Position.x = 2.25
            Initial_Position.y = 0.0
        self.get_logger().info(f"Person_Initial_position: {Initial_Position}")
        pointing_again_flag = False
        for receive_challenge in range(Receive_Challenge_Times):
            overlay_text_msg.data = f'STATE : Receive Instruction({receive_challenge+1}/{Receive_Challenge_Times})\nStart Pointing.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            self.get_logger().info(f'Published: "STATE : Receive Instruction({receive_challenge+1}/{Receive_Challenge_Times})\nStart Pointing." to overlay_text')
            robot_motion_msg = String()
            robot_motion_msg.data = 'INITIAL_POSE'
            # self.publisher_robot_motion.publish(robot_motion_msg) # 未定義
            self.get_logger().info("===================-before")
            self.get_logger().info("===================-after")
            overlay_text_msg.data = 'STATE : Receive Instruction\nWait for Pick_it_up! message.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            self.get_logger().info('Published: "STATE : Receive Instruction\nWait for Pick_it_up! message." to overlay_text')
            wait_pick_it_up_request = self.get_service_type('wait_command').Request()
            wait_pick_it_up_response = await self.call_service(self.wait_pick_it_up_client, wait_pick_it_up_request)
            if wait_pick_it_up_response is not None and wait_pick_it_up_response.success:
                self.get_logger().info("2回目")
                get_tf_request.frame_id = "person"
                pointing_position1_response = await self.call_service(self.pointing_estimate_client, get_tf_request)
                if pointing_position1_response is not None:
                    Pointing_Position1.x = pointing_position1_response.transform.translation.x
                    Pointing_Position1.y = pointing_position1_response.transform.translation.y
                    Pointing_Position1.z = pointing_position1_response.transform.translation.z
                time.sleep(2.0)
                time.sleep(2.5)
                if Pointing_Position1.x == 0.0 and Pointing_Position1.y == 0.0 and Pointing_Position1.z == 0.0:
                    time.sleep(1.5)
                    self.get_logger().info("3回目")
                    pointing_position1_response = await self.call_service(self.pointing_estimate_client, get_tf_request)
                    if pointing_position1_response is not None:
                        Pointing_Position1.x = pointing_position1_response.transform.translation.x
                        Pointing_Position1.y = pointing_position1_response.transform.translation.y
                        Pointing_Position1.z = pointing_position1_response.transform.translation.z
                self.get_logger().info(f"POINTING_POSITION1: {Pointing_Position1}")
                time.sleep(1)
                self.get_logger().info("指差し開始")
                self.get_logger().info("1回目の認識")
                overlay_text_msg.data = 'STATE : Receive Instruction\nWait for Clean_up! message.'
                self.publisher_overlaytext.publish(overlay_text_msg)
                self.get_logger().info('Published: "STATE : Receive Instruction\nWait for Clean_up! message." to overlay_text')
                wait_clean_up_request = self.get_service_type('wait_command').Request()
                wait_clean_up_response = await self.call_service(self.wait_clean_up_client, wait_clean_up_request)
                if wait_clean_up_response is not None and wait_clean_up_response.success:
                    self.get_logger().info("4回目")
                    pointing_position2_response = await self.call_service(self.pointing_estimate_client, get_tf_request)
                    if pointing_position2_response is not None:
                        Pointing_Position2.x = pointing_position2_response.transform.translation.x
                        Pointing_Position2.y = pointing_position2_response.transform.translation.y
                        Pointing_Position2.z = pointing_position2_response.transform.translation.z
                    self.get_logger().info(f"POINTING_POSITION2: {Pointing_Position2}")
                    time.sleep(2.0)
                    time.sleep(2.5)
                    if Pointing_Position2.x == 0.0 and Pointing_Position2.y == 0.0 and Pointing_Position2.z == 0.0:
                        time.sleep(1.5)
                        self.get_logger().info("5回目")
                        pointing_position2_response = await self.call_service(self.pointing_estimate_client, get_tf_request)
                        if pointing_position2_response is not None:
                            Pointing_Position2.x = pointing_position2_response.transform.translation.x
                            Pointing_Position2.y = pointing_position2_response.transform.translation.y
                            Pointing_Position2.z = pointing_position2_response.transform.translation.z
                    self.get_logger().info(f"POINTING_POSITION2: {Pointing_Position2}")
                    time.sleep(1)
                    self.get_logger().info("2回目の認識")
                    Next_State = 'search'
                    return 'finish'
                overlay_text_msg.data = 'STATE : Receive Instruction\nStopped following.'
                self.publisher_overlaytext.publish(overlay_text_msg)
                if Pointing_Position1.x == 0.0 and Pointing_Position1.y == 0.0 and Pointing_Position1.z == 0.0:
                    if pointing_again_flag == True:
                        # person_follow_ctrl_request = SetBool.Request() # 未定義
                        # await self.call_service(self.person_follow_service_client, person_follow_ctrl_request) # 未定義
                        person_follow_kill_msg = String()
                        # self.publisher_person_follow_kill.publish(person_follow_kill_msg) # 未定義
                        time.sleep(3)
                        Next_State = "give_up"
                        return 'finish'
                    self.get_logger().info("再度指さし")
                    repointing_request = self.get_service_type('repointing').Request()
                    await self.call_service(self.repointing_client, repointing_request)
                    # person_follow_ctrl_request = SetBool.Request() # 未定義
                    # await self.call_service(self.person_follow_service_client, person_follow_ctrl_request) # 未定義
                    # person_follow_initialize() # 未定義
                    initial_position = Pose()
                    initial_position.position.x = 0.0
                    initial_position.position.y = 0.1
                    initial_position.orientation.w = 1.0
                    # move_res = move.processing_pose(initial_position) # 未定義
                    # if move_res == 'FAILURE': # 未定義
                    # 	self.get_logger().info("移動失敗...") # 未定義
                    # 	Next_State = "give_up" # 未定義
                    # 	return 'finish' # 未定義
                    # else:
                    task_msg = String()
                    task_msg.data = "Point_it_again"
                    self.publisher_task_msg.publish(task_msg)
                    pointing_again_flag = True
                else:
                    self.get_logger().info("get person_point !!!")
                    overlay_text_msg.data = 'STATE : Receive Instruction\n"Get person_point !!!"'
                    self.publisher_overlaytext.publish(overlay_text_msg)
                    # Object_Area = pointing_estimate.search_person_region(Pointing_Position1, Initial_Position) # 未定義
                    break
        Next_State = 'search'
        return 'finish'

class Ros2Search(Node):
    def __init__(self):
        super().__init__('search_state')
        self.publisher_processing = self.create_publisher(String, 'processing_command', 10)
        self.publisher_overlaytext = self.create_publisher(String, 'overlay_text', 10)
        self.publisher_person_follow_kill = self.create_publisher(String, 'person_follow_kill', 10)
        self.publisher_base_ctrl = self.create_publisher(String, 'base_ctrl_command', 10)
        self.publisher_robot_motion = self.create_publisher(String, 'robot_motion_command', 10)
        self.object_recog_client = self.create_client(self.get_service_type('object_recognition'), 'object_recognition_service')
        self.start_tf_broadcaster_client = self.create_client(self.get_service_type('start_tf_broadcaster'), 'start_tf_broadcaster_service')
        self.search_close_frames_client = self.create_client(self.get_service_type('search_close_frames'), 'search_close_frames_service')
        self.escape_cost_position_client = self.create_client(self.get_service_type('escape_cost_position'), 'escape_cost_position_service')
        self.rotate_to_object_area_client = self.create_client(self.get_service_type('rotate_to_object_area'), 'rotate_to_object_area_service')
        self.move_processing_client = self.create_client(self.get_service_type('move_pose'), 'move_processing_service')

    def get_service_type(self, service_name):
        if service_name == 'object_recognition':
            from std_srvs.srv import Trigger
            return Trigger
        elif service_name == 'start_tf_broadcaster':
            from std_srvs.srv import Trigger
            return Trigger
        elif service_name == 'search_close_frames':
            from your_package.srv import SearchCloseFrames # 実際の型に合わせてください
            return SearchCloseFrames
        elif service_name == 'escape_cost_position':
            from your_package.srv import EscapeCostPosition # 実際の型に合わせてください
            return EscapeCostPosition
        elif service_name == 'rotate_to_object_area':
            from your_package.srv import RotateToObjectArea # 実際の型に合わせてください
            return RotateToObjectArea
        elif service_name == 'move_pose':
            from your_package.srv import MovePose # 実際の型に合わせてください
            return MovePose
        else:
            return None

    async def call_service(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{client.srv_name} service not available, waiting again...')
        future = client.call_async(request)
        await future
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Exception while calling service {client.srv_name}: {future.exception()}')
            return None

    async def execute(self):
        global Next_State, Object_Area, Target_Obj_Frame_Name, Pointing_Position1, Pointing_Position2, Pointing_Direction1, Pointing_Direction2, Initial_Position, Head_Direction1, Head_Direction2

        self.get_logger().info("in Search")
        time.sleep(1.0)

        person_follow_kill_msg = String()
        self.publisher_person_follow_kill.publish(person_follow_kill_msg)
        self.get_logger().info("Published command to kill person follow")

        self.get_logger().info("ssd_kill") # 該当機能がROS 2ノード内にある場合は呼び出し方を変更

        search_pose = Pose()
        search_pose.orientation.w = 1.0

        person_2_robot_dis = 1.1

        self.get_logger().info(f"Pointing_Direction1: {Pointing_Direction1}")
        if Object_Area == "center":
            if Initial_Position.x == 0.0 and Initial_Position.y == 0.0 and Initial_Position.z == 0.0:
                Initial_Position = Pointing_Position1

            if Pointing_Direction1 is not None:
                move_direction_rad = Pointing_Direction1 + math.pi / 2
                Move_Direction = math.degrees(move_direction_rad % (2 * math.pi))
                self.get_logger().info(f"DIRECTION: {Pointing_Direction1}")

                search_pose.position.x = person_2_robot_dis * math.cos(move_direction_rad) + Initial_Position.x
                search_pose.position.y = person_2_robot_dis * math.sin(move_direction_rad) + Initial_Position.y
            else:
                self.get_logger().warn("Pointing_Direction1 is None, cannot calculate search pose for 'center'.")
                Next_State = "give_up"
                return 'finish'

        else:
            Move_Direction = Pointing_Direction1
            search_pose.position = Pointing_Position1

        self.get_logger().info(f"Move_Direction: {Move_Direction}")
        q = quaternion_from_euler(0, 0, math.radians(Move_Direction))
        search_pose.orientation.x = q[0]
        search_pose.orientation.y = q[1]
        search_pose.orientation.z = q[2]
        search_pose.orientation.w = q[3]

        if search_pose.position.x == 0.0 and search_pose.position.y == 0.0 and search_pose.position.z == 0.0 and search_pose.orientation.w == 1.0 and search_pose.orientation.x == 0.0 and search_pose.orientation.y == 0.0 and search_pose.orientation.z == 0.0:
            Next_State = "give_up"
            return 'finish'

        time.sleep(9.0)

        self.get_logger().info(f"Moving to search pose: {search_pose}")
        move_request = self.get_service_type('move_pose').Request()
        move_request.target_pose = search_pose
        move_response = await self.call_service(self.move_processing_client, move_request)

        if move_response is None or not move_response.success:
            self.get_logger().warn("移動失敗...")
            self.get_logger().info("再移動...")

            escape_request = self.get_service_type('escape_cost_position').Request()
            escape_request.object_area = Object_Area
            escape_request.current_pose = search_pose
            re_search_response = await self.call_service(self.escape_cost_position_client, escape_request)

            if re_search_response is not None and re_search_response.success:
                move_request.target_pose = re_search_response.escaped_pose
                move_response = await self.call_service(self.move_processing_client, move_request)
            else:
                self.get_logger().warn("Failed to get escape cost position.")
                move_response = None

        if move_response is None or not move_response.success:
            self.get_logger().error("移動失敗2...")
            Next_State = "give_up"
            return 'finish'

        if Object_Area == "center":
            base_ctrl_msg = String()
            base_ctrl_msg.data = f"T:{Pointing_Direction1}"
            self.publisher_base_ctrl.publish(base_ctrl_msg)
            self.get_logger().info(f"Published: T:{Pointing_Direction1} to base_ctrl_command")
        else:
            rotate_request = self.get_service_type('rotate_to_object_area').Request()
            rotate_request.object_area = Object_Area
            rotate_response = await self.call_service(self.rotate_to_object_area_client, rotate_request)
            if rotate_response is not None and rotate_response.success:
                base_ctrl_msg = String()
                base_ctrl_msg.data = f"T:{Pointing_Direction1}"
                self.publisher_base_ctrl.publish(base_ctrl_msg)
                self.get_logger().info(f"Published: T:{Pointing_Direction1} to base_ctrl_command")
            else:
                self.get_logger().warn("Failed to rotate to object area.")

        self.get_logger().info("オブジェクトのデータ収集開始")
        start_tf_request = self.get_service_type('start_tf_broadcaster').Request()
        await self.call_service(self.start_tf_broadcaster_client, start_tf_request)

        overlay_text_msg = String()
        overlay_text_msg.data = 'STATE : Search Object\nStart Search Object Position.'
        self.publisher_overlaytext.publish(overlay_text_msg)
        self.get_logger().info('Published: "STATE : Search Object\nStart Search Object Position." to overlay_text')

        object_recog_request = self.get_service_type('object_recognition').Request()
        object_recog_response = await self.call_service(self.object_recog_client, object_recog_request)

        grasp_object_list = []
        if object_recog_response is not None and hasattr(object_recog_response, 'object_list'):
            grasp_object_list = object_recog_response.object_list

        search_close_request = self.get_service_type('search_close_frames').Request()
        search_close_request.base_frame = "base_footprint"
        search_close_request.object_frames = grasp_object_list
        search_close_response = await self.call_service(self.search_close_frames_client, search_close_request)

        if search_close_response is not None and search_close_response.closest_frames:
            Target_Obj_Frame_Name = search_close_response.closest_frames
        else:
            Target_Obj_Frame_Name = []

        if not Target_Obj_Frame_Name or (Target_Obj_Frame_Name and Target_Obj_Frame_Name[0] == 'F'):
            if Object_Area == "center":
                base_ctrl_msg = String()
                base_ctrl_msg.data = "T:40"
                self.publisher_base_ctrl.publish(base_ctrl_msg)
                self.get_logger().info('Published: "T:40" to base_ctrl_command')

            base_ctrl_msg = String()
            base_ctrl_msg.data = "X:35"
            self.publisher_base_ctrl.publish(base_ctrl_msg)
            self.get_logger().info('Published: "X:35" to base_ctrl_command')

            if Object_Area == "center":
                base_ctrl_msg = String()
                base_ctrl_msg.data = "T:-40"
                self.publisher_base_ctrl.publish(base_ctrl_msg)
                self.get_logger().info('Published: "T:-40" to base_ctrl_command')

            object_recog_response = await self.call_service(self.object_recog_client, object_recog_request)
            grasp_object_list = []
            if object_recog_response is not None and hasattr(object_recog_response, 'object_list'):
                grasp_object_list = object_recog_response.object_list
            search_close_request.object_frames = grasp_object_list
            search_close_response = await self.call_service(self.search_close_frames_client, search_close_request)
            if search_close_response is not None and search_close_response.closest_frames:
                Target_Obj_Frame_Name = search_close_response.closest_frames
            else:
                Target_Obj_Frame_Name = []

            if not Target_Obj_Frame_Name or (Target_Obj_Frame_Name and Target_Obj_Frame_Name[0] == 'F'):
                Next_State = 'give_up'
                return 'finish'

        Next_State = 'grasp'
        return 'finish'

class Ros2Grasp(Node):
    def __init__(self):
        super().__init__('grasp_state')
        self.publisher_processing = self.create_publisher(String, 'processing_command', 10)
        self.publisher_overlaytext = self.create_publisher(String, 'overlay_text', 10)
        self.publisher_robot_motion = self.create_publisher(String, 'robot_motion_command', 10)
        self.publisher_base_ctrl = self.create_publisher(String, 'base_ctrl_command', 10)
        self.publisher_task_msg = self.create_publisher(String, 'task_message', 10)
        self.standing_position_estimation_client = self.create_client(self.get_service_type('standing_position'), 'standing_position_estimation_service')
        self.move_processing_client = self.create_client(self.get_service_type('move_pose'), 'move_processing_service')
        self.get_tf_client = self.create_client(self.get_service_type('get_tf'), 'get_tf_service')
        self.gripper_open_client = self.create_client(self.get_service_type('gripper_command'), 'gripper_open_service')
        self.gripper_move_to_target_client = self.create_client(self.get_service_type('gripper_move'), 'gripper_move_to_target_service')
        self.gripper_close_client = self.create_client(self.get_service_type('gripper_command'), 'gripper_close_service')
        self.grasp_judge_client = self.create_client(self.get_service_type('grasp_judge'), 'grasp_judge_service')
        self.adjustment_low_client = self.create_client(self.get_service_type('object_adjustment'), 'adjustment_low_service')
        self.adjustment_high_client = self.create_client(self.get_service_type('object_adjustment'), 'adjustment_high_service')
        self.wait_correct_answer_client = self.create_client(self.get_service_type('wait_command_bool'), 'wait_correct_answer_service')
        self.adjustment_hand_client = self.create_client(self.get_service_type('gripper_command'), 'adjustment_hand_service')

    def get_service_type(self, service_name):
        if service_name == 'standing_position':
            from your_package.srv import GetPose
            return GetPose
        elif service_name == 'move_pose':
            from your_package.srv import MovePose
            return MovePose
        elif service_name == 'get_tf':
            from tf2_ros.srv import FrameGraph
            return FrameGraph
        elif service_name == 'gripper_command':
            from std_srvs.srv import Trigger
            return Trigger
        elif service_name == 'gripper_move':
            from your_package.srv import MoveToPose
            return MoveToPose
        elif service_name == 'grasp_judge':
            from std_srvs.srv import Trigger
            return Trigger
        elif service_name == 'object_adjustment':
            from std_srvs.srv import Trigger
            return Trigger
        elif service_name == 'wait_command_bool':
            from your_package.srv import WaitForBool
            return WaitForBool
        else:
            return None

    async def call_service(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{client.srv_name} service not available, waiting again...')
        future = client.call_async(request)
        await future
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Exception while calling service {client.srv_name}: {future.exception()}')
            return None

    async def execute(self):
        global Next_State, Target_Obj_Frame_Name, Object_Area, Initial_Position, Grasp_Challenge_Times
        overlay_text_msg = String()
        overlay_text_msg.data = 'STATE : Grasp Object'
        self.publisher_overlaytext.publish(overlay_text_msg)
        self.get_logger().info('Published: "STATE : Grasp Object" to overlay_text')

        move_challange_time = 0
        correct_judge_count = 0
        res_grasp_judge = False

        for i in range(len(Target_Obj_Frame_Name)):
            move_challange_time += 1
            if move_challange_time == 3:
                Next_State = "give_up"
                return 'finish'

            overlay_text_msg.data = f'STATE : Grasp Object\nGrasping target object[{Target_Obj_Frame_Name[i]}]'
            self.publisher_overlaytext.publish(overlay_text_msg)
            self.get_logger().info(f'Published: "STATE : Grasp Object\nGrasping target object[{Target_Obj_Frame_Name[i]}]" to overlay_text')

            robot_motion_msg = String()
            robot_motion_msg.data = 'INITIAL_POSE'
            self.publisher_robot_motion.publish(robot_motion_msg)
            self.get_logger().info('Published: "INITIAL_POSE" to robot_motion_command')

            standing_position_request = self.get_service_type('standing_position').Request()
            standing_position_request.target_frame = Target_Obj_Frame_Name[i]
            standing_position_request.distance = 0.9
            standing_position_response = await self.call_service(self.standing_position_estimation_client, standing_position_request)

            if standing_position_response is not None and standing_position_response.result.position.x != 0.0 or standing_position_response.result.position.y != 0.0 or standing_position_response.result.position.z != 0.0:
                move_request = self.get_service_type('move_pose').Request()
                move_request.target_pose = standing_position_response.result
                move_response = await self.call_service(self.move_processing_client, move_request)
                if move_response is None or not move_response.success:
                    Next_State = "give_up"
                    return 'finish'
            else:
                self.get_logger().warn(f"Failed to estimate standing position for {Target_Obj_Frame_Name[i]}.")
                Next_State = "give_up"
                return 'finish'

            object_shift_z = -0.01
            for grasp_challenge in range(Grasp_Challenge_Times):
                self.get_logger().info(f"object_shift_z: {object_shift_z}")
                if grasp_challenge == 2:
                    object_shift_z = -0.04
                    self.get_logger().info(f"object_shift_z: {object_shift_z}")
                self.get_logger().info(f"### grasp_challenge({grasp_challenge+1}/{Grasp_Challenge_Times}) ###")
                robot_motion_msg.data = 'INITIAL_POSE'
                self.publisher_robot_motion.publish(robot_motion_msg)

                get_tf_request = self.get_service_type('get_tf').Request()
                get_tf_request.frame_id = Target_Obj_Frame_Name[i]
                target_obj_height_response = await self.call_service(self.get_tf_client, get_tf_request)
                target_obj_height = Point()
                if target_obj_height_response is not None:
                    target_obj_height.z = target_obj_height_response.transform.translation.z
                self.get_logger().info(f"物の高さ: {target_obj_height.z}")

                if target_obj_height.z < 0.34:
                    await self.call_service(self.adjustment_low_client, self.get_service_type('object_adjustment').Request())
                else:
                    await self.call_service(self.adjustment_high_client, self.get_service_type('object_adjustment').Request())

                standing_position_request.target_frame = Target_Obj_Frame_Name[i]
                standing_position_request.distance = 0.9
                standing_position_response = await self.call_service(self.standing_position_estimation_client, standing_position_request)

                if standing_position_response is not None and standing_position_response.result.position.x != 0.0 or standing_position_response.result.position.y != 0.0 or standing_position_response.result.position.z != 0.0:
                    move_request.target_pose = standing_position_response.result
                    move_response = await self.call_service(self.move_processing_client, move_request)
                    if move_response is None or not move_response.success:
                        Next_State = "give_up"
                        return 'finish'
                else:
                    Next_State = "give_up"
                    return 'finish'

                #物を把持する処理
                overlay_text_msg.data = 'STATE : Grasp Object\nGrasping target object.'
                self.publisher_overlaytext.publish(overlay_text_msg)
                open_gripper_response = await self.call_service(self.gripper_open_client, self.get_service_type('gripper_command').Request())

                move_to_target_request = self.get_service_type('gripper_move').Request()
                move_to_target_request.target_frame = Target_Obj_Frame_Name[i]
                move_to_target_request.relative_position.x = -0.3
                move_to_target_request.relative_position.z = object_shift_z
                move_to_target_response = await self.call_service(self.gripper_move_to_target_client, move_to_target_request)
                time.sleep(3.0)

                if move_to_target_response is None or not move_to_target_response.success:
                    Next_State = "give_up"
                    return 'finish'

                base_ctrl_msg = String()
                base_ctrl_msg.data = "X:33"
                self.publisher_base_ctrl.publish(base_ctrl_msg)
                self.get_logger().info('Published: "X:33" to base_ctrl_command')
                time.sleep(1.0)

                close_gripper_response = await self.call_service(self.gripper_close_client, self.get_service_type('gripper_command').Request())
                time.sleep(1.0)

                grasp_judge_response = await self.call_service(self.grasp_judge_client, self.get_service_type('grasp_judge').Request())
                res_grasp_judge = grasp_judge_response is not None and grasp_judge_response.success

                grasp_again_flag = False
                if not res_grasp_judge:
                    grasp_again_flag = True
                    await self.call_service(self.gripper_open_client, self.get_service_type('gripper_command').Request())
                    base_ctrl_msg.data = "X:4"
                    self.publisher_base_ctrl.publish(base_ctrl_msg)
                    time.sleep(1.0)
                    await self.call_service(self.gripper_close_client, self.get_service_type('gripper_command').Request())
                    time.sleep(1.0)
                    grasp_judge_response = await self.call_service(self.grasp_judge_client, self.get_service_type('grasp_judge').Request())
                    res_grasp_judge = grasp_judge_response is not None and grasp_judge_response.success

                if not res_grasp_judge:
                    base_ctrl_msg.data = "X:-37"
                    self.publisher_base_ctrl.publish(base_ctrl_msg)
                else:
                    if len(Target_Obj_Frame_Name) > 1 and correct_judge_count < 1:
                        overlay_text_msg.data = 'STATE : Grasp Object\nIs it correct?'
                        self.publisher_overlaytext.publish(overlay_text_msg)
                        task_msg = String()
                        task_msg.data = "Is_this_correct?"
                        self.publisher_task_msg.publish(task_msg)
                        time.sleep(1.0)
                        wait_correct_answer_request = self.get_service_type('wait_command_bool').Request()
                        correct_object_response = await self.call_service(self.wait_correct_answer_client, wait_correct_answer_request)
                        correct_object = correct_object_response is not None and correct_object_response.result

                        if correct_object:
                            task_msg.data = "Object_grasped"
                            self.publisher_task_msg.publish(task_msg)
                            if grasp_again_flag:
                                base_ctrl_msg.data = "X:-37"
                                self.publisher_base_ctrl.publish(base_ctrl_msg)
                            else:
                                base_ctrl_msg.data = "X:-33"
                                self.publisher_base_ctrl.publish(base_ctrl_msg)
                            overlay_text_msg.data = 'STATE : Grasp Object\nGrasped correct object !!!!!'
                            self.publisher_overlaytext.publish(overlay_text_msg)
                            robot_motion_msg.data = 'INITIAL_POSE'
                            self.publisher_robot_motion.publish(robot_motion_msg)
                            Next_State = 'send'
                            return 'finish'
                        else:
                            overlay_text_msg.data = 'STATE : Grasp Object\nGrasped wrong object ...'
                            self.publisher_overlaytext.publish(overlay_text_msg)
                            await self.call_service(self.gripper_open_client, self.get_service_type('gripper_command').Request())
                            time.sleep(1.0)
                            if grasp_again_flag:
                                base_ctrl_msg.data = "X:-37"
                                self.publisher_base_ctrl.publish(base_ctrl_msg)
                            else:
                                base_ctrl_msg.data = "X:-33"
                                self.publisher_base_ctrl.publish(base_ctrl_msg)
                            time.sleep(1.0)
                            await self.call_service(self.gripper_close_client, self.get_service_type('gripper_command').Request())
                            break
                    else:
                        self.get_logger().info("オブジェクト１つのとき！")
                        task_msg = String()
                        task_msg.data = "Object_grasped"
                        self.publisher_task_msg.publish(task_msg)
                        time.sleep(1.0)
                        # adjustment_hand() # 未定義
                        adjustment_hand_response = await self.call_service(self.adjustment_hand_client, self.get_service_type('gripper_command').Request())
                        if grasp_again_flag:
                            base_ctrl_msg.data = "X:-37"
                            self.publisher_base_ctrl.publish(base_ctrl_msg)
                        else:
                            base_ctrl_msg.data = "X:-33"
                            self.publisher_base_ctrl.publish(base_ctrl_msg)
                        time.sleep(1.0)
                        overlay_text_msg.data = 'STATE : Grasp Object\nGrasped correct object !!!!!'
                        self.publisher_overlaytext.publish(overlay_text_msg)
                        robot_motion_msg.data = 'INITIAL_POSE'
                        self.publisher_robot_motion.publish(robot_motion_msg)
                        Next_State = 'send'
                        return 'finish'

            if not res_grasp_judge:
                Next_State = "give_up"
                return 'finish'

        Next_State = 'send'
        return 'finish'

class Ros2Send(Node):
    def __init__(self):
        super().__init__('send_state')
        self.publisher_overlaytext = self.create_publisher(String, 'overlay_text', 10)
        self.publisher_task_msg = self.create_publisher(String, 'task_message', 10)
        self.standing_position_estimation_client = self.create_client(self.get_service_type('standing_position'), 'standing_position_estimation_service')
        self.move_processing_client = self.create_client(self.get_service_type('move_pose'), 'move_processing_service')
        self.search_most_closest_point_client = self.create_client(self.get_service_type('search_closest_point'), 'search_most_closest_point_service')
        self.wait_for_frame_exists_client = self.create_client(self.get_service_type('wait_for_frame'), 'wait_for_frame_exists_service')
        self.box_entry_gate_detection_ctrl_client = self.create_client(self.get_service_type('box_detection_ctrl'), 'box_entry_gate_detection_ctrl_service')
        self.wait_for_frame_exists_placeable_client = self.create_client(self.get_service_type('wait_for_frame'), 'wait_for_frame_exists_placeable_service')
        self.placeable_position_estimator_ctrl_client = self.create_client(self.get_service_type('place_position_estimator_ctrl'), 'place_position_estimator_ctrl_service')
        self.gripper_move_to_target_client = self.create_client(self.get_service_type('gripper_move'), 'gripper_move_to_target_service')
        self.gripper_open_client = self.create_client(self.get_service_type('gripper_command'), 'gripper_open_service')
        self.obj_depth_measure_client = self.create_client(self.get_service_type('get_float'), 'get_obj_depth_service')
        self.send_robot_motion_publisher = self.create_publisher(String, 'robot_motion_command', 10)

    def get_service_type(self, service_name):
        if service_name == 'search_closest_point':
            from your_package.srv import SearchClosestPoint
            return SearchClosestPoint
        elif service_name == 'wait_for_frame':
            from your_package.srv import WaitForFrame
            return WaitForFrame
        elif service_name == 'standing_position':
            from your_package.srv import GetPose
            return GetPose
        elif service_name == 'move_pose':
            from your_package.srv import MovePose
            return MovePose
        elif service_name == 'box_detection_ctrl':
            from std_srvs.srv import SetBool
            return SetBool
        elif service_name == 'place_position_estimator_ctrl':
            from std_srvs.srv import SetBool
            return SetBool
        elif service_name == 'gripper_move':
            from your_package.srv import MoveToPose
            return MoveToPose
        elif service_name == 'gripper_command':
            from std_srvs.srv import Trigger
            return Trigger
        elif service_name == 'get_float':
            from your_package.srv import GetFloat
            return GetFloat
        else:
            return None

    async def call_service(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{client.srv_name} service not available, waiting again...')
        future = client.call_async(request)
        await future
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Exception while calling service {client.srv_name}: {future.exception()}')
            return None

    async def execute(self):
        global Next_State, Target_Put_Frame_Name, Pointing_Position2, Pointing_Direction2
        overlay_text_msg = String()
        overlay_text_msg.data = 'STATE : Send Object'
        self.publisher_overlaytext.publish(overlay_text_msg)
        self.get_logger().info('Published: "STATE : Send Object" to overlay_text')

        if Pointing_Position2.x == 0.0 and Pointing_Position2.y == 0.0 and Pointing_Position2.z == 0.0:
            Next_State = 'give_up'
            return 'finish'

        estimated_destination_point = Point()
        if Pointing_Direction2 is not None:
            estimated_destination_point.x = Pointing_Position2.x + math.cos(math.radians(Pointing_Direction2))
            estimated_destination_point.y = Pointing_Position2.y + math.sin(math.radians(Pointing_Direction2))
        else:
            self.get_logger().warn("Pointing_Direction2 is None, cannot estimate destination.")
            Next_State = 'give_up'
            return 'finish'

        search_closest_request = self.get_service_type('search_closest_point').Request()
        search_closest_request.target_point = estimated_destination_point
        search_closest_response = await self.call_service(self.search_most_closest_point_client, search_closest_request)

        if search_closest_response is not None and search_closest_response.closest_frame:
            Target_Put_Frame_Name = search_closest_response.closest_frame
            self.get_logger().info(f"TARGET_PUT_FRAME: {Target_Put_Frame_Name}")
        else:
            Target_Put_Frame_Name = 'FAILURE'
            self.get_logger().warn("Failed to find closest put frame.")

        if Target_Put_Frame_Name == 'FAILURE':
            Next_State = 'give_up'
            return 'finish'

        if not Target_Put_Frame_Name:
            Next_State = 'give_up'
            return 'finish'

        wait_frame_request = self.get_service_type('wait_for_frame').Request()
        wait_frame_request.frame_name = Target_Put_Frame_Name
        wait_frame_request.timeout = 10.0
        wait_frame_response = await self.call_service(self.wait_for_frame_exists_client, wait_frame_request)
        if wait_frame_response is None or not wait_frame_response.exists:
            Next_State = 'give_up'
            return 'finish'

        if Target_Put_Frame_Name == 'DestinationCandidatesPosition10':
            Next_State = 'give_up'
            return 'finish'

        elif Target_Put_Frame_Name == 'DestinationCandidatesPosition09':#ダンボールの場合
            overlay_text_msg.data = 'STATE : Send\nMove to box position.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            self.get_logger().info('Published: "STATE : Send\nMove to box position." to overlay_text')

            standing_position_request = self.get_service_type('standing_position').Request()
            standing_position_request.target_frame = Target_Put_Frame_Name
            standing_position_request.distance = 1.0
            standing_position_response = await self.call_service(self.standing_position_estimation_client, standing_position_request)

            if standing_position_response is not None:
                move_request = self.get_service_type('move_pose').Request()
                move_request.target_pose = standing_position_response.result
                move_response = await self.call_service(self.move_processing_client, move_request)
                if move_response is None or not move_response.success:
                    Next_State = 'give_up'
                    return 'finish'
            else:
                Next_State = 'give_up'
                return 'finish'

            overlay_text_msg.data = 'STATE : Send\nPuting target object to this box.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            self.get_logger().info('Published: "STATE : Send\nPuting target object to this box." to overlay_text')

            move_to_target_request = self.get_service_type('gripper_move').Request()
            move_to_target_request.target_frame = Target_Put_Frame_Name
            move_to_target_request.relative_position.y = 0.05
            move_to_target_request.relative_position.z = 0.5
            res_gripper_response = await self.call_service(self.gripper_move_to_target_client, move_to_target_request)
            res_gripper = res_gripper_response is not None and res_gripper_response.success

            if not res_gripper:
                Next_State = 'give_up'
                return 'finish'
            else:
                time.sleep(3.0)
                await self.call_service(self.gripper_open_client, self.get_service_type('gripper_command').Request())
                time.sleep(1.5)
                task_msg = String()
                task_msg.data = "Task_finished"
                self.publisher_task_msg.publish(task_msg)
                Next_State = 'finish'
                return 'finish'

        obj_depth_request = self.get_service_type('get_float').Request()
        obj_depth_response = await self.call_service(self.obj_depth_measure_client, obj_depth_request)
        object_z = 0.0
        if obj_depth_response is not None:
            object_z = obj_depth_response.data

        send_robot_motion_msg = String()
        send_robot_motion_msg.data = 'INITIAL_POSE'
        self.send_robot_motion_publisher.publish(send_robot_motion_msg)
        self.get_logger().info('Published: "INITIAL_POSE" to robot_motion_command')

        overlay_text_msg.data = 'STATE : Send\nMove to pointed position.'
        self.publisher_overlaytext.publish(overlay_text_msg)
        self.get_logger().info('Published: "STATE : Send\nMove to pointed position." to overlay_text')

        standing_position_request.target_frame = Target_Put_Frame_Name
        standing_position_request.distance = 0.9
        standing_position_response = await self.call_service(self.standing_position_estimation_client, standing_position_request)

        if standing_position_response is not None:
            move_request = self.get_service_type('move_pose').Request()
            move_request.target_pose = standing_position_response.result
            move_response = await self.call_service(self.move_processing_client, move_request)
            if move_response is None or not move_response.success:
                Next_State = 'give_up'
                return 'finish'
        else:
            Next_State = 'give_up'
            return 'finish'

        if Target_Put_Frame_Name in ['DestinationCandidatesPosition01', 'DestinationCandidatesPosition02', 'DestinationCandidatesPosition03']:
            overlay_text_msg.data = 'STATE : Send\nEstimate trash putable position.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            send_robot_motion_msg.data = 'DETECTING_BOX_POSE'
            self.send_robot_motion_publisher.publish(send_robot_motion_msg)
            time.sleep(2.0)
            box_detect_ctrl_request = self.get_service_type('box_detection_ctrl').Request()
            box_detect_ctrl_request.data = True
            await self.call_service(self.box_entry_gate_detection_ctrl_client, box_detect_ctrl_request)
            time.sleep(1.0)
            wait_frame_placeable_request = self.get_service_type('wait_for_frame').Request()
            wait_frame_placeable_request.frame_name = "placeable_point"
            wait_frame_placeable_request.timeout = 30.0
            get_box_frame_response = await self.call_service(self.wait_for_frame_exists_placeable_client, wait_frame_placeable_request)
            box_detect_ctrl_request.data = False
            await self.call_service(self.box_entry_gate_detection_ctrl_client, box_detect_ctrl_request)
            if get_box_frame_response is None or not get_box_frame_response.exists:
                Next_State = 'give_up'
                return 'finish'
        else:
            overlay_text_msg.data = 'STATE : Send\nEstimate putable position.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            send_robot_motion_msg.data = 'DETECTING_POSE'
            self.send_robot_motion_publisher.publish(send_robot_motion_msg)
            time.sleep(2.0)
            place_position_ctrl_request = self.get_service_type('place_position_estimator_ctrl').Request()
            place_position_ctrl_request.data = True
            await self.call_service(self.placeable_position_estimator_ctrl_client, place_position_ctrl_request)
            time.sleep(1.0)
            wait_frame_placeable_request.frame_name = "placeable_point"
            get_frame_response = await self.call_service(self.wait_for_frame_exists_placeable_client, wait_frame_placeable_request)
            place_position_ctrl_request.data = False
            await self.call_service(self.placeable_position_estimator_ctrl_client, place_position_ctrl_request)
            if get_frame_response is None or not get_frame_response.exists:
                Next_State = 'give_up'
                return 'finish'

        overlay_text_msg.data = 'STATE : Send\nPutting target object to this place.'
        self.publisher_overlaytext.publish(overlay_text_msg)
        time.sleep(1.0)
        move_to_target_request = self.get_service_type('gripper_move').Request()
        move_to_target_request.target_frame = "placeable_point"
        move_to_target_request.relative_position.z = object_z + 0.01
        res_gripper_response = await self.call_service(self.gripper_move_to_target_client, move_to_target_request)
        res_gripper = res_gripper_response is not None and res_gripper_response.success

        if not res_gripper:
            Next_State = 'give_up'
            return 'finish'
        else:
            time.sleep(3.0)
            await self.call_service(self.gripper_open_client, self.get_service_type('gripper_command').Request())
            time.sleep(1.0)
            task_msg = String()
            task_msg.data = "Task_finished"
            self.publisher_task_msg.publish(task_msg)
            Next_State = 'finish'
            return 'finish'

class Ros2IsFinish(Node):
    def __init__(self):
        super().__init__('is_finish_state')
        self.publisher_overlaytext = self.create_publisher(String, 'overlay_text', 10)
        self.publisher_task_msg = self.create_publisher(String, 'task_message', 10)
        # self.move_cancel_client = self.create_client(self.get_service_type('move_cancel'), 'move_cancel_service')
        # self.processing_kill_publisher = self.create_publisher(String, 'processing_command', 10)
        # self.get_task_failed_state_client = self.create_client(self.get_service_type('get_bool'), 'get_task_failed_state_service')

    def get_service_type(self, service_name):
        # if service_name == 'move_cancel':
        #     from std_srvs.srv import Trigger
        #     return Trigger
        # elif service_name == 'get_bool':
        #     from std_srvs.srv import TriggerGetBool
        #     return TriggerGetBool
        return None

    async def call_service(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{client.srv_name} service not available, waiting again...')
        future = client.call_async(request)
        await future
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Exception while calling service {client.srv_name}: {future.exception()}')
            return None

    async def execute(self):
        global Next_State
        overlay_text_msg = String()

        if Next_State == "search":
            overlay_text_msg.data = 'STATE : Is_finish\nSuccessful acquisition of skeleton!!!'
            self.publisher_overlaytext.publish(overlay_text_msg)
            return 'continue_task'
        elif Next_State == "grasp":
            overlay_text_msg.data = 'STATE : Is_finish\nSuccessful search object!!!'
            self.publisher_overlaytext.publish(overlay_text_msg)
            return 'continue_task'
        elif Next_State == "send":
            overlay_text_msg.data = 'STATE : Is_finish\nSuccessful object gripping!!!'
            self.publisher_overlaytext.publish(overlay_text_msg)
            time.sleep(1.0)
            task_failed = False # task_commom.get_task_failed_state() の代替
            if not task_failed:
                overlay_text_msg.data = 'STATE : Is_finish\nSuccessful object gripping!!!'
                self.publisher_overlaytext.publish(overlay_text_msg)
                return 'continue_task'
            else:
                overlay_text_msg.data = 'STATE : Is_finish\nFailure to grasp objects.'
                self.publisher_overlaytext.publish(overlay_text_msg)
                # processing_kill_msg = String()
                # processing_kill_msg.data = "kill"
                # self.processing_kill_publisher.publish(processing_kill_msg)
                return 'session_finish'
        elif Next_State == "finish":
            time.sleep(1.0)
            task_failed = False # task_commom.get_task_failed_state() の代替
            if not task_failed:
                overlay_text_msg.data = 'STATE : Is_finish\nSuccessful to put a target object!!!'
            else:
                overlay_text_msg.data = 'STATE : Is_finish\nFailed to place the target object.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            return 'session_finish'
        elif Next_State == "give_up":
            overlay_text_msg.data = 'STATE : Is_finish\nGive up.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            task_msg = String()
            task_msg.data = "Give_up"
            self.publisher_task_msg.publish(task_msg)
            # move_cancel_request = Trigger.Request()
            # await self.call_service(self.move_cancel_client, move_cancel_request)
            # processing_kill_msg.data = "kill"
            # self.processing_kill_publisher.publish(processing_kill_msg)
            time.sleep(2)
            return 'session_finish'
        else:
            overlay_text_msg.data = 'STATE : Is_finish\nTime is up.'
            self.publisher_overlaytext.publish(overlay_text_msg)
            # move_cancel_request = Trigger.Request()
            # await self.call_service(self.move_cancel_client, move_cancel_request)
            # processing_kill_msg.data = "kill"
            # self.processing_kill_publisher.publish(processing_kill_msg)
            time.sleep(2)
            return 'session_finish'

class Ros2StateMachine(Node):
    def __init__(self):
        super().__init__('cleanup_task_sm')
        self.sm = None
        self.introspection_server = None

    def create_state_machine(self):
        self.sm = StateMachine(outcomes=['finish'])
        with self.sm:
            StateMachine.add('Start', Ros2Start(), transitions={'start_OK': 'Check'})
            StateMachine.add('Check', Ros2Check(), transitions={'receive': 'Receive', 'search': 'Search', 'grasp': 'Grasp', 'send': 'Send'})
            StateMachine.add('Receive', Ros2Receive(), transitions={'finish': 'Is_finish'})
            StateMachine.add('Search', Ros2Search(), transitions={'finish': 'Is_finish'})
            StateMachine.add('Grasp', Ros2Grasp(), transitions={'finish': 'Is_finish'})
            StateMachine.add('Send', Ros2Send(), transitions={'finish': 'Is_finish'})
            StateMachine.add('Is_finish', Ros2IsFinish(), transitions={'continue_task': 'Check', 'session_finish': 'Start'})
        return self.sm

    async def execute_state_machine(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self.sm)
        try:
            await executor.spin_until_future_complete(self.sm.execute_async())
        finally:
            executor.shutdown()

    def start_introspection_server(self):
        self.introspection_server = IntrospectionServer('server_name', self.sm, '/CLEANUP_TASK')
        self.introspection_server.start()

    def stop_introspection_server(self):
        if self.introspection_server:
            self.introspection_server.stop()

async def main(args=None):
    rclpy.init(args=args)
    state_machine_node = Ros2StateMachine()
    try:
        sm = state_machine_node.create_state_machine()
        state_machine_node.start_introspection_server()
        await state_machine_node.execute_state_machine()
    finally:
        state_machine_node.stop_introspection_server()
        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())