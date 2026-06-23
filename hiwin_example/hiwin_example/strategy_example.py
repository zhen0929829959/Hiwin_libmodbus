#!/usr/bin/env python3
import time
import json
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from hiwin_interfaces.srv import RobotCommand

DEFAULT_VELOCITY = 10
DEFAULT_ACCELERATION = 10

PHOTO_POSE = [0.00, -3.00, 15.50, 0.00, -75.00, -90.00]

APRILTAG_TOPIC = '/apriltag/pose_base'

TARGET_Z = 300.0
TARGET_RX = 180.0
TARGET_RY = 0.0
TARGET_RZ = 0.0   


class States(Enum):
    INIT = 0
    MOVE_TO_PHOTO_POSE = 1
    WAIT_APRILTAG = 2
    MOVE_ABOVE_TAG = 3
    CHECK_POSE = 4
    FINISH = 5


class ExampleStrategy(Node):

    def __init__(self):
        super().__init__('example_strategy')

        self.hiwin_client = self.create_client(
            RobotCommand,
            'hiwinmodbus_service'
        )

        self.latest_tag_position_mm = None
        self.has_tag = False
        self.printed_tag_once = False

        self.tag_sub = self.create_subscription(
            String,
            APRILTAG_TOPIC,
            self.tag_pose_callback,
            10
        )

    def tag_pose_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if "position_mm" not in data:
                self.get_logger().error('AprilTag data has no position_mm')
                return

            self.latest_tag_position_mm = data["position_mm"]
            self.has_tag = True

            if not self.printed_tag_once:
                self.get_logger().info(
                    f'AprilTag position_mm: {self.latest_tag_position_mm}'
                )
                self.printed_tag_once = True

        except Exception as e:
            self.get_logger().error(f'Failed to parse AprilTag data: {e}')

    def _state_machine(self, state: States) -> States:

        if state == States.INIT:
            self.get_logger().info('INIT')
            return States.MOVE_TO_PHOTO_POSE

        elif state == States.MOVE_TO_PHOTO_POSE:
            self.get_logger().info('MOVE_TO_PHOTO_POSE')

            req = self.generate_robot_request(
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=PHOTO_POSE
            )

            res = self.call_hiwin(req)
            self.get_logger().info(f'Move response: {res}')

            time.sleep(5)
            return States.WAIT_APRILTAG

        elif state == States.WAIT_APRILTAG:
            self.get_logger().info(f'WAIT_APRILTAG: {APRILTAG_TOPIC}')

            start_time = time.time()

            while rclpy.ok() and not self.has_tag:
                time.sleep(0.05)

                if time.time() - start_time > 10.0:
                    self.get_logger().error('AprilTag timeout!')
                    return States.CHECK_POSE

            self.get_logger().info('AprilTag detected!')
            return States.MOVE_ABOVE_TAG

        elif state == States.MOVE_ABOVE_TAG:
            self.get_logger().info('MOVE_ABOVE_TAG')

            tag_x, tag_y, tag_z = self.latest_tag_position_mm

            pose = Twist()
            pose.linear.x = float(tag_x)
            pose.linear.y = float(tag_y)
            pose.linear.z = TARGET_Z

            pose.angular.x = TARGET_RX
            pose.angular.y = TARGET_RY
            pose.angular.z = TARGET_RZ

            self.get_logger().info(
                f'Target pose: '
                f'x={pose.linear.x:.3f}, '
                f'y={pose.linear.y:.3f}, '
                f'z={pose.linear.z:.3f}, '
                f'rx={pose.angular.x:.3f}, '
                f'ry={pose.angular.y:.3f}, '
                f'rz={pose.angular.z:.3f}'
            )

            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                tool=7,
                base=0,
                pose=pose
            )

            res = self.call_hiwin(req)
            self.get_logger().info(f'Move above tag response: {res}')

            time.sleep(3)
            return States.CHECK_POSE

        elif state == States.CHECK_POSE:
            self.get_logger().info('CHECK_POSE')

            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.CHECK_POSE
            )

            res = self.call_hiwin(req)

            if res:
                self.get_logger().info(
                    f'Current position: {list(res.current_position)}'
                )

            return States.FINISH

        else:
            self.get_logger().error('Input state not supported!')
            return None

    def _main_loop(self):
        state = States.INIT

        while rclpy.ok() and state != States.FINISH:
            state = self._state_machine(state)

            if state is None:
                self.get_logger().error('State machine stopped.')
                break

        self.get_logger().info('FINISH')

    def _wait_for_future_done(self, future: Future, timeout=-1):
        time_start = time.time()

        while not future.done():
            time.sleep(0.01)

            if timeout > 0 and time.time() - time_start > timeout:
                self.get_logger().error('Wait for service timeout!')
                return False

        return True

    def generate_robot_request(
        self,
        holding=True,
        cmd_mode=RobotCommand.Request.PTP,
        cmd_type=RobotCommand.Request.POSE_CMD,
        velocity=DEFAULT_VELOCITY,
        acceleration=DEFAULT_ACCELERATION,
        tool=7,
        base=0,
        digital_input_pin=2,
        digital_output_pin=6,
        digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
        pose=Twist(),
        joints=[float('inf')] * 6,
        circ_s=[],
        circ_end=[],
        jog_joint=6,
        jog_dir=0,
        move_dir="z",
        move_dis=0.01
    ):
        request = RobotCommand.Request()

        request.digital_input_pin = digital_input_pin
        request.digital_output_pin = digital_output_pin
        request.digital_output_cmd = digital_output_cmd
        request.acceleration = acceleration
        request.jog_joint = jog_joint
        request.velocity = velocity
        request.tool = tool
        request.base = base
        request.cmd_mode = cmd_mode
        request.cmd_type = cmd_type
        request.circ_end = circ_end
        request.jog_dir = jog_dir
        request.holding = holding
        request.joints = joints
        request.circ_s = circ_s
        request.pose = pose
        request.move_dir = move_dir
        request.move_dis = move_dis

        return request

    def call_hiwin(self, req):
        while not self.hiwin_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.hiwin_client.call_async(req)

        if self._wait_for_future_done(future):
            return future.result()

        return None

    def start_main_loop_thread(self):
        self.main_loop_thread = Thread(target=self._main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()


def main(args=None):
    rclpy.init(args=args)

    strategy = ExampleStrategy()
    strategy.start_main_loop_thread()

    while rclpy.ok() and strategy.main_loop_thread.is_alive():
        rclpy.spin_once(strategy, timeout_sec=0.1)

    strategy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()