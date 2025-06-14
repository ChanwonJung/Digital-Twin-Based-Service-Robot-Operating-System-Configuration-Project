from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
import time


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        # PD control related variables
        self.last_error = 0 
        # self.MAX_VEL = 0.1  ## 기존 코드
        self.MAX_VEL = 0.25    ## 수정 코드

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()


        # 신호등 subscription 생성
        self.sub_traffic_light = self.create_subscription(String, '/traffic_light/status', self.callback_traffic_light_status, 1)
        self.traffic_light_status = 'RED'  # RED, YELLOW, GREEN, NONE
        self.traffic_light_stop_bool = False

        # 표지판 subscription 생성
        self.sub_traffic_sign = self.create_subscription(String, '/detect/traffic_sign', self.traffic_callback, 10)
        self.traffic = ''

        # 감속 지속 시간 설정
        self.slowdown_start_time = None
        self.slowdown_duration = 10

        # 횡단보도 일시정지 모드, 일시 정지 여부
        self.crosswork_stop_mode = False
        self.crosswork_stop_bool = False


    '''신호등 subscription callback'''
    def callback_traffic_light_status(self, msg):
        self.traffic_light_status = msg.data


    '''표지판 subscription callback'''
    def traffic_callback(self, msg):

        # 횡단보도, 보행자 동시에 인식 시 횡단보도 일시정지 모드 True
        if self.traffic == 'pedestrian':
            if msg.data == 'crosswalk':
                self.crosswork_stop_mode = True
        elif self.traffic == 'crosswalk':
            if msg.data == 'pedestrian':
                self.crosswork_stop_mode = True

        # traffic 수신
        self.traffic = msg.data
        self.get_logger().info(f'📥 [Traffic 수신] "{msg.data}"')


    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data


    '''메인 함수. 속도를 조절하고 퍼블리시 하는 함수'''
    def callback_follow_lane(self, desired_center):
        """
        Receive lane center data to generate lane following control commands.
        If avoidance mode is enabled, lane following control is ignored.
        """
        if self.avoid_active:
            return

        center = desired_center.data
        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        # 속도 계산
        twist = Twist()
        # twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)  ## 기존 코드(상한값 0.05)
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.20)    ## 수정 코드
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)


        '''신호등 코드'''
        if self.traffic_light_status == 'RED' and not self.traffic_light_stop_bool:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('🔴 RED light - Stopping')
            self.pub_cmd_vel.publish(twist)
            return
        
        elif self.traffic_light_status == 'YELLOW' and not self.traffic_light_stop_bool:
            linear_speed = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
            twist.linear.x = linear_speed * 0.3
            twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            self.get_logger().info(f'🟡 YELLOW light - Slowing down to {twist.linear.x:.3f} m/s')
            self.pub_cmd_vel.publish(twist)
            return

        elif self.traffic_light_status == 'GREEN' and not self.traffic_light_stop_bool:
            self.traffic_light_stop_bool = True
            self.get_logger().info('🟢 GREEN light - Go!')


        '''감속, 가속 코드'''
        # 과속방지턱 감지된 경우 속도 감속
        if self.traffic == 'speed_bump':
            if self.slowdown_start_time is None:
                self.slowdown_start_time = time.time()
                self.get_logger().info(f"⏬ Speed Bump Detected. Slowing down for {self.slowdown_duration} seconds.")
            # 감속 지속 시간 동안 감속 
            if time.time() - self.slowdown_start_time < self.slowdown_duration:
                twist.linear.x = 0.05
            else:
                self.traffic = ''  # 감속 종료 후 상태 초기화
                self.slowdown_start_time = None
                self.get_logger().info("✅ Speed Bump Slowdown Finished.")
        
        # 어린이 보호구역 감지된 경우 속도 감속
        elif self.traffic == 'school_zone':
            if self.slowdown_start_time is None:
                self.slowdown_start_time = time.time()
                self.get_logger().info(f"⏬ School Zone Detected. Slowing down for {self.slowdown_duration} seconds.")
            # 감속 지속 시간 동안 감속 
            if time.time() - self.slowdown_start_time < self.slowdown_duration:
                twist.linear.x = 0.05
            else:
                self.traffic = ''  # 감속 종료 후 상태 초기화
                self.slowdown_start_time = None
                self.get_logger().info("✅ School Zone Slowdown Finished.")

        # 속도 100 감지된 경우 속도 가속
        elif self.traffic == 'speed_limit_100':
            twist.linear.x = 0.5
            self.get_logger().info("⏫  Speed 100 Detected. Speed UP!")

        # 속도 30 감지된 경우 속도 감속
        elif self.traffic == 'speed_limit_30':
            if self.slowdown_start_time is None:
                self.slowdown_start_time = time.time()
                self.get_logger().info(f"⏬  Speed 30 Detected. Slowing DOWN for {self.slowdown_duration} seconds.")
            # 감속 지속 시간 동안 감속
            if time.time() - self.slowdown_start_time < self.slowdown_duration:
                twist.linear.x = 0.1
            else:
                self.traffic = ''  # 감속 종료 후 상태 초기화
                self.slowdown_start_time = None
                self.get_logger().info("✅ Speed 30 Slowdown Finished!")
        
        # 보행자와 횡단보도가 동시에 감지되면 일시 정지
        elif self.crosswork_stop_mode and not self.crosswork_stop_bool:
            if self.slowdown_start_time is None:
                self.slowdown_start_time = time.time()
                self.get_logger().info(f"🛑 Crosswork Detected. Stop for {self.slowdown_duration} seconds.")
                self.get_logger().info("✅ AAA")
            # 감속 지속 시간 동안 정지
            if time.time() - self.slowdown_start_time < self.slowdown_duration:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("✅ BBB")
            else:
                self.traffic = ''  # 정지 종료 후 상태 초기화
                self.slowdown_start_time = None
                self.crosswork_stop_bool = True
                self.get_logger().info("✅ Crosswork Stop Finished. Go!")
        else:
            # speed_bump가 아닌 다른 상황일 경우 타이머 초기화
            self.slowdown_start_time = None


        # /control/cmd_vel 퍼블리시
        self.pub_cmd_vel.publish(twist)
        # self.get_logger().info(f'linear.x = {twist.linear.x:.3f}, angular.z = {twist.angular.z:.3f}')  # 속도 로그 출력
        

    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)


    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')


    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
