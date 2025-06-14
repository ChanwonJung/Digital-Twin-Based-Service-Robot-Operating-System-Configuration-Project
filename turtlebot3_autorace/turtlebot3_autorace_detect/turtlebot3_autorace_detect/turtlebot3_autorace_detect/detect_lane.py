import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import String


class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        parameter_descriptor_hue = ParameterDescriptor(
            description='hue parameter range',
            integer_range=[IntegerRange(
                from_value=0,
                to_value=179,
                step=1)]
        )
        parameter_descriptor_saturation_lightness = ParameterDescriptor(
            description='saturation and lightness range',
            integer_range=[IntegerRange(
                from_value=0,
                to_value=255,
                step=1)]
        )
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detect.lane.white.hue_l', 0,
                    parameter_descriptor_hue),
                ('detect.lane.white.hue_h', 179,
                    parameter_descriptor_hue),
                ('detect.lane.white.saturation_l', 0,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.saturation_h', 70,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_l', 105,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.hue_l', 10,
                    parameter_descriptor_hue),
                ('detect.lane.yellow.hue_h', 127,
                    parameter_descriptor_hue),
                ('detect.lane.yellow.saturation_l', 70,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.saturation_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_l', 95,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('is_detection_calibration_mode', False)
            ]
        )

        self.hue_white_l = self.get_parameter(
            'detect.lane.white.hue_l').get_parameter_value().integer_value
        self.hue_white_h = self.get_parameter(
            'detect.lane.white.hue_h').get_parameter_value().integer_value
        self.saturation_white_l = self.get_parameter(
            'detect.lane.white.saturation_l').get_parameter_value().integer_value
        self.saturation_white_h = self.get_parameter(
            'detect.lane.white.saturation_h').get_parameter_value().integer_value
        self.lightness_white_l = self.get_parameter(
            'detect.lane.white.lightness_l').get_parameter_value().integer_value
        self.lightness_white_h = self.get_parameter(
            'detect.lane.white.lightness_h').get_parameter_value().integer_value

        self.hue_yellow_l = self.get_parameter(
            'detect.lane.yellow.hue_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter(
            'detect.lane.yellow.hue_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter(
            'detect.lane.yellow.saturation_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter(
            'detect.lane.yellow.saturation_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter(
            'detect.lane.yellow.lightness_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter(
            'detect.lane.yellow.lightness_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_detection_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.cbGetDetectLaneParam)

        self.sub_image_type = 'raw'         # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.cbFindLane, 1
                )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self.cbFindLane, 1
                )

        if self.pub_image_type == 'compressed':
            self.pub_image_lane = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1
                )
        elif self.pub_image_type == 'raw':
            self.pub_image_lane = self.create_publisher(
                Image, '/detect/image_output', 1
                )

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 1
                    )
                self.pub_image_yellow_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub2/compressed', 1
                    )
            elif self.pub_image_type == 'raw':
                self.pub_image_white_lane = self.create_publisher(
                    Image, '/detect/image_output_sub1', 1
                    )
                self.pub_image_yellow_lane = self.create_publisher(
                    Image, '/detect/image_output_sub2', 1
                    )

        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)

        self.pub_yellow_line_reliability = self.create_publisher(
            UInt8, '/detect/yellow_line_reliability', 1
            )

        self.pub_white_line_reliability = self.create_publisher(
            UInt8, '/detect/white_line_reliability', 1
            )

        self.pub_lane_state = self.create_publisher(UInt8, '/detect/lane_state', 1)

        self.cvBridge = CvBridge()

        self.counter = 1

        self.window_width = 1000.
        self.window_height = 600.

        self.reliability_white_line = 100
        self.reliability_yellow_line = 100

        self.mov_avg_left = np.empty((0, 3))
        self.mov_avg_right = np.empty((0, 3))

        # 표지판 subscription 생성
        self.sub_traffic_sign = self.create_subscription(String, '/detect/traffic_sign', self.traffic_callback, 10)

        # 표지판 정보 문자열
        self.traffic = ''

        # 오른쪽 모드, 왼쪽 모드 bool
        self.right_mode = False
        self.left_mode = False


    # 표지판 subscription callback
    def traffic_callback(self, msg):
        
        # 교차로 모드 (오른쪽 or 왼쪽)
        if self.traffic == 'intersection':
            if msg.data == 'right':
                # 표지판이 'intersection' 일 때 다음으로 'right' 표지판이 수신되면 오른쪽 모드 True
                self.right_mode = True
                self.get_logger().info(f'➡️ [Right Mode ON!]')
            elif msg.data == 'left':
                # 표지판이 'intersection' 일 때 다음으로 'left' 표지판이 수신되면 왼쪽 모드 True
                self.left_mode = True
                self.get_logger().info(f'⬅️ [Left Mode ON!]')
        
        if self.traffic == 'speed_bump':
            self.right_mode = False
            self.left_mode = False
            
        # traffic 수신
        self.traffic = msg.data
        self.get_logger().info(f'📥 [Traffic 수신] "{msg.data}"')


    def cbGetDetectLaneParam(self, parameters):
        for param in parameters:
            self.get_logger().info(f'Parameter name: {param.name}')
            self.get_logger().info(f'Parameter value: {param.value}')
            self.get_logger().info(f'Parameter type: {param.type_}')
            if param.name == 'detect.lane.white.hue_l':
                self.hue_white_l = param.value
            elif param.name == 'detect.lane.white.hue_h':
                self.hue_white_h = param.value
            elif param.name == 'detect.lane.white.saturation_l':
                self.saturation_white_l = param.value
            elif param.name == 'detect.lane.white.saturation_h':
                self.saturation_white_h = param.value
            elif param.name == 'detect.lane.white.lightness_l':
                self.lightness_white_l = param.value
            elif param.name == 'detect.lane.white.lightness_h':
                self.lightness_white_h = param.value
            elif param.name == 'detect.lane.yellow.hue_l':
                self.hue_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.hue_h':
                self.hue_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.saturation_l':
                self.saturation_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.saturation_h':
                self.saturation_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.lightness_l':
                self.lightness_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.lightness_h':
                self.lightness_yellow_h = param.value
            return SetParametersResult(successful=True)

    def cbFindLane(self, image_msg):
        # Change the frame rate by yourself. Now, it is set to 1/3 (10fps).
        # Unappropriate value of frame rate may cause huge delay on entire recognition process.
        # This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        white_fraction, cv_white_lane = self.maskWhiteLane(cv_image)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_image)

        try:
            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.fit_from_lines(
                    self.left_fit, cv_yellow_lane)
                self.mov_avg_left = np.append(
                    self.mov_avg_left, np.array([self.left_fit]), axis=0
                    )

            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.fit_from_lines(
                    self.right_fit, cv_white_lane)
                self.mov_avg_right = np.append(
                    self.mov_avg_right, np.array([self.right_fit]), axis=0
                    )
        except Exception:
            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
                self.mov_avg_left = np.array([self.left_fit])

            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
                self.mov_avg_right = np.array([self.right_fit])

        MOV_AVG_LENGTH = 5

        self.left_fit = np.array([
            np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])
            ])
        self.right_fit = np.array([
            np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])
            ])

        if self.mov_avg_left.shape[0] > 1000:
            self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]

        if self.mov_avg_right.shape[0] > 1000:
            self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]

        self.make_lane(cv_image, white_fraction, yellow_fraction)
        
    ### 수정 코드
    def maskWhiteLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 평균 밝기 계산 (V 채널 사용)
        avg_brightness = np.mean(hsv[:, :, 2])

        # 자동 하한 조정: 새로운 데이터 기반 선형 근사식 적용
        predicted = int(1.46 * avg_brightness + 20)
        predicted = max(60, min(predicted, 255))  # 안정적 범위 제한

        # 현재 값에서 목표 값으로 점진적 보정
        if self.lightness_white_l < predicted:
            self.lightness_white_l = min(self.lightness_white_l + 10, predicted)
        elif self.lightness_white_l > predicted:
            self.lightness_white_l = max(self.lightness_white_l - 10, predicted)

        # self.get_logger().info(f'[White] Average Brightness: {avg_brightness:.2f}')
        # self.get_logger().info(f'[White] Average Brightness: {avg_brightness:.2f}, 'f'lightness_white_l: {self.lightness_white_l}')
        
        # 마스킹
        lower_white = np.array([self.hue_white_l, self.saturation_white_l, self.lightness_white_l])
        upper_white = np.array([self.hue_white_h, self.saturation_white_h, self.lightness_white_h])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        fraction_num = np.count_nonzero(mask)

        # 신뢰도 조정 (기존 유지)
        how_much_short = 600 - np.count_nonzero(np.any(mask, axis=1))
        self.reliability_white_line = max(0, min(100, self.reliability_white_line + (5 if how_much_short <= 100 else -5)))

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line
        self.pub_white_line_reliability.publish(msg_white_line_reliability)

        if self.is_calibration_mode:
            pub_mask = self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg') if self.pub_image_type == 'compressed' else self.cvBridge.cv2_to_imgmsg(mask, 'bgr8')
            self.pub_image_white_lane.publish(pub_mask)

        return fraction_num, mask
    
    def maskYellowLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        avg_brightness = np.mean(hsv[:, :, 2])
        
        # ✅ 자동 하한 조정 (밝기에 따라 목표 계산)
        target_lightness_l = int(avg_brightness * 1.2)
        target_lightness_l = max(60, min(target_lightness_l, 130))  # 안전한 범위 제한

        # 현재 값에서 목표로 부드럽게 이동 (ex. 1프레임에 5씩)
        if self.lightness_yellow_l < target_lightness_l:
            self.lightness_yellow_l = min(self.lightness_yellow_l + 10, target_lightness_l)
        elif self.lightness_yellow_l > target_lightness_l:
            self.lightness_yellow_l = max(self.lightness_yellow_l - 10, target_lightness_l)

        # self.get_logger().info(f'[Yellow] Average Brightness: {avg_brightness:.2f}, 'f'lightness_yellow_l: {self.lightness_yellow_l}')
        
        # 마스크 생성
        lower_yellow = np.array([self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l])
        upper_yellow = np.array([self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        fraction_num = np.count_nonzero(mask)

        # 신뢰도 조정 (기존 유지)
        how_much_short = 600 - np.count_nonzero(np.any(mask, axis=1))
        self.reliability_yellow_line = max(0, min(100, self.reliability_yellow_line + (5 if how_much_short <= 100 else -5)))

        msg_yellow_line_reliability = UInt8()
        msg_yellow_line_reliability.data = self.reliability_yellow_line
        self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)

        if self.is_calibration_mode:
            pub_mask = self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg') if self.pub_image_type == 'compressed' else self.cvBridge.cv2_to_imgmsg(mask, 'bgr8')
            self.pub_image_yellow_lane.publish(pub_mask)

        return fraction_num, mask
        # '''


    def fit_from_lines(self, lane_fit, image):
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        lane_inds = (
            (nonzerox >
                (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) &
            (nonzerox <
                (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin))
                )

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        lane_fit = np.polyfit(y, x, 2)

        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit

    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        out_img = np.dstack((img_w, img_w, img_w)) * 255

        midpoint = np.int_(histogram.shape[0] / 2)

        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 20

        window_height = np.int_(img_w.shape[0] / nwindows)

        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        x_current = lane_base

        margin = 50

        minpix = 50

        lane_inds = []

        for window in range(nwindows):
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            cv2.rectangle(
                out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            good_lane_inds = (
                (nonzeroy >= win_y_low) &
                (nonzeroy < win_y_high) &
                (nonzerox >= win_x_low) &
                (nonzerox < win_x_high)
                ).nonzero()[0]

            lane_inds.append(good_lane_inds)

            if len(good_lane_inds) > minpix:
                x_current = np.int_(np.mean(nonzerox[good_lane_inds]))

        lane_inds = np.concatenate(lane_inds)

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
        except Exception:
            lane_fit = self.lane_fit_bef

        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit
    
    def calculate_curvature(self, fit, y_eval=599):
        A = fit[0]
        B = fit[1]
        curvature_radius = ((1 + (2 * A * y_eval + B) ** 2) ** 1.5) / abs(2 * A)
        return curvature_radius
    
    def is_curve(self, left_fit, right_fit, threshold=6000):
        left_radius = self.calculate_curvature(left_fit)
        right_radius = self.calculate_curvature(right_fit)

        is_left_curve = left_radius < threshold
        is_right_curve = right_radius < threshold

        # self.get_logger().info(f'[Curve Check] Radius: L={left_radius:.1f} ({"CURVE" if is_left_curve else "straight"}), 'f'R={right_radius:.1f} ({"CURVE" if is_right_curve else "straight"})')

        return is_left_curve or is_right_curve


    '''메인 함수. 차선을 인식하고 중심선을 계산하는 함수'''
    def make_lane(self, cv_image, white_fraction, yellow_fraction):
        # Create an image to draw the lines on
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])
        centerx = None  # ✅ 중심선 x 좌표 초기화
        lane_state = UInt8()

        # 차선 시각화
        if yellow_fraction > 3000:
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)

        if white_fraction > 3000:
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)


        # 오른쪽 모드일 때 중심선 계산 분기 (오른쪽 흰선을 따라가도록)
        if self.right_mode:
            centerx = np.subtract(self.right_fitx, 230)
            lane_state.data = 3

        # 왼쪽 모드일 때 중심선 계산 분기 (왼쪽 노란선을 따라가도록)
        elif self.left_mode:
            centerx = np.add(self.left_fitx, 230)
            lane_state.data = 1

        # 일반 상황일 때 중심선 계산 분기
        else:
            if self.reliability_white_line > 50 and self.reliability_yellow_line > 50:
                if white_fraction > 3000 and yellow_fraction > 3000:
                    if self.is_curve(self.left_fit, self.right_fit):
                        if self.reliability_white_line >= self.reliability_yellow_line:
                            centerx = np.subtract(self.right_fitx, 230)
                            lane_state.data = 3
                        else:
                            centerx = np.add(self.left_fitx, 230)
                            lane_state.data = 1
                    else:
                        centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
                        lane_state.data = 2
                        pts = np.hstack((pts_left, pts_right))
                        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

                elif white_fraction > 3000:
                    centerx = np.subtract(self.right_fitx, 230)
                    lane_state.data = 3
                elif yellow_fraction > 3000:
                    centerx = np.add(self.left_fitx, 230)
                    lane_state.data = 1

            elif self.reliability_white_line > 50:
                centerx = np.subtract(self.right_fitx, 230)
                lane_state.data = 3

            elif self.reliability_yellow_line > 50:
                centerx = np.add(self.left_fitx, 230)
                lane_state.data = 1

            else:
                self.is_center_x_exist = False
                lane_state.data = 0

        # 중심선 시각화 및 퍼블리시
        if centerx is not None:
            self.is_center_x_exist = True
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

            msg_desired_center = Float64()
            msg_desired_center.data = centerx.item(250)
            self.pub_lane.publish(msg_desired_center)

            # 곡률 로그
            left_curvature = self.calculate_curvature(self.left_fit)
            right_curvature = self.calculate_curvature(self.right_fit)
            # self.get_logger().info(f'[Curvature] Left: {left_curvature:.1f}px, Right: {right_curvature:.1f}px')

        self.pub_lane_state.publish(lane_state)

        # 최종 이미지 합성
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        if self.pub_image_type == 'compressed':
            self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))
        else:
            self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
