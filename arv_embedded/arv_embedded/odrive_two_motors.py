import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_VELOCITY_CONTROL

class DualODriveController(Node):
    def __init__(self):
        super().__init__('dual_odrive_controller')

        #serial may need to be converted to hex?
        self.odrv0 = odrive.find_any(serial_number="odrv0_serial") 
        self.odrv1 = odrive.find_any(serial_number="odrv1_serial")

        self.calibrate_motor()

        self.subscription = self.create_subscription(
            Twist,'cmd_vel',self.cmd_vel_callback,10)

        self.publisher = self.create_publisher(Twist, 'enc_vel', 10)

        self.timer = self.create_timer(0.1, self.publish_enc_vel)

    def calibrate_motor(self):

        self.get_logger().info("Calibrating...")
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
            pass
        self.odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.odrv1.axis0.current_state != AXIS_STATE_IDLE:
            pass
        self.get_logger().info("Calibration complete")

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    def cmd_vel_callback(self, msg):
        left_vel = msg.linear.x - msg.angular.z
        right_vel = msg.linear.x + msg.angular.z

        self.odrv0.axis0.controller.input_vel = left_vel
        self.odrv1.axis0.controller.input_vel = right_vel

    def publish_enc_vel(self):
        enc_vel_left = self.odrv0.axis0.vel_estimate
        enc_vel_right = self.odrv1.axis0.vel_estimate
        msg = Twist()
        msg.linear.x = (enc_vel_left + enc_vel_right) / 2
        msg.angular.z = (enc_vel_right - enc_vel_left) / 2
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualODriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
