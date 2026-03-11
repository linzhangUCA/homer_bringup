import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_about_axis
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sin, cos, pi
import serial

class PicoInterface(Node):
    def __init__(self):
        super().__init__("pico_interface")
        # Create serial communication to Pico
        self.pico_msngr = serial.Serial(
            "/dev/ttyACM0",
            115200,
            timeout=0.01,
        )  # for UART, use ttyAMA0
        self.pico_comm_timer = self.create_timer(0.0133, self.process_pico_message)  # 75 Hz
        # Create target velocity subscriber
        self.cmd_vel_listner = self.create_subscription(
            topic="cmd_vel",
            msg_type=Twist,
            callback=self.set_targ_vels,
            qos_profile=1,
        )
        # variables
        self.encoder_lin_vel = 0.0
        self.encoder_ang_vel = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.targ_lin_vel = 0.0
        self.targ_ang_vel = 0.0
        # constants
        self.get_logger().info("HomeR's motion controller is up.")

    def process_pico_message(self):
        msg_to_pico = f"{self.targ_lin_vel:.3f},{self.targ_ang_vel:.3f}\n"
        self.pico_msngr.write(msg_to_pico.encode("utf-8"))
        if self.pico_msngr.inWaiting() > 0:
            msg_from_pico = self.pico_msngr.readline().decode("utf-8", "ignore").strip()
            if msg_from_pico:
                motion_data = msg_from_pico.split(",")
                if len(motion_data) == 8:
                    try:
                        self.meas_lin_vel = float(motion_data[0])
                        self.meas_ang_vel = float(motion_data[1])
                    except ValueError:
                        self.meas_lin_vel = 0.0
                        self.meas_ang_vel = 0.0
        self.get_logger().debug(
            f"Measured velocity\nlinear: {self.meas_lin_vel}, angular: {self.meas_ang_vel}"
        )

    def set_targ_vels(self, msg):
        targ_lin_vel = msg.linear.x
        targ_ang_vel = msg.angular.z
        self.pico_msngr.write(f"{targ_lin_vel:.3f},{targ_ang_vel:.3f}\n".encode("utf-8"))
        self.get_logger().debug(
            f"Set target velocity\nlinear: {targ_lin_vel}, angular: {targ_ang_vel}"
        )


def main(args=None):
    rclpy.init(args=args)
    pico_interface = PicoInterface()
    rclpy.spin(pico_interface)
    pico_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

