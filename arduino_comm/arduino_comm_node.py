# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from nis import cat
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial
from .arduino_comm import Arduino, VehicleState, Actuation
from ackermann_msgs.msg import AckermannDriveStamped
from typing import Optional


class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__("arduino_comm_node")
        self.cmd_sub_ = self.create_subscription(
            AckermannDriveStamped, "/manual_control", self.cmd_recvd, 10
        )
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.arduino = Arduino(
            port="/dev/ttyACM0", timeout=0.1, write_timeout=0.1, baudrate=115200
        )

        self.msg: Optional[AckermannDriveStamped] = None

    def cmd_recvd(self, msg: AckermannDriveStamped):
        self.msg = msg

    def timer_callback(self):
        if self.msg:
            try:
                speed = self.msg.drive.speed
                angle = self.msg.drive.steering_angle

                self.arduino.write_actuation(
                    actuation=Actuation(
                        speed=speed,
                        steering=angle,
                        brake=0,
                    )
                )
                print(f"Sent: Speed-> {speed}, angle -> {angle}")
                state: VehicleState = self.arduino.read_state()
                print(
                    f"Recv: Speed->{state.actuation.speed}, angle -> {state.actuation.steering}"
                )
                print()
            except Exception as e:
                print(e)

    def destroy_node(self):
        self.arduino.write_actuation(actuation=Actuation(speed=0, steering=0, brake=0))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
