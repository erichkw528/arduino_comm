# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from nis import cat
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial
from .arduino_comm import Arduino, VehicleState, Actuation


class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.arduino = Arduino(port="/dev/ttyACM0")

    def timer_callback(self):
        try:
            self.arduino.write_actuation(
                actuation=Actuation(throttle=0, steering=0, brake=0)
            )
            state: VehicleState = self.arduino.read_state()
        except Exception as e:
            print(e)

    def destroy_node(self):
        self.arduino.write_actuation(
            actuation=Actuation(throttle=0, steering=0, brake=0)
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArduinoCommNode()
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
