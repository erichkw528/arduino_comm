# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from nis import cat
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
import serial
from .arduino_comm import Arduino, VehicleState, Actuation, StatusEnum
from ackermann_msgs.msg import AckermannDriveStamped
from roar_msgs.msg import EgoVehicleControl, VehicleStatus
from typing import Optional


class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__("arduino_comm_node")
        self.cmd_sub_ = self.create_subscription(
            EgoVehicleControl, "/arduino/ego_vehicle_control", self.cmd_recvd, 10
        )
        self.vehicle_state_publisher_ = self.create_publisher(
            VehicleStatus, "/arduino/vehicle_status", 10
        )
        self.declare_parameter("write_period", 0.1)
        self.declare_parameter("read_period", 0.1)
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("timeout", 0.1)
        self.declare_parameter("write_timeout", 0.1)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("vehicle_status_header", "base_link")

        write_period: float = (
            self.get_parameter("write_period").get_parameter_value().double_value
        )
        read_period: float = (
            self.get_parameter("read_period").get_parameter_value().double_value
        )

        self.write_timer = self.create_timer(write_period, self.write_timer_callback)
        self.read_timer = self.create_timer(read_period, self.read_timer_callback)

        self.arduino: Optional[Arduino] = None
        try:
            self.arduino = Arduino(
                port=self.get_parameter("port").get_parameter_value().string_value,
                timeout=self.get_parameter("timeout")
                .get_parameter_value()
                .double_value,
                write_timeout=self.get_parameter("write_timeout")
                .get_parameter_value()
                .double_value,
                baudrate=self.get_parameter("baudrate")
                .get_parameter_value()
                .integer_value,
            )
        except Exception as e:
            self.get_logger().error(f"Unable to initialize arduino: {e}")
        self.state: Optional[VehicleState] = None
        self.msg: Optional[EgoVehicleControl] = None

    def cmd_recvd(self, msg: EgoVehicleControl):
        self.msg = msg

    def write_timer_callback(self):
        if self.arduino == None:
            self.get_logger().error("Not writing, Arduino is not initialized")
            return
        if self.msg:
            self._write_msg_to_arduino(self.msg)

    def read_timer_callback(self):
        curr_state: VehicleState = self._read_state_from_arduino()
        self._publish_vehicle_state(curr_state=curr_state)

    def _publish_vehicle_state(self, curr_state: Optional[VehicleState]):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = (
            self.get_parameter("vehicle_status_header")
            .get_parameter_value()
            .string_value
        )
        msg = VehicleStatus()
        if curr_state == None:
            self.get_logger().error(f"Vehicle status is abnormal: [{curr_state}]")
            msg.header = header
            self.vehicle_state_publisher_.publish(msg)
            return

        if curr_state.status != StatusEnum.NORMAL:
            self.get_logger().error(f"Vehicle status is abnormal: [{curr_state}]")
            return

        self.state = curr_state
        msg.angle = self.state.angle
        msg.is_auto = self.state.is_auto
        msg.actuation.throttle = self.state.actuation.throttle
        msg.actuation.steer = self.state.actuation.steering
        msg.actuation.brake = self.state.actuation.brake
        msg.actuation.reverse = self.state.actuation.reverse
        msg.status = 1
        self.vehicle_state_publisher_.publish(msg)

    def _read_state_from_arduino(self) -> Optional[VehicleState]:
        try:
            if self.arduino == None:
                return VehicleState(status=StatusEnum.UNKNOWN_ERROR)
            state: VehicleState = self.arduino.read_state()
            return state
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            return VehicleState(status=StatusEnum.UNKNOWN_ERROR)

    def _write_msg_to_arduino(self, msg: EgoVehicleControl):
        try:
            self.arduino.write_actuation(
                actuation=Actuation(
                    throttle=msg.throttle,
                    steering=msg.steer,
                    brake=msg.brake,
                    reverse=msg.reverse,
                )
            )
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def destroy_node(self):
        if self.arduino:
            self.arduino.write_actuation(
                actuation=Actuation(speed=0, steering=0, brake=0)
            )
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
