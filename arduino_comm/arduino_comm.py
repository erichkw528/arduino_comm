# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from time import sleep
import serial
from typing import List, Any, Dict
import json
import numpy as np
from pydantic import BaseModel, validator


class Actuation(BaseModel):
    throttle: float = 0.0
    steering: float = 0.0
    brake: float = 0.0

    @validator("throttle")
    def check_throttle(cls, v):
        assert 0 <= v <= 1, f"throttle value {v} incorrect."
        return v

    @validator("steering")
    def check_steering(cls, v):
        assert -1 <= v <= 1, f"steering value {v} incorrect."
        return v

    @validator("brake")
    def check_brake(cls, v):
        assert -1 <= v <= 1, f"brake value {v} incorrect."
        return v


class TargetAction(BaseModel):
    speed: float = 0.0
    steering: float = 0.0
    brake: float = 0.0

    @validator("speed")
    def check_throttle(cls, v):
        assert 0 <= v <= 1, f"speed value {v} incorrect."
        return v

    @validator("steering")
    def check_steering(cls, v):
        assert -1 <= v <= 1, f"steering value {v} incorrect."
        return v

    @validator("brake")
    def check_brake(cls, v):
        assert -1 <= v <= 1, f"brake value {v} incorrect."
        return v


class VehicleState(BaseModel):
    speed: float = 0.0
    is_auto: bool = False
    actuation: Actuation = Actuation()
    target_action: TargetAction = TargetAction()
    angle: float = 0.0

    class Config:
        arbitrary_types_allowed = True


class Arduino:
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: float = 0.1,
        write_timeout: float = 0.1,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.arduino = serial.Serial(
            port=port, baudrate=baudrate, timeout=timeout, write_timeout=write_timeout
        )
        self.initialize()

    def initialize(self):
        self.arduino.close()
        self.arduino.open()
        self.arduino.flushOutput()
        self.arduino.flushInput()
        sleep(0.1)

    def p_read_state(self) -> List:
        self.arduino.write(b"<s>")
        buf: bytes = self.arduino.read_until(b"\r\n")
        data = buf.decode("utf-8").strip().split(",")
        print(data)
        return data

    def read_state(self) -> VehicleState:
        data: List = self.p_read_state()
        state = VehicleState()
        state.target_action.speed = data[0]
        state.target_action.steering = data[1]
        state.target_action.brake = data[2]
        state.actuation.throttle = data[3]
        state.actuation.steering = data[4]
        state.actuation.brake = data[5]
        state.speed = data[6]
        state.is_auto = data[7]
        state.angle = data[8]

        return state

    def p_write(self, fields: List[Any]) -> None:
        output = ",".join([str(f) for f in fields])
        output = "<a," + output + ">"
        self.arduino.write(output.encode("utf-8"))

    def write_actuation(self, act: TargetAction) -> None:
        fields = [act.speed, act.steering, act.brake]
        self.p_write(fields=fields)

    def close(self):
        self.arduino.close()


if __name__ == "__main__":
    import time

    arduino = Arduino(
        port="/dev/ttyACM0", timeout=0.1, write_timeout=0.1, baudrate=115200
    )
    try:
        while True:
            try:
                init = time.time()

                # arduino.write_actuation(
                #     TargetAction(speed=0.5, steering=0.5, brake=0.5)
                # )
                state = arduino.read_state()
                # print(state)
                # print()

            except Exception as e:
                print(e)
    finally:
        arduino.close()
