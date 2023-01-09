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
        assert -1 <= v <= 1, f"throttle value {v} incorrect."

    @validator("steering")
    def check_steering(cls, v):
        assert -1 <= v <= 1, f"steering value {v} incorrect."

    @validator("brake")
    def check_brake(cls, v):
        assert -1 <= v <= 1, f"brake value {v} incorrect."


class VehicleState(BaseModel):
    speed: float = 0.0
    is_auto: bool = False
    actuation: Actuation = Actuation()
    is_steering_left_limitor_on: bool = False
    is_steering_right_limitor_on: bool = False
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
        return data

    def read_state(self) -> VehicleState:
        data: List = self.p_read_state()
        state = VehicleState()
        state.speed = float(data[0])
        state.is_auto = float(data[1])
        state.actuation.throttle = float(data[2])
        state.actuation.steering = float(data[3])
        state.actuation.brake = float(data[4])
        return state

    def p_write(self, fields: List[Any]) -> None:
        output = ",".join([str(f) for f in fields])
        output = "<a," + output + ">"
        self.arduino.write(output.encode("utf-8"))

    def write_actuation(self, actuation: Actuation) -> None:
        fields = [actuation.throttle, actuation.steering, actuation.brake]
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
                state: dict = arduino.read_state()

                data = {"throttle": 1, "steering": 0.5, "brake": 0.0}

                arduino.write_actuation(Actuation(**data))

                print(f"Duration 6: {1/(time.time()-init)}")
                print()

            except Exception as e:
                print(e)
    finally:
        arduino.close()
