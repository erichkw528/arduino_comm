# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from time import sleep
import serial
from typing import List, Any, Dict
import json
import numpy as np
from pydantic import BaseModel, validator
import struct


class Actuation(BaseModel):
    throttle: float = 0.0
    steering: float = 0.0
    brake: float = 0.0
    reverse: bool = False

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


class VehicleState(BaseModel):
    is_auto: bool = False
    actuation: Actuation = Actuation()
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

    def p_read_state(self) -> bytes:
        self.arduino.write(b"<s>\0")
        buf: bytes = self.arduino.read_until(b"\r\n")
        return buf

    def read_state(self) -> VehicleState:
        data: bytes = self.p_read_state()

        data = data[:-2]
        if len(data) > 2:
            is_auto = bool(struct.unpack("b", data[:1])[0])
            throttle = struct.unpack("f", data[1:5])[0]
            steering = struct.unpack("f", data[5:9])[0]
            brake = struct.unpack("f", data[9:13])[0]
            angle = struct.unpack("f", data[13:17])[0]
        else:
            print(f"Something is wrong -- data: {data}")
        is_auto = struct.unpack("f", data[:4])[0]

        state = VehicleState()
        state.actuation.throttle = data[0]
        state.actuation.steering = data[1]
        state.actuation.brake = data[2]
        state.is_auto = data[3]
        state.angle = data[4]

        return state

    def p_write(self, fields: List[Any]) -> None:
        output = ",".join([str(f) for f in fields])
        output = "<a," + output + ">"
        self.arduino.write(output.encode("utf-8"))

    def write_actuation(self, act: Actuation) -> None:
        fields = [act.throttle, act.steering, act.brake]
        self.p_write(fields=fields)

    def close(self):
        self.write_actuation(Actuation(throttle=0.0, steering=0.0, brake=0.0))
        self.arduino.close()

    def reset(self):
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
                #     Actuation(throttle=0.5, steering=0.5, brake=0.5)
                # )
                state = arduino.read_state()
                # print(state)
                # print()

            except Exception as e:
                print(e)
    finally:
        arduino.close()
