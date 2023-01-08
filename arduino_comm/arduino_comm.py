# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from time import sleep
import serial
from typing import List, Any, Dict
import json

from pydantic import BaseModel


class Actuation:
    throttle: float = 0.0
    steering: float = 0.0
    brake: float = 0.0


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

    def read_state(self) -> Dict:
        self.arduino.write(b"<s>")
        buf: bytes = self.arduino.read_until(b"\r\n")
        data = buf.decode("utf-8").strip()
        print(data)
        return data

    def write(self, fields: List[Any]) -> None:
        output = ",".join([str(f) for f in fields])
        output = "<a," + output + ">"
        self.arduino.write(output.encode("utf-8"))

    def close(self):
        self.arduino.close()


if __name__ == "__main__":
    import time

    arduino = Arduino(port="/dev/ttyACM0", timeout=0.1, write_timeout=0.1)
    try:
        while True:
            try:
                prev = time.time()
                state: dict = arduino.read_state()
                now = time.time()
                print(f"{1 / (now-prev)} ", state)
            except Exception as e:
                print(e)
    finally:
        arduino.close()
