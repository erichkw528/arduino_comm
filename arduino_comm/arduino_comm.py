# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from time import sleep
import serial
from typing import List, Any


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

    def read_state(self, fields_types: List[type]) -> List[Any]:
        self.arduino.write(b"<s>")
        buf: bytes = self.arduino.read_until(b"\r\n")
        data = buf.decode("utf-8").strip()[1:-1].split(",")  # get rid of <>
        if len(data) != len(fields_types):
            raise Exception(f"fields is not equal to {len(fields_types)}")

        result = [fields_types[i](data[i]) for i in range(len(fields_types))]
        return result

    def write(self, fields: List[Any]) -> None:
        output = ",".join([str(f) for f in fields])
        output = "<a," + output + ">"
        self.arduino.write(output.encode("utf-8"))


if __name__ == "__main__":
    arduino = Arduino(port="/dev/cu.usbmodem11301", timeout=0.1, write_timeout=0.1)
    try:
        while True:
            try:
                prev = time.time()
                result = arduino.read_state([float, int, int, int, int])
                now = time.time()
                print(f"{1 / (now-prev)} output result: ", result)
            except Exception as e:
                pass
    finally:
        arduino.close()
