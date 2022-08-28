from time import sleep
import serial
from typing import List, Any 

class Arduino():
    def __init__(self, port:str, baudrate:int=9600, timeout:float=0.1, write_timeout:float=0.1) -> None:
        self.port = port 
        self.baudrate = baudrate
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=write_timeout)
        self.initialize() 
    
    def initialize(self):
        self.arduino.close()
        self.arduino.open()
        self.arduino.flushOutput()
        self.arduino.flushInput()
        sleep(0.1)
    
    def read_state(self, fields_types:List[type]) -> List[Any]:
        self.arduino.write(b'<s>')
        buf:bytes = self.arduino.read_until(b'\r\n')
        data = buf.decode('utf-8').strip()[1:-1].split(",") # get rid of <> 
        if len(data) == fields_types:
            raise Exception(f"fields is not equal to {len(fields_types)}")
            
        result = [fields_types[i](data[i]) for i in range(len(fields_types))]
        return result

    def write(self, fields:List[Any]) -> None:
        output = ",".join([str(f) for f in fields])
        output = "<a," + output + ">"
        self.arduino.write(output.encode('utf-8'))

