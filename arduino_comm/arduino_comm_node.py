from nis import cat
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial 
from .arduino_comm import Arduino

class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.arduino = Arduino(port="/dev/ttyACM0")

    def timer_callback(self):
        try:
            data = self.arduino.read_state(fields_types = [float, int, int, int])
            print(data)
            self.arduino.write([1600,1400,1500])
        except Exception as e:
            print(e)
    def destroy_node(self):
        self.arduino.write([1500,1500,1500])
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()